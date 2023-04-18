/**
 * Arduino DSMR parser.
 *
 * This software is licensed under the MIT License.
 *
 * Copyright (c) 2015 Matthijs Kooijman <matthijs@stdin.nl>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Field parsing functions
 */

#pragma once

#include "util.h"
#include "parser.h"

/*
#ifndef DSMR_GAS_MBUS_ID
#define DSMR_GAS_MBUS_ID 1
#endif
#ifndef DSMR_WATER_MBUS_ID
#define DSMR_WATER_MBUS_ID 2
#endif
#ifndef DSMR_THERMAL_MBUS_ID
#define DSMR_THERMAL_MBUS_ID 3
#endif
#ifndef DSMR_SUB_MBUS_ID
#define DSMR_SUB_MBUS_ID 4
#endif
*/

namespace dsmr
{

  /**
 * Superclass for data items in a P1 message.
 */
  template <typename T>
  struct ParsedField
  {
    template <typename F>
    void apply(F &f) { f.apply(*static_cast<T *>(this)); }
    // By defaults, fields have no unit
    static const char *unit() { return ""; }
  };

  template <typename T, size_t minlen, size_t maxlen>
  struct StringField : ParsedField<T>
  {
    ParseResult<void> parse(const char *str, const char *end)
    {
      ParseResult<String> res = StringParser::parse_string(minlen, maxlen, str, end);
      if (!res.err)
        static_cast<T *>(this)->val() = res.result;
      return res;
    }
  };

  // A timestamp is essentially a string using YYMMDDhhmmssX format (where
  // X is W or S for wintertime or summertime). Parsing this into a proper
  // (UNIX) timestamp is hard to do generically. Parsing it into a
  // single integer needs > 4 bytes top fit and isn't very useful (you
  // cannot really do any calculation with those values). So we just parse
  // into a string for now.
  template <typename T>
  struct TimestampField : StringField<T, 13, 13>
  {
  };

  // Value that is parsed as a three-decimal float, but stored as an
  // integer (by multiplying by 1000). Supports val() (or implicit cast to
  // float) to get the original value, and int_val() to get the more
  // efficient integer value. The unit() and int_unit() methods on
  // FixedField return the corresponding units for these values.
  struct FixedValue
  {
    operator float() { return val(); }
    float val() { return _value / 1000.0; }
    uint32_t int_val() { return _value; }

    uint32_t _value;
  };

  // Floating point numbers in the message never have more than 3 decimal
  // digits. To prevent inefficient floating point operations, we store
  // them as a fixed-point number: an integer that stores the value in
  // thousands. For example, a value of 1.234 kWh is stored as 1234. This
  // effectively means that the integer value is the value in Wh. To allow
  // automatic printing of these values, both the original unit and the
  // integer unit is passed as a template argument.
  template <typename T, const char *_unit, const char *_int_unit>
  struct FixedField : ParsedField<T>
  {
    ParseResult<void> parse(const char *str, const char *end)
    {
      // Check if the value is a float value, plus its expected unit type.
      ParseResult<uint32_t> res_float = NumParser::parse(3, _unit, str, end);
      if (!res_float.err) {
        static_cast<T *>(this)->val()._value = res_float.result;
        return res_float;
      }
      // If not, then check for an int value, plus its expected unit type.
      // This accomodates for some smart meters that publish int values instead
      // of floats. E.g. most meters would publish "1-0:1.8.0(000441.879*kWh)",
      // but some use "1-0:1.8.0(000441879*Wh)" instead.
      ParseResult<uint32_t> res_int = NumParser::parse(0, _int_unit, str, end);
      if (!res_int.err) {
        static_cast<T *>(this)->val()._value = res_int.result;
        return res_int;
      }
      // If not, then return the initial error result for the float parsing step.
      return res_float;
    }

    static const char *unit() { return _unit; }
    static const char *int_unit() { return _int_unit; }
  };

  struct TimestampedFixedValue : public FixedValue
  {
    String timestamp;
  };

  // Some numerical values are prefixed with a timestamp. This is simply
  // both of them concatenated, e.g. 0-1:24.2.1(150117180000W)(00473.789*m3)
  template <typename T, const char *_unit, const char *_int_unit>
  struct TimestampedFixedField : public FixedField<T, _unit, _int_unit>
  {
    ParseResult<void> parse(const char *str, const char *end)
    {
      // First, parse timestamp
      ParseResult<String> res = StringParser::parse_string(13, 13, str, end);
      if (res.err)
        return res;

      static_cast<T *>(this)->val().timestamp = res.result;

      // Which is immediately followed by the numerical value
      return FixedField<T, _unit, _int_unit>::parse(res.next, end);
    }
  };

  // A integer number is just represented as an integer.
  template <typename T, const char *_unit>
  struct IntField : ParsedField<T>
  {
    ParseResult<void> parse(const char *str, const char *end)
    {
      ParseResult<uint32_t> res = NumParser::parse(0, _unit, str, end);
      if (!res.err)
        static_cast<T *>(this)->val() = res.result;
      return res;
    }

    static const char *unit() { return _unit; }
  };

  // A RawField is not parsed, the entire value (including any
  // parenthesis around it) is returned as a string.
  template <typename T>
  struct RawField : ParsedField<T>
  {
    ParseResult<void> parse(const char *str, const char *end)
    {
      // Just copy the string verbatim value without any parsing
      concat_hack(static_cast<T *>(this)->val(), str, end - str);
      return ParseResult<void>().until(end);
    }
  };

  namespace fields
  {

    struct units
    {
      // These variables are inside a struct, since that allows us to make
      // them constexpr and define their values here, but define the storage
      // in a cpp file. Global const(expr) variables have implicitly
      // internal linkage, meaning each cpp file that includes us will have
      // its own copy of the variable. Since we take the address of these
      // variables (passing it as a template argument), this would cause a
      // compiler warning. By putting these in a struct, this is prevented.
      static constexpr char none[] = "";
      static constexpr char kWh[] = "kWh";
      static constexpr char Wh[] = "Wh";
      static constexpr char kW[] = "kW";
      static constexpr char W[] = "W";
      static constexpr char V[] = "V";
      static constexpr char mV[] = "mV";
      static constexpr char A[] = "A";
      static constexpr char mA[] = "mA";
      static constexpr char m3[] = "m3";
      static constexpr char dm3[] = "dm3";
      static constexpr char GJ[] = "GJ";
      static constexpr char MJ[] = "MJ";
      static constexpr char kvar[] = "kvar";
      static constexpr char kvarh[] = "kvarh";
      static constexpr char Hz[] = "Hz";
    };

//    const uint8_t GAS_MBUS_ID = DSMR_GAS_MBUS_ID;
//    const uint8_t WATER_MBUS_ID = DSMR_WATER_MBUS_ID;
//    const uint8_t THERMAL_MBUS_ID = DSMR_THERMAL_MBUS_ID;
//    const uint8_t SUB_MBUS_ID = DSMR_SUB_MBUS_ID;

#define DEFINE_FIELD(fieldname, value_t, obis, field_t, field_args...) \
  struct fieldname : field_t<fieldname, ##field_args>                  \
  {                                                                    \
    value_t fieldname;                                                 \
    bool fieldname##_present = false;                                  \
    static constexpr ObisId id = obis;                                 \
    static constexpr char name[] = #fieldname;                         \
    value_t &val() { return fieldname; }                               \
    bool &present() { return fieldname##_present; }                    \
  }

    /* Meter identification. This is not a normal field, but a
 * specially-formatted first line of the message */
    DEFINE_FIELD(identification, String, ObisId(255, 255, 255, 255, 255, 255), RawField);

/*_____fields____*/
    
    /* Date-time stamp of the P1 message */
    DEFINE_FIELD(timestamp, String, ObisId(0, 0, 1, 0, 0), TimestampField);

    /* Equipment identifier */
    DEFINE_FIELD(equipment_id, String, ObisId(0, 0, 96, 1, 0), StringField, 0, 96);

    /* Meter Reading electricity delivered to client (Special for Lux) in 0,001 kWh */
    DEFINE_FIELD(energy_delivered_lux, FixedValue, ObisId(1, 0, 1, 8, 0), FixedField, units::kWh, units::Wh);
    /* Meter Reading electricity delivered to client (Tariff 1) in 0,001 kWh */
    DEFINE_FIELD(energy_delivered_tariff1, FixedValue, ObisId(1, 0, 1, 8, 1), FixedField, units::kWh, units::Wh);
    /* Meter Reading electricity delivered to client (Tariff 2) in 0,001 kWh */
    DEFINE_FIELD(energy_delivered_tariff2, FixedValue, ObisId(1, 0, 1, 8, 2), FixedField, units::kWh, units::Wh);
    /* Meter Reading electricity delivered by client (Special for Lux) in 0,001 kWh */
    DEFINE_FIELD(energy_returned_lux, FixedValue, ObisId(1, 0, 2, 8, 0), FixedField, units::kWh, units::Wh);
    /* Meter Reading electricity delivered by client (Tariff 1) in 0,001 kWh */
    DEFINE_FIELD(energy_returned_tariff1, FixedValue, ObisId(1, 0, 2, 8, 1), FixedField, units::kWh, units::Wh);
    /* Meter Reading electricity delivered by client (Tariff 2) in 0,001 kWh */
    DEFINE_FIELD(energy_returned_tariff2, FixedValue, ObisId(1, 0, 2, 8, 2), FixedField, units::kWh, units::Wh);

    DEFINE_FIELD(total_imported_energy, FixedValue, ObisId(1, 0, 3, 8, 0), FixedField, units::kvarh, units::kvarh);
    DEFINE_FIELD(total_exported_energy, FixedValue, ObisId(1, 0, 4, 8, 0), FixedField, units::kvarh, units::kvarh);

    /* Tariff indicator electricity. The tariff indicator can also be used
     * to switch tariff dependent loads e.g boilers. This is the responsibility of the P1 user */
    DEFINE_FIELD(electricity_tariff, String, ObisId(0, 0, 96, 14, 0), StringField, 4, 4);

    /* Actual electricity power delivered (+P) in 1 Watt resolution */
    DEFINE_FIELD(power_delivered, FixedValue, ObisId(1, 0, 1, 7, 0), FixedField, units::kW, units::W);
    /* Actual electricity power received (-P) in 1 Watt resolution */
    DEFINE_FIELD(power_returned, FixedValue, ObisId(1, 0, 2, 7, 0), FixedField, units::kW, units::W);

    /* The actual threshold Electricity in kW. Removed in 4.0.7 / 4.2.2 / 5.0 */
    DEFINE_FIELD(electricity_threshold, FixedValue, ObisId(0, 0, 17, 0, 0), FixedField, units::kW, units::W);

    /* Text message max 2048 characters (Note: Spec says 1024 in comment and
     * 2048 in format spec, so we stick to 2048). */
    DEFINE_FIELD(message_long, String, ObisId(0, 0, 96, 13, 0), StringField, 0, 2048);

    /* Instantaneous voltage L1 in 0.1V resolution (Note: Spec says V
     * resolution in comment, but 0.1V resolution in format spec. Added in 5.0) */
    DEFINE_FIELD(voltage_l1, FixedValue, ObisId(1, 0, 32, 7, 0), FixedField, units::V, units::mV);
    /* Instantaneous voltage L2 in 0.1V resolution (Note: Spec says V
     * resolution in comment, but 0.1V resolution in format spec. Added in 5.0) */
    DEFINE_FIELD(voltage_l2, FixedValue, ObisId(1, 0, 52, 7, 0), FixedField, units::V, units::mV);
    /* Instantaneous voltage L3 in 0.1V resolution (Note: Spec says V
     * resolution in comment, but 0.1V resolution in format spec. Added in 5.0) */
    DEFINE_FIELD(voltage_l3, FixedValue, ObisId(1, 0, 72, 7, 0), FixedField, units::V, units::mV);

    /* Instantaneous current L1 in A resolution */
    DEFINE_FIELD(current_l1, FixedValue, ObisId(1, 0, 31, 7, 0), FixedField, units::A, units::mA);
    /* Instantaneous current L2 in A resolution */
    DEFINE_FIELD(current_l2, FixedValue, ObisId(1, 0, 51, 7, 0), FixedField, units::A, units::mA);
    /* Instantaneous current L3 in A resolution */
    DEFINE_FIELD(current_l3, FixedValue, ObisId(1, 0, 71, 7, 0), FixedField, units::A, units::mA);

/****    ADDED OBIS CODES    ****/
    
    /* cumulativeActiveEnergyCombined; Absolute active energy (A+) total [kWh] */
    DEFINE_FIELD(energy_combined_total, FixedValue, ObisId(1, 0, 15, 8, 0), FixedField, units::kWh, units::Wh);    

    /* currentLimitationLimit1; / maximumCurrentL1 (mA) */
    DEFINE_FIELD(maximum_current_l1, FixedValue, ObisId(1, 0, 31, 4, 0), FixedField, units::A, units::mA);
    /* currentLimitationLimit1; / maximumCurrentL2 (mA)  */
    DEFINE_FIELD(maximum_current_l2, FixedValue, ObisId(1, 0, 51, 4, 0), FixedField, units::A, units::mA);
    /* currentLimitationLimit1; / maximumCurrentL3 (mA)  */
    DEFINE_FIELD(maximum_current_l3, FixedValue, ObisId(1, 0, 71, 4, 0), FixedField, units::A, units::mA);
    
    /* frequency; Frequency [Hz]  */
    DEFINE_FIELD(frequency, FixedValue, ObisId(1, 0, 14, 7, 0), FixedField, units::Hz);
    
    /* momentaryPowerFactor; Instantaneous power factor (in rawfileld, but probably should use other struct */
    DEFINE_FIELD(power_factor, String, ObisId(1, 0, 13, 7, 0), RawField);
    /* momentaryPowerFactorL1; Instantaneous power factor in phase L1 (in rawfileld, but probably should use other struct */
    DEFINE_FIELD(power_factor_l1, String, ObisId(1, 0, 33, 7, 0), RawField);
    /* momentaryPowerFactorL2; Instantaneous power factor in phase L2 (in rawfileld, but probably should use other struct */
    DEFINE_FIELD(power_factor_l2, String, ObisId(1, 0, 53, 7, 0), RawField);
    /* momentaryPowerFactorL3; Instantaneous power factor in phase L3 (in rawfileld, but probably should use other struct */
    DEFINE_FIELD(power_factor_l3, String, ObisId(1, 0, 73, 7, 0), RawField);
    
     /* Datas on the end of the last month. Text message max 2048 characters */
    DEFINE_FIELD(monthly_datas, String, ObisId(0, 0, 98, 1, 0), StringField, 0, 2048);

    /* COSEM_logical_device_name (string) */
    DEFINE_FIELD(COSEM_logical_device_name, String, ObisId(0, 0, 42, 0, 0), StringField, 0, 64);
    
    /* Megszakító státusz ??? */
    DEFINE_FIELD(breaker_status, String, ObisId(0, 0, 96, 50, 68), StringField, 0, 2048);
    
  } // namespace fields

} // namespace dsmr
