/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __DURATION_T__
#define __DURATION_T__

#include <tuna.h>

template <typename T>
struct _duration_t final
{
	static_assert(tuna::is_same<T, uint24> || tuna::is_same<T, uint32>, "duration_t can only be 24 or 32 bits");

	using type = T;

private:
	struct short_types final
	{
		using day_t = uint8;
		using hour_t = uint16;
		using min_t = uint24;
		using sec_t = uint24;
	};
	struct long_types final
	{
		using day_t = uint16;
		using hour_t = uint24;
		using min_t = uint32;
		using sec_t = uint32;
	};
	constexpr static auto typer()
	{
		if constexpr (tuna::is_same<T, uint24>)
		{
			return short_types{};
		}
		else
		{
			return long_types{};
		}
	}
	using types = decltype(typer());
public:

  /**
   * @brief Duration is stored in seconds
   */
	type value = 0;

  /**
   * @brief Constructor
   */
	_duration_t() = default;

  /**
   * @brief Constructor
   *
   * @param seconds The number of seconds
   */
	_duration_t(uint24 const &seconds) : value(seconds) {}

	_duration_t(uint32 const &seconds) : value(seconds) {}

  /**
   * @brief Equality comparison
   * @details Overloads the equality comparison operator
   *
   * @param value The number of seconds to compare to
   * @return True if both durations are equal
   */
  bool operator==(const type &value) const {
    return (this->value == value);
  }

  /**
   * @brief Inequality comparison
   * @details Overloads the inequality comparison operator
   *
   * @param value The number of seconds to compare to
   * @return False if both durations are equal
   */
  bool operator!=(const type &value) const {
    return ! this->operator==(value);
  }

  /**
   * @brief Formats the duration as years
   * @return The number of years
   */
  inline uint8 year() const {
    return this->day() / 365;
  }

  /**
   * @brief Formats the duration as days
   * @return The number of days
   */
  inline typename types::day_t day() const {
    return this->hour() / 24;
  }

  /**
   * @brief Formats the duration as hours
   * @return The number of hours
   */
  inline typename types::hour_t hour() const {
    return this->minute() / 60;
  }

  /**
   * @brief Formats the duration as minutes
   * @return The number of minutes
   */
  inline typename types::min_t minute() const {
    return this->second() / 60;
  }

  /**
   * @brief Formats the duration as seconds
   * @return The number of seconds
   */
  inline typename types::sec_t second() const {
    return this->value;
  }

  /**
   * @brief Formats the duration as a string
   * @details String will be formated using a "full" representation of duration
   *
   * @param buffer The array pointed to must be able to accommodate 21 bytes
   *
   * Output examples:
   *  123456789012345678901 (strlen)
   *  135y 364d 23h 59m 59s
   *  364d 23h 59m 59s
   *  23h 59m 59s
   *  59m 59s
   *  59s
   */
  void toString(char *buffer) const {
    int y = this->year(),
        d = this->day() % 365,
        h = this->hour() % 24,
        m = this->minute() % 60,
        s = this->second() % 60;

    if (y) sprintf_P(buffer, PSTR("%iy %id %ih %im %is"), y, d, h, m, s);
    else if (d) sprintf_P(buffer, PSTR("%id %ih %im %is"), d, h, m, s);
    else if (h) sprintf_P(buffer, PSTR("%ih %im %is"), h, m, s);
    else if (m) sprintf_P(buffer, PSTR("%im %is"), m, s);
    else sprintf_P(buffer, PSTR("%is"), s);
  }

  /**
  * @brief Formats the duration as a string
  * @details String will be formated using a "full" representation of duration
  *
  * @param buffer The array pointed to must be able to accommodate 21 bytes
  *
  * Output examples:
  *  123456789012345678901 (strlen)
  *  135y 364d 23h 59m 59s
  *  364d 23h 59m 59s
  *  23h 59m 59s
  *  59m 59s
  *  59s
  */
  template <uint8_t length>
  void toString(char (&buffer)[length]) const {
	  int y = this->year(),
		  d = this->day() % 365,
		  h = this->hour() % 24,
		  m = this->minute() % 60,
		  s = this->second() % 60;

	  if (y) snprintf_P(buffer, length, PSTR("%iy %id %ih %im %is"), y, d, h, m, s);
	  else if (d) snprintf_P(buffer, length, PSTR("%id %ih %im %is"), d, h, m, s);
	  else if (h) snprintf_P(buffer, length, PSTR("%ih %im %is"), h, m, s);
	  else if (m) snprintf_P(buffer, length, PSTR("%im %is"), m, s);
	  else snprintf_P(buffer, length, PSTR("%is"), s);
  }

  /**
   * @brief Formats the duration as a string
   * @details String will be formated using a "digital" representation of duration
   *
   * @param buffer The array pointed to must be able to accommodate 10 bytes
   *
   * Output examples:
   *  123456789 (strlen)
   *  99:59
   *  11d 12:33
   */
  uint8_t toDigital(char *buffer, bool with_days=false) const {
    uint16_t h = uint16_t(this->hour()),
             m = uint16_t(this->minute() % 60UL);
    if (with_days) {
      uint16_t d = this->day();
      sprintf_P(buffer, PSTR("%ud %02u:%02u"), d, h % 24, m);
      return d >= 10 ? 8 : 7;
    }
    else if (h < 100) {
      sprintf_P(buffer, PSTR("%02u:%02u"), h % 24, m);
      return 5;
    }
    else {
      sprintf_P(buffer, PSTR("%u:%02u"), h, m);
      return 6;
    }
  }
};

using duration_t = _duration_t<uint24>;
using duration32_t = _duration_t<uint32>;

#endif // __DURATION_T__
