/*
 Print.cpp - Base class that provides print() and println()
 Copyright (c) 2008 David A. Mellis.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 Modified 23 November 2006 by David A. Mellis
 Modified 03 August 2015 by Chuck Todd
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Arduino.h"

#include "Print.h"

// Public Methods //////////////////////////////////////////////////////////////

/* default implementation: may be overridden */
size_t Print::write(const uint8_t * __restrict buffer, size_t size) __restrict
{
  size_t n = 0;
  while (size--) {
    if (write(*buffer++)) n++;
    else break;
  }
  return n;
}

size_t Print::print(const __FlashStringHelper * __restrict ifsh) __restrict
{
  PGM_P __restrict p = reinterpret_cast<PGM_P __restrict>(ifsh);
  size_t n = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    if (write(c)) n++;
    else break;
  }
  return n;
}

uint8_t Print::print(char c) __restrict
{
  return write(c);
}

uint8_t Print::print(const double & __restrict n, uint8_t digits) __restrict
{
  return printFloat(n, digits);
}

uint8_t Print::print(uint8_t n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(uint16_t n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const __uint24 & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const uint32_t & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const uint64_t & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(int16_t n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const __int24 & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const int32_t & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

uint8_t Print::print(const int64_t & __restrict n, uint8_t base) __restrict
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

size_t Print::println(const __FlashStringHelper * __restrict ifsh) __restrict
{
  size_t n = print(ifsh);
  n += println();
  return n;
}

uint8_t Print::println(void) __restrict
{
  return write("\r\n");
}

uint8_t Print::println(char c) __restrict
{
  uint8_t n = print(c);
  n += println();
  return n;
}

uint8_t Print::println(const double & __restrict num, uint8_t digits) __restrict
{
  uint8_t n = print(num, digits);
  n += println();
  return n;
}

uint8_t Print::println(uint8_t num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(uint16_t num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const __uint24 & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const uint32_t & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const uint64_t & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(int16_t num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const __int24 & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const int32_t & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

uint8_t Print::println(const int64_t & __restrict num, uint8_t base) __restrict
{
  uint8_t n = print(num, base);
  n += println();
  return n;
}

// Private Methods /////////////////////////////////////////////////////////////

template <typename T>
uint8_t Print::printNumber(T n, uint8_t base) __restrict
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    char c = n % base;
    n /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  return write(str);
}

uint8_t Print::printFloat(double number, uint8_t digits) __restrict
{ 
  uint8_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
  
  // Handle negative numbers
  if (number < 0.0)
  {
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print('.'); 
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)(remainder);
    n += print(toPrint);
    remainder -= toPrint; 
  } 
  
  return n;
}
