/*
  Print.h - Base class that provides print() and println()
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
*/

#ifndef Print_h
#define Print_h

#include <inttypes.h>
#include <stdio.h> // for size_t
#include <string.h>

//#include "Printable.h"

#define DEC 10
#define HEX 16
#define OCT 8
#ifdef BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
#undef BIN
#endif
#define BIN 2

class __FlashStringHelper;

class Print
{
  private:
    int write_error;
    template <typename T>
    uint8_t printNumber(T, uint8_t) __restrict;
    uint8_t printFloat(double, uint8_t) __restrict;
  protected:
    void setWriteError(int err = 1) __restrict { write_error = err; }
  public:
    Print() : write_error(0) {}
  
    int getWriteError() const __restrict { return write_error; }
    void clearWriteError() __restrict { setWriteError(0); }
  
    virtual uint8_t write(uint8_t) __restrict  = 0;

    template <size_t N>
    size_t write(const char(&__restrict str)[N]) __restrict
    {
      return write((const uint8_t * __restrict)str, N - 1);
    }
    size_t write(const char * __restrict str) __restrict
    {
      return write((const uint8_t * __restrict)str, strlen(str));
    }
    virtual size_t write(const uint8_t * __restrict buffer, size_t size) __restrict;
    size_t write(const char * __restrict buffer, size_t size) __restrict {
      return write((const uint8_t * __restrict)buffer, size);
    }

    // default to zero, meaning "a single write may block"
    // should be overriden by subclasses with buffering
    virtual unsigned int availableForWrite() __restrict { return 0; }

    size_t print(const __FlashStringHelper * __restrict) __restrict;
    template <size_t N>
    size_t print(const char(&__restrict str)[N]) __restrict
    {
      return write(str);
    }
    size_t print(const char * __restrict str) __restrict
    {
      return write(str);
    }
    size_t print(char * __restrict str) __restrict
    {
      return write((const char * __restrict)str);
    }

    uint8_t print(uint8_t, uint8_t = DEC) __restrict;
    uint8_t print(uint16_t, uint8_t = DEC) __restrict;
    uint8_t print(const __uint24 & __restrict, uint8_t = DEC) __restrict;
    uint8_t print(const uint32_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t print(const uint64_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t print(int16_t __restrict, uint8_t = DEC) __restrict;
    uint8_t print(const __int24 & __restrict, uint8_t = DEC) __restrict;
    uint8_t print(const int32_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t print(const int64_t & __restrict, uint8_t = DEC) __restrict;

    uint8_t print(char) __restrict;
    uint8_t print(const double & __restrict, uint8_t = 2) __restrict;
    uint8_t print(const float & __restrict a, uint8_t b = 2) __restrict
    {
      print(double(a), b);
    }
    size_t println(const __FlashStringHelper *) __restrict;
    template <size_t N>
    size_t println(const char(&__restrict str)[N]) __restrict
    {
      size_t n = print(str);
      n += println();
      return n;
    }
    size_t println(const char * __restrict str) __restrict
    {
      size_t n = print(str);
      n += println();
      return n;
    }
    size_t println(char * __restrict str) __restrict
    {
      size_t n = print(str);
      n += println();
      return n;
    }

    uint8_t println(char) __restrict;

    // integer types
    uint8_t println(uint8_t, uint8_t = DEC) __restrict;
    uint8_t println(uint16_t, uint8_t = DEC) __restrict;
    uint8_t println(const __uint24 & __restrict, uint8_t = DEC) __restrict;
    uint8_t println(const uint32_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t println(const uint64_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t println(int16_t, uint8_t = DEC) __restrict;
    uint8_t println(const __int24 & __restrict, uint8_t = DEC) __restrict;
    uint8_t println(const int32_t & __restrict, uint8_t = DEC) __restrict;
    uint8_t println(const int64_t & __restrict, uint8_t = DEC) __restrict;

    uint8_t println(const double & __restrict, uint8_t = 2) __restrict;
    uint8_t println(const float & __restrict a, uint8_t b = 2) __restrict
    {
      return println(a, b);
    }
    uint8_t println(void) __restrict;

    virtual void flush() __restrict { /* Empty implementation for backward compatibility */ }
};

#endif
