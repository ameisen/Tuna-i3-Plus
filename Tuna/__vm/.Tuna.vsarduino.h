#pragma once

#if defined(__INTELLISENSE__)

#undef __cplusplus
#define __cplusplus 201703L
#undef __inline__
#define __inline__
#undef __asm__
#define __asm__(...)
#undef __extension__
#define __extension__
#undef __volatile__
#define __volatile__(...)

#undef volatile
#define volatile(...) 
#undef __builtin_va_start
#define __builtin_va_start
#undef __builtin_va_end
#define __builtin_va_end
#undef __attribute__
#define __attribute__(...)
#undef NOINLINE
#define NOINLINE
#undef prog_void
#define prog_void

#undef PGM_VOID_P
using PGM_VOID_P = void *;


#undef __builtin_constant_p
#define __builtin_constant_p
#undef __builtin_strlen
#define __builtin_strlen

#undef NEW_H
#define NEW_H

#undef __builtin_va_list
using __builtin_va_list = void *;

#undef div_t
using div_t = int;
#undef ldiv_t
using ldiv_t = int;

#undef __builtin_va_list
using __builtin_va_list = void *;



//#include <arduino.h>
//#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const char *)(string_literal))


#undef pgm_read_byte
#undef pgm_read_word
#undef pgm_read_dword
#undef pgm_read_float
#undef pgm_read_ptr

#define pgm_read_byte(address_short) uint8_t{} 
#define pgm_read_word(address_short) uint16_t{}
#define pgm_read_dword(address_short) uint32_t{}
#define pgm_read_float(address_short) float{}
#define pgm_read_ptr(address_short) short{}

//#include "Marlin.ino"
#endif
