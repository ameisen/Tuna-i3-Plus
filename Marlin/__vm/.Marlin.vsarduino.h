#if defined(__INTELLISENSE__)

#pragma once

#ifdef __cplusplus
#	undef __cplusplus
#endif
#define __cplusplus 201703L
#define __inline__
#define __asm__(...)
#define __extension__
#define __volatile__(...)

#define volatile(...) 
#define __builtin_va_start
#define __builtin_va_end
#define __attribute__(...)
#define NOINLINE
#define prog_void
#define PGM_VOID_P void *


#ifndef __builtin_constant_p
	#define __builtin_constant_p
#endif
#ifndef __builtin_strlen
	#define __builtin_strlen
#endif


#define NEW_H
typedef void *__builtin_va_list;
//extern "C" void __cxa_pure_virtual() {;}

typedef int div_t;
typedef int ldiv_t;


typedef void *__builtin_va_list;
//extern "C" void __cxa_pure_virtual() {;}



//#include <arduino.h>
//#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const char *)(string_literal))


#define pgm_read_byte(address_short) uint8_t() 
#define pgm_read_word(address_short) uint16_t() 
#define pgm_read_dword(address_short) uint32_t()
#define pgm_read_float(address_short) float()
#define pgm_read_ptr(address_short)   short()

//#include "Marlin.ino"
#endif
