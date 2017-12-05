#pragma once

// no need for namespaces as macros don't have namespaces.

// TODO
// These are macros for this file. They will be used to populate constexpr values for regular code, and will be #undef'd
// outside of those contexts.

// Is the architecture a Harvard architecture?
#define __harvard 1

// Is the architecture a von Neumann architecture?
#define __von_neumann (!__harvard)
// ~TODO

// Marks the data as existing in program memory. Always prefer to use wrapper structures, and when avr-llvm is viable, __flash.
// This will not be defined as anything if the architecture being built for is not Harvard.
#if __harvard
# define __flashmem __attribute__((__progmem__))
#else
# define __flashmem
#endif

// Marks the function as being always inlined if possible.
#define __forceinline __attribute__((always_inline))

// Marks that this location in code is unreachable.
#define __unreachable __builtin_unreachable()
// Marks that the compiler can assume that it is guaranteed that the provided condition is true.
#define __assume(c) { if (!(c)) { __unreachable; } }
// Marks that the given branch is likely.
#define __likely(c) (__builtin_expect(c, true))
// Marks that the given branch is unlikely.
#define __unlikely(c) (__builtin_expect(c, false))
// Returns 'true' if the provided expression can be resolved at compile-time, otherwise false.
#define __is_constexpr(x) __builtin_constant_p(x)

// Creates a memory barrier, where all stores to memory must be completed.
#define __memorybarrier do { asm volatile("":::"memory"); } while (false)

// Specifies that a function does not return.
#define __noreturn __attribute__((noreturn))

// Returns the line number (uint32) where this is. When used as a default argument, it returns the line number of the caller.
#define __line uint32(__builtin_LINE())

// This is mainly used internally (prefer _p otherwise) to generate a progmem string from a const char * return.
// This will just return a regular const char * if the architecture being built for is not Harvard.
#if __harvard
# define __pgm_string(s) \
  ([](const char * const str = s) -> Tuna::flash_string { static constexpr const char __flashmem * const _str = str; return {_str}; }())
#else
# define __pgm_string(s) (s)
#endif

// Returns the name of the function. When used as a default argument, returns the name of the caller function. Stored in program memory.
#define __function_name __pgm_string(__builtin_FUNCTION())

// Returns the name of the source file. When used as a default argument, returns the name of the caller function's file. Stored in program memory.
#define __file_name __pgm_string(__builtin_FILE())

// Allocates memory on the stack. Call only in block scope, and the data will remain valid within scope. It is generally avoided due to issues
// with stack overflow and with memory going out of scope (particularly with inlining).
#define __alloca(size) __builtin_alloca(size)

// Defines a symbol as an alias of another symbol.
#define __alias(symbol) __attribute__((alias (#symbol)))

// Defines a symbol as a weak alias of another symbol.
#define __weak_alias(symbol) __attribute__((weak, alias (#symbol)))

// TODO assume_aligned

// Marks a function as being unlikely to be executed.
#define __cold __attribute__((cold))

// Marks a function as being likely to be executed.
#define __hot __attribute__((hot))

// Marks a function as being 'const' - that is, does not examine any values except their arguments (no dereferencing of pointers or references), and has no side-effects.
#define __const __attribute__((const))

// Marks a function as being 'pure' - that is, does not examine any values except their arguments (no dereferencing of pointers or references) and global values, and has no side-effects.
#define __pure __attribute__((pure))

// Marks a function as being deprecated
#define __deprecated(why) __attribute__((deprecated (#why)))

// Forces the function to have all code within it to be inlined, if possible.
#define __flatten __attribute__((flatten))

// Marks the function as an interrupt handler.
#define __interrupt __attribute__((interrupt))

// Marks the function as a signal handler. Equivalent to __interrupt except on AVR, where 'sei' is not emitted.
#define __signal __attribute__((signal))

// Marks the function as being ineligible for inlining. Avoid using this.
#define __no_inline __attribute__((noinline))

// Marks the function as never returning a null pointer.
#define __non_null __attribute__((returns_nonnull))

// Marks the function as emitting a warning when its return value is unused.
#define __warn_unused __attribute__((warn_unused_result))

// A macro which wraps a constexpr test. Has to be a macro because there's not a good way to do this otherwise.
// Intellisense has a different variant due to differences in behavior between gcc and msvc.
#define constexpr_test(name, ...) \
  static_assert(name(__VA_ARGS__) || true, "Function is not constexpr: " #name);

// Macro wrapper for static_assert, which _only_ exists when not building for Intellisense.
// Also adds a slightly easier-to-use macro for compile-only.
#if !__INTELLISENSE__
# define __compiling 1
# define c_static_assert(...) static_assert(__VA_ARGS__)
#else
# define __compiling 0
# define c_static_assert(...)
#endif // __INTELLISENSE__

// A few extra helpers for intellisense to limit IDE errors
#if !__compiling
# undef __assume
# define __assume(expression)

# undef __memorybarrier
# define __memorybarrier 0
#endif //!__compiling

// TODO leaf, maybe
