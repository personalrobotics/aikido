#ifndef AIKIDO_COMMON_DEPRECATED_HPP_
#define AIKIDO_COMMON_DEPRECATED_HPP_

#include <dart/config.hpp>

// NOTE: Deprecated macros are used for backward compatibility between different
// minor versions of AIKIDO. Deprecated API can be removed in every major
// version up.

#if defined(__GNUC__) || defined(__clang__)
#define AIKIDO_DEPRECATED(version) __attribute__((deprecated))
#elif defined(_MSC_VER)
#define AIKIDO_DEPRECATED(version) __declspec(deprecated)
#else
#define AIKIDO_DEPRECATED(version) ()
#endif

// We define two convenient macros that can be used to suppress
// deprecated-warnings for a specific code block rather than a whole project.
// This macros would be useful when you need to call deprecated function for
// some reasons (e.g., for backward compatibility) but don't want warnings.
//
// Example code:
//
// deprecated_function()  // warning
//
// AIKIDO_SUPPRESS_DEPRECATED_BEGIN
// deprecated_function()  // okay, no warning
// AIKIDO_SUPPRESS_DEPRECATED_END
//
#if defined(__GNUC__) || defined(__GNUG__)

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN                                       \
  _Pragma("GCC diagnostic push")                                               \
      _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")

#define AIKIDO_SUPPRESS_DEPRECATED_END _Pragma("GCC diagnostic pop")

#elif defined(__clang__)

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN                                       \
  _Pragma("clang diagnostic push")                                             \
      _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")

#define AIKIDO_SUPPRESS_DEPRECATED_END _Pragma("clang diagnostic pop")

#else

#warning "AIKIDO is being built by unsupported compiler."

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN
#define AIKIDO_SUPPRESS_DEPRECATED_END

#endif

#endif // AIKIDO_COMMON_DEPRECATED_HPP_
