#ifndef AIKIDO_UTIL_WARNING_HPP_
#define AIKIDO_UTIL_WARNING_HPP_

// We define convenient macros that can be used to suppress warnings for a
// specific code block rather than a whole project. This macros would be useful
// when you need to suppress warnings due to a compiler bug or call deprecated
// function for some reason (e.g., for backward compatibility) but don't want
// warnings.
//
// Example code:
//
// deprecated_function()  // warning
//
// AIKIDO_SUPPRESS_DEPRECATED_BEGIN
// deprecated_function()  // okay, no warning
// AIKIDO_SUPPRESS_DEPRECATED_END

#if defined(__GNUC__) || defined(__GNUG__)

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN                                       \
  _Pragma("GCC diagnostic push")                                               \
      _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#define AIKIDO_SUPPRESS_DEPRECATED_END _Pragma("GCC diagnostic pop")
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_BEGIN                               \
  _Pragma("GCC diagnostic push")                                               \
      _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_END _Pragma("GCC diagnostic pop")

#elif defined(__clang__)

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN                                       \
  _Pragma("clang diagnostic push")                                             \
      _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")
#define AIKIDO_SUPPRESS_DEPRECATED_END _Pragma("clang diagnostic pop")
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_BEGIN                               \
  _Pragma("clang diagnostic push")                                             \
      _Pragma("clang diagnostic ignored \"-Wmaybe-uninitialized\"")
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_END _Pragma("clang diagnostic pop")

#elif defined(_MSC_VER)

#define AIKIDO_SUPPRESS_DEPRECATED_BEGIN                                       \
  __pragma(warning(push)) __pragma(warning(disable : 4996))
#define AIKIDO_SUPPRESS_DEPRECATED_END __pragma(warning(pop))
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_BEGIN
#define AIKIDO_SUPPRESS_MAYBEUNINITIALIZED_END

#endif

#endif // AIKIDO_UTIL_WARNING_HPP_
