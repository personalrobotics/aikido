#ifndef AIKIDO_UTIL_COMPILER_HPP_
#define AIKIDO_UTIL_COMPILER_HPP_

#if defined(__GNUC__) || defined(__GNUG__)

#define AIKIDO_COMPILER_GCC 1
#define AIKIDO_COMPILER_GCC_VERSION_AT_LEAST(x,y,z) \
  (__GNUC__ > x || (__GNUC__ >= x && \
  (__GNUC_MINOR__ > y || (__GNUC_MINOR__ >= y && \
  __GNUC_PATCHLEVEL__ >= z))))
#define AIKIDO_COMPILER_GCC_VERSION_AT_MOST(x,y,z) \
  (__GNUC__ < x || (__GNUC__ <= x && \
  (__GNUC_MINOR__ < y || (__GNUC_MINOR__ <= y && \
  __GNUC_PATCHLEVEL__ <= z))))
#define AIKIDO_COMPILER_CLANG 0
#define AIKIDO_COMPILER_MSVC 0

#elif defined(__clang__)

#define AIKIDO_COMPILER_GCC 0
#define AIKIDO_COMPILER_GCC_VERSION_AT_LEAST(x,y,z) 0
#define AIKIDO_COMPILER_GCC_VERSION_AT_MOST(x,y,z) 0
#define AIKIDO_COMPILER_CLANG 1
#define AIKIDO_COMPILER_MSVC 0

#elif defined(_MSC_VER)

#define AIKIDO_COMPILER_GCC 0
#define AIKIDO_COMPILER_GCC_VERSION_AT_LEAST(x,y,z) 0
#define AIKIDO_COMPILER_GCC_VERSION_AT_MOST(x,y,z) 0
#define AIKIDO_COMPILER_CLANG 0
#define AIKIDO_COMPILER_MSVC 1

#endif

#endif // AIKIDO_UTIL_COMPILER_HPP_
