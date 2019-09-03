# AIKIDO Style Guide #

The code in this library generally follows the same coding conventions
as [DART](https://github.com/dartsim/dart).

* [C++ Style](#c-style)
  * [C++ Header Style](#header-style)
  * [C++ Source Style](#source-style)
  * [Autoformatting using ClangFormat](#autoformatting-using-clangformat)
* [Python Style](#python-style)
* [CMake Style](#cmake-style)

## C++ Style

### Header Style

C++ headers should be contained in a subdirectory of `include/` that matches
their namespace, with the extension `.hpp`.

* Use **two-space** indentation
* Use **camelCase** function names
* Use **PascalCase** class names
* No "cuddled" braces!

```c++
#ifndef AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_  // Header guards must include the library, namespace, and source file names.
#define AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_

// Place all dependency includes at the top of the file.
// Use absolute paths, and place these at the top.
#include <stl_headers>
#include <library/headers.hpp>
#include "aikido/common/pointers.hpp"
#include "aikido/component_name/ExampleInterface.hpp"
#include "aikido/component_name/ExampleOtherInterface.hpp"

// Namespaces scopes should be one line each with "cuddled" braces.
namespace aikido {
namespace example {

// Use the following macro (defined in aikido/common/pointers.hpp) to declare
// STL "smart" pointers. Pointers should be declared ahead of the class so
// that the class itself can use the pointers.
AIKIDO_DECLARE_POINTERS(ExampleClass)

/// A required Doxygen comment description for this class. This can be extended
/// to include various useful details about the class, and can use the standard
/// Doxygen tag set to refer to other classes or documentation. It should use
/// the '///' style of block comment.
class ExampleClass
    : public ExampleInterface
    , public ExampleOtherInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Many classes that require Eigen will also need this macro

  /// Required brief description of constructor. This will often be as simple as:
  /// "Creates an instance of ExampleClass."
  ///
  /// \param[in] foo This is an example parameter description.
  /// \param[in] bar This is a longer example parameter description that needs
  /// to wrap across multiple lines.
  ExampleClass(std::unique_ptr<util::RNG> foo,
               const Eigen::Isometry3d& bar = Eigen::Isometry3d::Identity());

  ExampleClass(const ExampleClass& other);
  ExampleClass(ExampleClass&& other);

  ExampleClass& operator=(const ExampleClass& other);
  ExampleClass& operator=(ExampleClass&& other);

  // If a class should be non-copyable, it should explicitly delete the following:
  ExampleClass(const ExampleClass&) = delete;
  ExampleClass(ExampleClass&& other) = delete;
  ExampleClass& operator=(const ExampleClass& other) = delete;
  ExampleClass& operator=(ExampleClass&& other) = delete;
  
  // Classes should explicitly declare a default virtual destructor
  // if they do not declare one (unless marking a class as final).
  virtual ~ExampleClass() = default; 

  // Documentation inherited.  <-- Use this comment to indicate that the docstring of the interface method applies
  int exampleInterfaceFunction() const override;  // <-- Always explicitly `override` interface functions without `virtual`

  /// Required brief description of method.
  /// \note If a method has output parameters, they should be the last
  /// arguments.
  ///
  /// \param[in] a A description of a
  /// \param[in] b A description of b
  /// \param[out] out A description of out
  int exampleMethod(int a, int b, int* out) const;

private:
  std::unique_ptr<util::RNG> mExampleMember; // Member variables are prefixed with "m"
};

/// This is a template class description.
/// \tparam T This is an example template parameter description.
template <typename T>
class TemplatedExampleClass
{
public:
  /// Brief description.
  T exampleMethod();
};

} // namespace example
} // namespace aikido

// In certain cases, such as heavily templated code, implementations must be included
// in headers. In this case, a "detail" header should be created in the "./detail"
// subdirectory with the same name as the main header file, but an "-impl" suffix.
// Private declarations in this header can use a "detail" sub-namespace.
#include "aikido/component_name/detail/ExampleClass-impl.hpp"

#endif // AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_
```

```c++
#ifndef AIKIDO_EXAMPLE_DETAIL_EXAMPLECLASS_IMPL_HPP_
#define AIKIDO_EXAMPLE_DETAIL_EXAMPLECLASS_IMPL_HPP_

#include "aikido/example/ExampleClass.hpp"

namespace aikido {
namespace example {

//==============================================================================
template <typename T>
T TemplatedExampleClass<T>::exampleMethod()
{
  return T();
}

} // namespace example
} // namespace aikido

#endif // AIKIDO_EXAMPLE_DETAIL_EXAMPLECLASS_IMPL_HPP_
```

### Source Style

C++ sources should be contained in a subdirectory of `src/` that matches their
namespace, with the extension `.cpp`.

* Use **two-space** indentation
* Use **camelCase** function names
* Use **PascalCase** class names
* No "cuddled" braces!

```c++
// Includes should be at the top of the file.
// The first include in a class source file should be the matching `.hpp` header file.
#include "aikido/example/ExampleClass.hpp"

#include <stl_headers>
#include <library/headers.hpp>
#include "aikido/example/OtherHeaders.hpp"

using boost::format;
using boost::str;
using aikido::statespace::SE3StateSpace;

// Namespace nesting is preferred to "using namespace" directives.
// Namespaces scopes should be one line each with "cuddled" braces.
namespace aikido {
namespace example {

// Each function is separated by an 80 column line of "=" characters.
//==============================================================================
int ExampleClass::exampleInterfaceFunction() const
{
  if (mExampleMember)
    return 3;

  return -1;
}

//==============================================================================
int ExampleClass::exampleMethod(int a, int b, int* out) const
{
  int result = a + b;
  if (out)
    *out = result;
  return result;
}

} // namespace example
} // namespace aikido
```

### Smart Pointers

> These guidelines are based on [this article][sutter-smart-pointers]. Consider
> looking at the full article for the details.

[sutter-smart-pointers]: https://herbsutter.com/2013/06/05/gotw-91-solution-smart-pointer-parameters/

* General Rules
  * Use a by-value `std::shared_ptr` as a parameter if the function surely takes
    the shared ownership.
  * Use a `const std::shared_ptr&` as a parameter only if you're not sure
    whether or not you'll take a copy and share ownership.
  * Use a non-const `std::shared_ptr&` parameter only to modify the
    `std::shared_ptr`.
  * Use `std::unique_ptr` anytime you want to use a `std::shared_ptr` but don't
    need to share ownership.
  * Otherwise use `Object*` instead, or `Object&` if not nullable.

* Exception: 
  * Always pass AIKIDO `State`s by raw pointer. This is due to some of the
    tricks we play with placement-`new` to reduce `State` memory overhead,
    deferencing a `State *` could theoretically invoke undefined behavior even
    if you store it in a reference.

### Autoformatting using ClangFormat

You can automatically format all AIKIDO code
using [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) through
CMake. Make sure `clang-format 6.0` is installed.

#### Using CMake

```bash
$ cd to/aikido/root/
$ mkdir build
$ cd build
$ make check-format # to check the code without formatting
$ make format       # to format the code
```

#### Using catkin

```bash
$ cd to/your/catkin/workspace/
$ catkin build aikido --no-deps --make-args format       # to format the code
$ catkin build aikido --no-deps --make-args check-format # to check the code without formatting
```

## Python Style

Python style is strictly based on [PEP8](https://www.python.org/dev/peps/pep-0008/) conventions.

```python
import numpy
import scipy

from example.foo import ModeratelyLongClassName
from example.baz import (ReallyQuiteLongClassName1, ReallyQuiteLongClassName2,
                         ReallyQuiteLongClassName3)

SOME_CONSTANT = 3
"""
Field descriptions follow the declaration.  This isn't technically PEP8, but it
is recognized by many documentation generators.
"""


class MyExampleClass(object):
    """
    This is a brief summary of the class.  It is required.
    
    This is an extended description of the class.  It may contain lots of useful
    information about the class as well as how to use it.
    """
    def __init__(self, foo, bar, baz="Default Argument"):
        """
        This is a brief description of the constructor.  It is required.
        
        This is an extended description of the constructor. It can contain
        additional information about how to use the constructor, or details
        about what certain arguments mean.
        
        :param foo: a brief required description of the parameter foo
        :type  foo: the type required for foo, e.g. 'str' or 'int'
        :param bar: a brief required description of the parameter bar, but if
                    it needs to be really long, it should be wrapped like so
        :type  bar: the type of bar, note the extra space to get nice alignment
             <it is OK to leave a space between param comments if it looks cleaner>
        :param baz: description of baz (defaults to "Default Argument")
        :type  baz: the type of the parameter baz
        """
        pass
```

## CMake Style

* Use **two-space** indentation
* Use **lowercase** function names
* Use **all-caps** variables except when referring to target names
* Use `target_VARIABLE` when naming target-specific variables
* **ALWAYS** quote singleton variables (e.g. `"${MY_VARIABLE}"` but not `${MY_LIST_VARIABLE}`) 

```cmake
cmake_minimum_required(VERSION 2.8.11)  # Always declare a minimum version in the top-level CMakeLists.txt.

project(aikido)  # Only declare a project name in the top-level CMakeLists.txt.

# Put in comments liberally!  CMake is complicated!
if(SOME_VARIABLE)
  message(STATUS "Variable was set to '${SOME_VARIABLE}'.")
endif()

# Prefer using LIST functions to SET functions when working with list variables
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")   # ALWAYS quote around singleton variables
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

set(MY_INCLUDE_DIR include)  # Use all-caps for variables.

find_package(SomeLibrary REQUIRED)  # Use REQUIRED keyword when appropriate.

# For now, `include_directories` is necessary, but later we will switch to `target_include_directories`.
include_directories(
  "${MY_INCUDE_DIR}"  # This should be quoted.
)

# Complex commands should be split into one line for each semantic group (with two-space indentation).
# It is OK to put a target or output on the first line.
include_directories(SYSTEM 
  ${SomeLibrary_INCLUDE_DIRS}  # This should NOT be quoted, because it is a list.
)

add_library("${PROJECT_NAME}" SHARED  # This target name is generated from a variable, so it should be quoted.
  src/MySourceCode.cpp
)

# Always prefer `target_link_directories` to `link_directories` or other global functions.
target_link_libraries("${PROJECT_NAME}"
  ${SomeLibrary_LIBRARIES}
)

# Tests will typically be added to the end of the time from a `tests` subdirectory like the following.
enable_testing()
add_subdirectory(tests)
```
