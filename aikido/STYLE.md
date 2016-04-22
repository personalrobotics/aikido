# AIKIDO Style Guide #

The code in this library generally follows the same coding convention as the [DART/KIDO](https://github.com/dartsim/dart).

## C++ Header Style

C++ Headers should be contained in paths that match their namespace, with the extension `.hpp`.

```c++
#ifndef AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_  // Header guards must include the library, namespace, and source file names.
#define AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_

// Place all dependency includes at the top of the file.
// Use relative paths for includes within the same directory structure, and place these at the top.
#include "ExampleInterface.hpp"  
#include "../other_example/ExampleOtherInterface.hpp"
#include <library/library.hpp>

// Namespaces scopes should be one line each with "cuddled" braces.
namespace aikido {
namespace example {

/// A required doxygen comment descripion for this class.  This can be extended to
/// include various useful detail about the class, and can use the standard doxygen
/// tag set to refer to other classes or documentation.  It should use the '\\\'
/// style of block comment.
class ExampleClass
    : public ExampleInterface
    , public ExampleOtherInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Many classes that require Eigen will also need this macro

  /// Required brief description of constructor.  This will often be as simple as:
  /// "Creates an instance of ExampleClass."
  ///
  /// \param[in] _foo this is an example parameter description
  /// \param[in] _bar this is a longer example parameter description that needs
  ///            to wrap across multiple lines.
  ExampleClass(std::unique_ptr<util::RNG> _foo,
               const Eigen::Isometry3d& _bar = Eigen::Isometry3d::Identity());

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
  int exampleInterfaceFunction() const override;  // <-- Always explictly `override` interface functions

  /// This is a docstring for a method, it is required.
  /// - If a method has output parameters, they should be the last arguments.
  /// - Argument names are prefixes with a leading "_".
  ///
  /// \param[in] _A a description of _A
  /// \param[in] _B a description of _B
  /// \param[out] _out a description of _out
  int exampleMethod(int _A, int _B, int *_out) const;

private:
  std::unique_ptr<util::RNG> mExampleMember; // Member variables are prefixed with "m"
};

// Use "using" directive to declare a shared pointer helper type.  It should not be `const`.
using ExamplePtr = std::shared_ptr<Example>; 

} // namespace example
} // namespace aikido

// In certain cases, such as heavily templated code, implementations must be included
// in headers. In this case, a "detail" header should be created in the "./detail"
// subdirectory with the same name as the main header file, but an "_impl" suffix.
// Private declarations in this header can use a "detail" sub-namespace.
#include "./detail/ExampleClass_impl.hpp"

#endif  // AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_
```

## C++ Source Style

```c++
// Includes should be at the top of the file.
// The first include in a class source file should be the matching `.hpp` header file.
#include <aikido/example/ExampleClass.hpp>
#include <boost/format.hpp>
#include <stdexcept>

using boost::format;
using boost::str;
using aikido::statespace::SE3StateSpace;

// Namespace nesting is preferred to "using namespace" directives.
// Namespaces scopes should be one line each with "cuddled" braces.
namespace aikido {
namespace example {

// Each function is separated by an 80 column line of "=" characters.
//=============================================================================
int ExampleClass::exampleInterfaceFunction() const
{
  if (mExampleMember)
    return 3;

  return -1;
}

//=============================================================================
int ExampleClass::exampleMethod(int _A, int _B, int *_out) const
{
  int result = A + B:
  
}

} // namespace example
} // namespace aikido
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
