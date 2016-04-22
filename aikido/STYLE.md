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
  /// \param _foo this is an example parameter description
  /// \param _bar this is a longer example parameter description that neesd
  ///        to wrap across multiple lines.
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
  int exampleInterfaceFunction() const override;

  /// This is a docstring for a method, it is required.
  void exampleMethod() const;

private:
  std::unique_ptr<util::RNG> mExampleMember; // Member variables are prefixed with "m"
};

// Use "using" directives to declare pointer helper types
using ExamplePtr = std::shared_ptr<Example>; 

}  // namespace example
}  // namespace aikido

#endif  // AIKIDO_EXAMPLE_EXAMPLECLASS_HPP_
```

## C++ Source Style

```c++
#include <aikido/constraint/TSR.hpp>
#include <dart/common/Console.h>
#include <dart/common/StlHelpers.h>
#include <dart/math/Geometry.h>
#include <boost/format.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>
#include <random>

using boost::format;
using boost::str;
using aikido::statespace::SE3StateSpace;

namespace aikido {
namespace constraint {

class TSRSampleGenerator : public SampleGenerator
{
public:
  TSRSampleGenerator(const TSRSampleGenerator&) = delete;
  TSRSampleGenerator(TSRSampleGenerator&& other) = delete;
  TSRSampleGenerator& operator=(const TSRSampleGenerator& other) = delete;
  TSRSampleGenerator& operator=(TSRSampleGenerator&& other) = delete;
  virtual ~TSRSampleGenerator() = default; 

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Return a transform sampled from this TSR.
  ///
  /// This function uses the provided RNG to create a sample `Tw_s` from the
  /// `Bw` bounds matrix of this TSR, and returns the result:
  /// `T0_w * Tw_s * Tw_e`.
  ///
  /// \param[in] rng Random number generator from which to sample
  /// \return a transform within the bounds of this TSR.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  bool canSample() const override;

  [...]

private:
  // For internal use only.
  TSRSampleGenerator(std::unique_ptr<util::RNG> _rng,
                     std::shared_ptr<statespace::SE3StateSpace> _stateSpace,
                     const Eigen::Isometry3d& _T0_w,
                     const Eigen::Matrix<double, 6, 2>& _Bw,
                     const Eigen::Isometry3d& _Tw_e);
  
  std::unique_ptr<util::RNG> mRng;

  std::shared_ptr<statespace::SE3StateSpace> mStateSpace;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

  // True for point TSR.
  bool mPointTSR;

  // True if point TSR and has already been sampled.
  bool mPointTSRSampled;


  friend class TSR;
};

[...]

//=============================================================================
bool TSRSampleGenerator::canSample() const
{
  if (mPointTSR && mPointTSRSampled)
    return false;

  return true;
}

//=============================================================================
int TSRSampleGenerator::getNumSamples() const
{
  if (mPointTSR && !mPointTSRSampled)
    return 1;

  if (mPointTSR && mPointTSRSampled)
    return 0;

  return NO_LIMIT;
}

} // namespace constraint
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
        
        :param baz: description of baz (defaults to "Default Argument")
        :type  baz: the type of the parameter baz
        """
        pass
```
