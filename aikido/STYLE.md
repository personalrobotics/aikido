# AIKIDO Style Guide #

The code in this library generally follows the same coding convention as the [DART/KIDO](https://github.com/dartsim/dart).

## C++ Header Style

```c++
#ifndef AIKIDO_CONSTRAINT_TSR_H_
#define AIKIDO_CONSTRAINT_TSR_H_

#include "Sampleable.hpp"
#include "../statespace/SE3StateSpace.hpp"
#include <Eigen/Dense>
#include "Projectable.hpp"
#include "Differentiable.hpp"
#include "TestableConstraint.hpp"
#include <dart/math/MathTypes.h>

namespace aikido {
namespace constraint {

/// TSRs describe end-effector constraint sets as subsets of SE(3).
/// A TSR consists of three parts:
///     T0_w: transform from the origin to the TSR frame w
///     B_w: 6 × 2 matrix of bounds in the coordinates of w.
///     Tw_e: end-effector offset transform in the coordinates of w
/// See:
/// Berenson, Dmitry, Siddhartha S. Srinivasa, and James Kuffner.
/// "Task space regions: A framework for pose-constrained manipulation
/// planning." IJRR 2001:
/// http://repository.cmu.edu/cgi/viewcontent.cgi?article=2024&context=robotics
class TSR : public SampleableConstraint,
            public Differentiable,
            public TestableConstraint,
            public Projectable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor.
  /// \param _rng Random number generator used by SampleGenerators for this TSR.
  /// \param _T0_w transform from the origin to the TSR frame w
  /// \param _Bw 6 × 2 matrix of bounds in the coordinates of w.
  ///        Top three rows bound translation, and bottom three rows
  ///        bound rotation following Roll-Pitch-Yaw convention.
  ///        _Bw(i, 0) should be less than or equal to _Bw(i, 1).
  /// \param _Tw_e end-effector offset transform in the coordinates of w
  TSR(std::unique_ptr<util::RNG> _rng,
      const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw =
          Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());


  /// Constructor with default random seed generator.
  /// \param _T0_w transform from the origin to the TSR frame w
  /// \param _Bw 6 × 2 matrix of bounds in the coordinates of w.
  ///        Top three rows bound translation, and bottom three rows
  ///        bound rotation following Roll-Pitch-Yaw convention.
  ///        _Bw(i, 0) should be less than or equal to _Bw(i, 1).
  /// \param _Tw_e end-effector offset transform in the coordinates of w
  TSR(const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw =
          Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const TSR& other);
  TSR(TSR&& other);

  TSR& operator=(const TSR& other);
  TSR& operator=(TSR&& other);

  virtual ~TSR() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Returns the SE3StateSpace which this TSR operates in.
  std::shared_ptr<statespace::SE3StateSpace> getSE3StateSpace() const;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* _s) const override;

  /// Throws an invalid_argument exception if this TSR is invalid.
  /// For a TSR to be valid, mBw(i, 0) <= mBw(i, 1).
  void validate() const;

  /// Set the random number generator used by SampleGenerators for this TSR.
  void setRNG(std::unique_ptr<util::RNG> rng);

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
      const statespace::StateSpace::State* _s) const override;

  /// Returns 6 x 6 matrix.
  /// Jacobian of TSR with respect to the se(3) tangent vector of _s.
  /// The jacobian is w.r.t. the origin frame.
  /// se(3) tangent vector follows dart convention:
  ///   top 3 rows is the angle-axis representation of _s's rotation.
  ///   bottom 3 rows represent the translation.
  Eigen::MatrixXd getJacobian(
      const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
      const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool project(const statespace::StateSpace::State* _s,
      statespace::StateSpace::State* _out) const override;

  /// Transformation from origin frame into the TSR frame "w".
  /// "w" is usually centered at the origin of an object held by the hand
  ///  or at a location on an object that is useful for grasping.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "w" frame into end frame.
  /// This often represent an offset from "w" to the origin of the end-effector.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<util::RNG> mRng;
  std::shared_ptr<statespace::SE3StateSpace> mStateSpace;
};

using TSRPtr = std::shared_ptr<TSR>;

}  // namespace constraint
}  // namespace aikido

#endif  // AIKIDO_CONSTRAINT_TSR_H_
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


class MyExampleClass(object)
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
