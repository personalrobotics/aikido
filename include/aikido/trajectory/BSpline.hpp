#ifndef AIKIDO_TRAJECTORY_BSPLIINE_HPP_
#define AIKIDO_TRAJECTORY_BSPLIINE_HPP_

#include "aikido/common/BSpline.hpp"
#include "aikido/common/pointers.hpp"
#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace trajectory {

AIKIDO_DECLARE_POINTERS(BSpline)

/// B-spline trajectory define in a \c StateSpace.
class BSpline : public Trajectory
{
public:
  using SplineType = common::BSpline<double, 1, Eigen::Dynamic>;
  using KnotVectorType = typename SplineType::KnotVectorType;
  using ControlPointVectorType = typename SplineType::ControlPointVectorType;

  /// Constructs BSpline from control points and knots. The degree is
  /// automatically determined by the
  ///
  /// \param[in] stateSpace State space this trajectory is defined in. Throw an
  /// exception if null.
  /// \param[in] knots Knots. The number of knots should be the same with the
  /// control points.
  /// \param[in] controlPoints Control points. The number of control points
  /// should be the same with the control points.
  BSpline(
      statespace::ConstStateSpacePtr stateSpace,
      const KnotVectorType& knots,
      const ControlPointVectorType& controlPoints);

  /// Constructs BSpline from control points where the knots are uniformly
  /// distributed.
  ///
  /// The start and end point are the same with the first and last control
  /// points, respectively.
  ///
  /// \param[in] stateSpace State space this trajectory is defined in. Throw an
  /// exception if null.
  /// \param[in] degree Degree of the b-spline.
  /// \param[in] controlPoints Control points. The number of control points
  /// should greater than degree. Otherwise, throw an exception.
  /// \param[in] startTime Start value of the time parameter. This will be the
  /// first knot value.
  /// \param[in] endTime End value of the time parameter. This will be the last
  /// knot value.
  BSpline(
      statespace::ConstStateSpacePtr stateSpace,
      std::size_t degree,
      const ControlPointVectorType& controlPoints,
      double startTime = 0.0,
      double endTime = 1.0);

  /// Constructs an empty trajectory.
  ///
  /// \param stateSpace State space this trajectory is defined in. Throw an
  /// exception if null.
  /// \param startTime Start time of the trajectory
  BSpline(
      statespace::ConstStateSpacePtr stateSpace,
      std::size_t degree,
      std::size_t numControlPoints,
      double startTime = 0.0,
      double endTime = 1.0);

  /// Copy constructor
  ///
  /// \param[in] other The other BSpline being copied.
  BSpline(const BSpline& other);

  /// Move constructor
  ///
  /// \param[in] other The other BSpline being moved.
  BSpline(BSpline&& other);

  /// Destructor
  ~BSpline() override;

  /// Copy assignment operator
  ///
  /// \param[in] other The other BSpline being copied.
  BSpline& operator=(const BSpline& other);

  /// Move assignment operator
  ///
  /// \param[in] other The other BSpline being moved.
  BSpline& operator=(BSpline&& other);

  /// Returns a clone of this BSpline.
  std::unique_ptr<trajectory::Trajectory> clone() const;

  /// Returns degree of the spline. Degree is order - 1.
  std::size_t getDegree() const;

  /// Returns order of the spline. Order is degree + 1.
  std::size_t getOrder() const;

  /// Returns the number of knots
  std::size_t getNumKnots() const;

  /// Returns the number of control points
  std::size_t getNumControlPoints() const;

  /// Sets start point of one sub space of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  void setStartPoint(std::size_t stateSpaceIndex, double value);

  /// Sets start point. Identical to call \c
  /// setStartPoint(stateSpace->logMap(state)).
  ///
  /// \param[in] point Start point on the tangent space.
  void setStartPoint(const Eigen::VectorXd& point);

  /// Sets start point.
  ///
  /// \param[in] state Start point.
  void setStartPoint(const statespace::StateSpace::State* state);

  /// Sets end point of one sub space of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the end
  /// point.
  void setEndPoint(std::size_t stateSpaceIndex, double value);

  /// Sets end point. Identical to call \c
  /// setEndPoint(stateSpace->logMap(state)).
  ///
  /// \param[in] point End point on the tangent space.
  void setEndPoint(const Eigen::VectorXd& point);

  /// Sets end point.
  ///
  /// \param[in] state End point.
  void setEndPoint(const statespace::StateSpace::State* state);

  /// Sets all the control points of one subspace of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  /// \param[in] controlPoints Control points. The size of the control points
  /// can
  /// be different depending on \c withStartControlPoints and \c
  /// withStartControlPoints. Decrease the number by one per one option is true.
  /// \param[in] withStartControlPoints Whether to take into account the start
  /// control point as variable.
  /// \param[in] withStartControlPoints Whether to take into account the end
  /// control point as variable.
  void setControlPoints(
      std::size_t stateSpaceIndex,
      const ControlPointVectorType& controlPoints,
      bool withStartControlPoint = true,
      bool withEndControlPoint = true);

  /// Sets one control point of one subspace of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  /// \param[in] value Control point value.
  /// \param[in] withStartControlPoints Whether to take into account the start
  /// control point as variable.
  /// \param[in] withStartControlPoints Whether to take into account the end
  /// control point as variable.
  void setControlPoints(
      std::size_t stateSpaceIndex,
      double value,
      bool withStartControlPoint = true,
      bool withEndControlPoint = true);

  const ControlPointVectorType& getControlPoints(
      std::size_t stateSpaceIndex) const;

  ControlPointVectorType getControlPoints(
      std::size_t stateSpaceIndex,
      bool withStartControlPoint,
      bool withEndControlPoint) const;

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::size_t getNumDerivatives() const override;

  // Documentation inherited.
  double getStartTime() const override;

  // Documentation inherited.
  double getEndTime() const override;

  // Documentation inherited.
  double getDuration() const override;

  // Documentation inherited.
  void evaluate(double t, statespace::StateSpace::State* state) const override;

  // Documentation inherited.
  void evaluateDerivative(
      double t, int derivative, Eigen::VectorXd& tangentVector) const override;

  /// Computes arc length.
  ///
  /// \param[in] distanceMetric Distance metric to measure the arc length.
  /// \param[in] resolution Size of the line segments.
  virtual double computeArcLength(
      const distance::DistanceMetric& distanceMetric,
      double resolution = 0.1) const;

protected:
  /// Computes number of knots given degree and number of control points.
  static std::size_t computeNumKnots(
      std::size_t degree, std::size_t numControlPoints);

  /// Computes uniform knots.
  ///
  /// \param[in] degree Degree of the b-spline
  /// \param[in] numControlPoints Number of the control points.
  /// \param[in] startTime Start time of the knots.
  /// \param[in] startTime End time of the knots.
  static KnotVectorType computeUniformKnots(
      std::size_t degree,
      std::size_t numControlPoints,
      double startTime = 0.0,
      double endTime = 1.0);

  /// State space.
  statespace::ConstStateSpacePtr mStateSpace;

  /// One-demensional splines.
  std::vector<SplineType> mSplines;

  /// Start time of b-spline.
  double mStartTime;
  // This variable can be redundant because the knots in Eigen::Spline encodes
  // the start time. This variable is for the case of zero dimensional state
  // space since m1DSpline will be empty in that case.

  /// End time of b-spline
  double mEndTime;
  // This variable can be redundant because the knots in Eigen::Spline encodes
  // the end time. This variable is for the case of zero dimensional state space
  // since m1DSpline will be empty in that case.
};

} // namespace trajectory
} // namespace aikido

#endif // AIKIDO_TRAJECTORY_BSPLIINE_HPP_
