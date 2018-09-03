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

  /// Constructs BSpline from control points and knots.
  ///
  /// The degree is automatically to be (num_knots - num_control_points - 1).
  ///
  /// \param[in] stateSpace State space this trajectory is defined in. Throw an
  /// exception if null.
  /// \param[in] knots Knots. The number of knots should be the same with the
  /// control points. The values of knots should be monotonically increased.
  /// Otherwise, it's undefined behavior.
  /// \param[in] controlPoints Control points. The number of control points
  /// should be the same with the control points.
  /// \throw If \c stateSpace is null.
  /// \throw If the size of \c knots is less than three.
  /// \throw If the size of \c controlPoints is less than one.
  /// \throw If the degree (size of \c knots - size of \c controlPoints - 1) is
  /// less than zero.
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
  /// \param[in] stateSpace State space this trajectory is defined in.
  /// \param[in] degree Degree of the b-spline.
  /// \param[in] controlPoints Control points. The number of control points
  /// should greater than degree. Otherwise, throw an exception.
  /// \param[in] startTime Start value of the time parameter. This will be the
  /// first knot value.
  /// \param[in] endTime End value of the time parameter. This will be the last
  /// knot value.
  /// \throw If \c stateSpace is null.
  /// \throw If the number of control points is equal to or less than the
  /// degree.
  BSpline(
      statespace::ConstStateSpacePtr stateSpace,
      std::size_t degree,
      const ControlPointVectorType& controlPoints,
      double startTime = 0.0,
      double endTime = 1.0);

  /// Constructs BSpline from the degree and the number of control points, which
  /// are all zeros.
  ///
  /// \param[in] stateSpace State space this trajectory is defined in. Throw an
  /// exception if null.
  /// \param[in] startTime Start time of the trajectory. This will be the first
  /// knot value.
  /// \param[in] endTime End value of the time parameter. This will be the last
  /// knot value.
  /// \throw If \c stateSpace is null.
  /// \throw If the number of control points is equal to or less than the
  /// degree.
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

  /// Sets start point of one sub space of the state space. Identical to setting
  /// the first control point.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  void setStartPoint(std::size_t stateSpaceIndex, double value);

  /// Sets start point. Identical to setting the first control point.
  /// setStartPoint(stateSpace->logMap(state)).
  ///
  /// \param[in] point Start point on the tangent space.
  void setStartPoint(const Eigen::VectorXd& point);

  /// Sets start point. Identical to setting the first control point.
  ///
  /// \param[in] state Start point.
  void setStartPoint(const statespace::StateSpace::State* state);

  /// Sets end point of one sub space of the state space. Identical to setting
  /// the last control point.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the end
  /// point.
  void setEndPoint(std::size_t stateSpaceIndex, double value);

  /// Sets end point. Identical to setting the last control point.
  /// setEndPoint(stateSpace->logMap(state)).
  ///
  /// \param[in] point End point on the tangent space.
  void setEndPoint(const Eigen::VectorXd& point);

  /// Sets end point. Identical to setting the last control point.
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
  /// \param[in] withStartControlPoint Whether to set the start control point.
  /// \param[in] withEndControlPoint Whether to set the end control point.
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
  /// \param[in] withStartControlPoint Whether to set the start control point.
  /// \param[in] withEndControlPoint Whether to set the end control point.
  void setControlPoints(
      std::size_t stateSpaceIndex,
      double value,
      bool withStartControlPoint = true,
      bool withEndControlPoint = true);

  /// Returns control points of one subspace of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  /// \return Control points.
  const ControlPointVectorType& getControlPoints(
      std::size_t stateSpaceIndex) const;

  /// Returns control points of one subspace of the state space.
  ///
  /// \param[in] stateSpaceIndex The index to the state space to set the start
  /// point.
  /// \param[in] withStartControlPoint Whether to get the start control point.
  /// \param[in] withEndControlPoint Whether to get the end control point.
  /// \return Control points.
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
  // TODO(JS): Consider adding this function to the base class as a pure virtual
  // function.

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
  /// \throw If the number of control points is equal to or less than the
  /// degree.
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
