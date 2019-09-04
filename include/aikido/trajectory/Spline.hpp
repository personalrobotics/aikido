#ifndef AIKIDO_TRAJECTORY_SPLINETRAJECTORY2_HPP_
#define AIKIDO_TRAJECTORY_SPLINETRAJECTORY2_HPP_

#include "aikido/common/pointers.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace trajectory {

AIKIDO_DECLARE_POINTERS(Spline)

/// Polynomial spline trajectory defined in a \c StateSpace. The trajectory is
/// represented as a sequence of polynomial segments between knot points. Each
/// segment is defined by a start state, a duration, and polynomial
/// coefficients for each dimension of the \c StateSpace. The polynomial for a
/// segment is parameterized in the tangent space of its start state.
///
/// It is possible to avoid singularities in \c expMap and \c logMap by
/// starting a new segment whenever passing near a singularity. Such a
/// parameterization always exists because Lie groups are locally Euclidean.
///
/// This trajectory does \b not guarantee any continuity (not even C0). It is
/// the responsibility of the user to pass in continuous spline coefficients
/// if continuity is desired.
class Spline : public Trajectory
{
public:
  /// Constructs an empty trajectory.
  ///
  /// \param _stateSpace State space this trajectory is defined in
  /// \param _startTime Start time of the trajectory
  Spline(statespace::ConstStateSpacePtr _stateSpace, double _startTime = 0.);

  virtual ~Spline();

  /// Add a segment to the end of this trajectory that starts at
  /// \c _startState, lasts for \c _duration, and is defined by a polynomial in
  /// the tangent space of \c _startState.
  ///
  /// The \c _coefficients matrix is a (num dimensions) x (num coefficients)
  /// matrix where the i-th row defines polynomial coefficients for the i-th
  /// dimension of the state space and the j-th column defines the coefficient
  /// for the j-th power of time parameter. For example, element (3, 2) defines
  /// the polynomial coefficient on t^2 for the third state space dimension.
  ///
  /// Polynomial coefficients are defined in the tangent space of the segment's
  /// start state, given by \c _startState. The polynomial encoded by \c
  /// _coefficients will be evaluated at times [ 0, \c _duration ]. Segments
  /// may have heterogeneous polynomial orders, and thus, coefficient matrices
  /// with differing numbers of columns.
  ///
  /// Note that the value of \c _startState is not checked for C0 continuity,
  /// nor are the derivatives evaluated at the start of the segment checked for
  /// higher-order continuity. It is the responsibility of the user to pass in
  /// continuous spline coefficients if continuity is desired.
  ///
  /// \param _coefficients Polynomial coefficients
  /// \param _duration Duration of this segment, must be positive
  /// \param _startState Start state of the segment
  void addSegment(
      const Eigen::MatrixXd& _coefficients,
      double _duration,
      const statespace::StateSpace::State* _startState);

  /// Adds a segment to the end of the trajectory while preserving C0
  /// continuity. This is a helper function that calls the three-argument
  /// overload of this function with \c _startState set to the end of the
  /// trajectory. As a result, the trajectory must be non-empty.
  ///
  /// Note that repeated calls to this function may accumulate numerical
  /// imprecision from repeated polynomial evaluation. We suggest calling
  /// the overload of the function that accepts an explicit start state
  /// if it is possible to do so.
  ///
  /// \param _coefficients Polynomial coefficients
  /// \param _duration Duration of this segment, must be positive
  void addSegment(const Eigen::MatrixXd& _coefficients, double _duration);

  /// Gets the number of segments in this spline.
  ///
  /// \return Number of segments in this spline
  std::size_t getNumSegments() const;

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
  void evaluate(
      double _t, statespace::StateSpace::State* _state) const override;

  // Documentation inherited.
  void evaluateDerivative(
      double _t,
      int _derivative,
      Eigen::VectorXd& _tangentVector) const override;

  /// Gets the number of waypoints.
  /// \return The number of waypoints
  std::size_t getNumWaypoints() const;

  /// Gets a waypoint.
  ///
  /// \param _index Waypoint index
  /// \param[out] state State of the waypoint at index \c _index
  void getWaypoint(
      std::size_t _index, statespace::StateSpace::State* state) const;

  /// Gets the time of a waypoint.
  /// \param _index Waypoint index
  /// \return Time of the waypoint at index \c _index
  double getWaypointTime(std::size_t _index) const;

  /// Gets the derivative of a waypoint.
  /// \param _index Waypoint index
  /// \param _derivative Order of derivative
  /// \param[out] _tangentVector Output tangent vector in the local frame at
  /// index \c _index
  void getWaypointDerivative(
      std::size_t _index,
      int _derivative,
      Eigen::VectorXd& _tangentVector) const;

  /// Gets the duration of a segment.
  /// \param _index Segment index
  /// \return Duration of the segment at index \c _index
  double getSegmentDuration(std::size_t _index) const;

  /// Gets the coefficients of a segment.
  /// \param _index Segment index
  /// \return Coefficients of the segment at index \c _index
  const Eigen::MatrixXd& getSegmentCoefficients(std::size_t _index) const;

  /// Gets the start state of a segment.
  /// \param _index Segment index
  /// \return Start state of the segment at index \c _index
  const aikido::statespace::StateSpace::State* getSegmentStartState(
      std::size_t _index) const;

private:
  struct PolynomialSegment
  {
    statespace::StateSpace::State* mStartState;
    Eigen::MatrixXd mCoefficients;
    double mDuration;
  };

  static Eigen::VectorXd evaluatePolynomial(
      const Eigen::MatrixXd& _coefficients, double _t, int _derivative);

  std::pair<std::size_t, double> getSegmentForTime(double _t) const;

  statespace::ConstStateSpacePtr mStateSpace;
  double mStartTime;
  std::vector<PolynomialSegment> mSegments;
};

} // namespace trajectory
} // namespace aikido

#endif // ifndef AIKIDO_TRAJECTORY_SPLINETRAJECTORY2_HPP_
