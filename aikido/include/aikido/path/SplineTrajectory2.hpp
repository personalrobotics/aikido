#ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#define AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#include "Trajectory.hpp"

namespace aikido {
namespace path {

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
class SplineTrajectory2 : public Trajectory
{
public:
  /// Constructs an empty trajectory.
  ///
  /// \param _stateSpace state space this trajectory is defined in
  /// \param _startTime start time of the trajectory
  SplineTrajectory2(
    statespace::StateSpacePtr _stateSpace, double _startTime = 0.);

  virtual ~SplineTrajectory2();
  
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
  /// \param _coefficients polynomial coefficients
  /// \param _duration duration of this segment, must be positive
  /// \param _startSTate start state of the segment
  void addSegment(const Eigen::MatrixXd& _coefficients, double _duration,
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
  /// \param _coefficients polynomial coefficients
  /// \param _duration duration of this segment, must be positive
  void addSegment(const Eigen::MatrixXd& _coefficients, double _duration);

  /// Gets the number of segments in this spline.
  ///
  /// \return number of segments in this spline
  size_t getNumSegments() const;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  size_t getNumDerivatives() const override;

  // Documentation inherited.
  double getStartTime() const override;

  // Documentation inherited.
  double getEndTime() const override;

  // Documentation inherited.
  double getDuration() const override;

  // Documentation inherited.
  void evaluate(
    double _t, statespace::StateSpace::State *_state) const override;

  // Documentation inherited.
  Eigen::VectorXd evaluate(double _t, int _derivative) const override;

private:
  struct PolynomialSegment
  {
    statespace::StateSpace::State* mStartState; 
    Eigen::MatrixXd mCoefficients;
    double mDuration;
  };

  static Eigen::VectorXd evaluatePolynomial(
    const Eigen::MatrixXd& _coefficients, double _t, int _derivative);

  std::pair<size_t, double> getSegmentForTime(double _t) const;

  statespace::StateSpacePtr mStateSpace;
  double mStartTime;
  std::vector<PolynomialSegment> mSegments;
};

} // namespace path
} // namespace aikido

#endif // ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
