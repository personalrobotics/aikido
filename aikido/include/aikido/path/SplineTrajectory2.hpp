#ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#define AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#include "Trajectory.hpp"

namespace aikido {
namespace path {

class SplineTrajectory2  : public Trajectory
{
public:
  struct PolynomialSegment
  {
    Eigen::MatrixXd mCoefficients;
    double mDuration;
  };

  SplineTrajectory2(
    statespace::StateSpacePtr _stateSpace,
    const statespace::StateSpace::State* _startState,
    double _startTime,
    std::vector<PolynomialSegment> _segments);

  virtual ~SplineTrajectory2();

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  int getNumDerivatives() const override;

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
  static Eigen::VectorXd evaluatePolynomial(
    const Eigen::MatrixXd& _coefficients, double _t, int _derivative);

  statespace::StateSpacePtr mStateSpace;
  statespace::StateSpace::State* mStartState;
  double mStartTime;
  std::vector<PolynomialSegment> mSegments;
};

} // namespace path
} // namespace aikido

#endif // ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
