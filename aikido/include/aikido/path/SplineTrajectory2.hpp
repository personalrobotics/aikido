#ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#define AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
#include "Trajectory.hpp"

namespace aikido {
namespace path {

class SplineTrajectory2  : public Trajectory
{
public:
  SplineTrajectory2(
    statespace::StateSpacePtr _stateSpace,
    const statespace::StateSpace::State* _startState,
    double _startTime);

  virtual ~SplineTrajectory2();
  
  void addSegment(
    const Eigen::MatrixXd& _coefficients, double _duration);

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
  struct PolynomialSegment
  {
    Eigen::MatrixXd mCoefficients;
    double mDuration;
  };

  static Eigen::VectorXd evaluatePolynomial(
    const Eigen::MatrixXd& _coefficients, double _t, int _derivative);

  std::pair<size_t, double> getSegmentForTime(double _t) const;

  void getSegmentStartState(
    size_t _index, statespace::StateSpace::State* _out) const;

  statespace::StateSpacePtr mStateSpace;
  statespace::StateSpace::State* mStartState;
  double mStartTime;
  std::vector<PolynomialSegment> mSegments;
};

} // namespace path
} // namespace aikido

#endif // ifndef AIKIDO_PATH_SPLINETRAJECTORY2_HPP_
