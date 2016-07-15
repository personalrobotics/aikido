#include <aikido/planner/mintos/MinTOS.hpp>
#include <aikido/util/StepSequence.hpp>
#include <mintos/MinTOS.h>

namespace aikido {
namespace planner {
namespace mintos {
namespace {

void vectorToEigen(const Math::Vector& _input, Eigen::VectorXd& _output)
{
  _output.resize(_input.size());

  for (int i = 0; i < _input.size(); ++i)
    _output[i] = _input[i];
}

void eigenToVector(const Eigen::VectorXd& _input, Math::Vector& _output)
{
  _output.resize(_input.size());

  for (int i = 0; i < _input.size(); ++i)
    _output[i] = _input[i];
}

class DifferentiableWrapper : public Math::VectorFieldFunction
{
public:
  explicit DifferentiableWrapper(const constraint::Differentiable* _constraint)
    : mConstraint{_constraint}
    , mStateSpace{mConstraint->getStateSpace()}
    , mState{mStateSpace->allocateState()}
  {
  }

  virtual ~DifferentiableWrapper()
  {
    mStateSpace->freeState(mState);
  }

  int NumDimensions() const override
  {
    return mConstraint->getConstraintDimension();
  }

  void PreEval(const Math::Vector& x) override
  {
    // Convert the input vector to a State.
    vectorToEigen(x, mTangentVector);
    mStateSpace->expMap(mTangentVector, mState);

    // Evaluate the constraint and store the result for future calls.
    mConstraint->getValueAndJacobian(mState, mValue, mJacobian);
  }

  void Eval(const Math::Vector& x, Math::Vector& v) override
  {
    eigenToVector(mValue, v);
  }

  Math::Real Jacobian_ij(const Math::Vector& x, int i, int j) override
  {
    return mJacobian(i, j);
  }

  void Hessian_i(const Math::Vector& x, int i, Math::Matrix& Hi) override
  {
    throw std::runtime_error("Hessian_i is not implemented on DifferentiableWrapper.");
  }

  Math::Real Hessian_ijk(const Math::Vector& x, int i, int j, int k) override
  {
    throw std::runtime_error("Hessian_ijk is not implemented on DifferentiableWrapper.");
  }

private:
  const constraint::Differentiable* mConstraint;
  std::shared_ptr<statespace::StateSpace> mStateSpace;
  statespace::StateSpace::State* mState;
  Eigen::VectorXd mTangentVector;
  Eigen::VectorXd mValue;
  Eigen::MatrixXd mJacobian;
};

class StateSpaceWrapper : public Mintos::GeodesicManifold
{
 public:
  virtual ~StateSpaceWrapper() = default;

  Math::Real Distance(const Mintos::Config& x, const Mintos::Config& y) override
  {
    throw std::runtime_error("Distance is not implemented.");
  }

  void Interpolate(
    const Mintos::Config& x, const Mintos::Config& y,
    Math::Real u, Mintos::Config& out) override
  {
    throw std::runtime_error("Interpolate is not implemented.");
  }

  void InterpolateDeriv(
    const Mintos::Config& a, const Mintos::Config& b,
    Math::Real u, Math::Vector& dx) override
  {
    throw std::runtime_error("InterpolateDeriv is not implemented.");
  }

  void InterpolateDerivA(
    const Mintos::Config& a, const Mintos::Config& b,
    Math::Real u, const Math::Vector& da, Math::Vector& dx) override
  {
    throw std::runtime_error("InterpolateDerivA is not implemented.");
  }

  void InterpolateDerivB(
    const Mintos::Config& a, const Mintos::Config& b,
    Math::Real u, const Math::Vector& db, Math::Vector& dx) override
  {
    throw std::runtime_error("InterpolateDerivB is not implemented.");
  }

  void InterpolateDeriv2(
    const Mintos::Config& a, const Mintos::Config& b,
    Math::Real u, Math::Vector& ddx) override
  {
    throw std::runtime_error("InterpolateDeriv2 is not implemented.");
  }

  void Integrate(
    const Mintos::Config& a, const Math::Vector& da,
    Mintos::Config& b) override
  {
    throw std::runtime_error("Integrate is not implemented.");
  }
};

} // namespace

std::unique_ptr<trajectory::Spline> interpolateAndTimeOptimizeTrajectory(
  const trajectory::Interpolated& _inputTrajectory,
  const constraint::Differentiable& _constraint,
  const Eigen::VectorXd& _minVelocity,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _minAcceleration,
  const Eigen::VectorXd& _maxAcceleration,
  double _constraintTolerance,
  double _interpolationTimestep)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  Eigen::VectorXd tangentVector;

  // Convert the trajectory into a sequence of milestones.
  std::vector<Mintos::Config> milestones;
  milestones.reserve(_inputTrajectory.getNumWaypoints());

  for (size_t iwaypoint = 0;
       iwaypoint < _inputTrajectory.getNumWaypoints();
       ++iwaypoint)
  {
    // This assumes that the StateSpace consists of Rn and SO(2) components.
    // TODO: This should work on all StateSpaces if we implement a custom
    // GeodesicManifold that defers all operations correctly.
    const auto state = _inputTrajectory.getWaypoint(iwaypoint);
    stateSpace->logMap(state, tangentVector);

    // Convert the Eigen::Vector into a Config (a Math::VectorTemplate).
    milestones.emplace_back(tangentVector.size(), tangentVector.data());
  }

  // Convert the Aikido constraint to a MinTOS VectorFieldFunction.
  DifferentiableWrapper constraintWrapper{&_constraint};

  // Convert velocity and acceleration bounds to MinTOS types.
  const Mintos::Vector minVelocity(_minVelocity.size(), _minVelocity.data());
  const Mintos::Vector maxVelocity(_maxVelocity.size(), _maxVelocity.data());
  const Mintos::Vector minAcceleration(
    _minAcceleration.size(), _minAcceleration.data());
  const Mintos::Vector maxAcceleration(
    _maxAcceleration.size(), _maxAcceleration.data());

  // Run MinTOS to produce a cubic Bezier curve.
  Mintos::TimeScaledBezierCurve outputCurve;
  auto const success = Mintos::InterpolateAndTimeOptimize(
    milestones,
    nullptr, // TODO: Replace this with a GeodesicManifold from the StateSpace.
    &constraintWrapper, _constraintTolerance,
    minVelocity, maxVelocity,
    minAcceleration, maxAcceleration,
    outputCurve);
  if (!success)
    throw std::runtime_error("Failed to optimize input path.");

  // Convert the output of MinTOS to an Aikido trajectory.
  util::StepSequence timeSequence{
    _interpolationTimestep, true, 0., outputCurve.EndTime()};
  Eigen::VectorXd position, velocity, acceleration;
  Math::Vector outputVector;


  for (const auto t : timeSequence)
  {
    // Evaluate the MinTOS trajectory.
    outputCurve.Eval(t, outputVector);
    vectorToEigen(outputVector, position);

    outputCurve.Deriv(t, outputVector);
    vectorToEigen(outputVector, velocity);

    outputCurve.Accel(t, outputVector);
    vectorToEigen(outputVector, acceleration);

    // TODO: Fit a polynomial to this segment.
  }

  return nullptr;
}

} // namespace mintos
} // namespace planner
} // namespace aikido
