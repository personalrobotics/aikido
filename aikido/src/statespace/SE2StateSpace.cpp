#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SE2StateSpaceSampleableConstraint.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpace::State::State()
  : mTransform(Isometry2d::Identity())
{
}

//=============================================================================
SE2StateSpace::State::State(const Isometry2d& _transform)
  : mTransform(_transform)
{
}

//=============================================================================
auto SE2StateSpace::State::getIsometry() const 
  -> const Isometry2d&
{
  return mTransform;
}

//=============================================================================
void SE2StateSpace::State::setIsometry(const Isometry2d& _transform)
{
  mTransform = _transform;
}

//=============================================================================
SE2StateSpace::SE2StateSpace()
  : mBounds()
{
  mBounds.col(0).setConstant(-std::numeric_limits<double>::infinity());
  mBounds.col(1).setConstant(+std::numeric_limits<double>::infinity());
}

//=============================================================================
SE2StateSpace::SE2StateSpace(const Bounds& _translationBounds)
  : mBounds(_translationBounds)
{
  for (size_t i = 0; i < mBounds.rows(); ++i)
  {
    if (mBounds(i, 0) > mBounds(i, 1))
    {
      std::stringstream msg;
      msg << "Lower bound exceeds upper bound for translation dimension "
          << i << ": " << mBounds(i, 0) << " > " << mBounds(i, 1) << ".";
      throw std::runtime_error(msg.str());
    }
  }
}

//=============================================================================
auto SE2StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
auto SE2StateSpace::getTranslationalBounds() const -> Bounds
{
  return mBounds;
}

//=============================================================================
auto SE2StateSpace::getIsometry(const State* _state) const
  -> const Isometry2d&
{
  return _state->getIsometry();
}

//=============================================================================
void SE2StateSpace::setIsometry(
  State* _state, const Isometry2d& _transform) const
{
  _state->setIsometry(_transform);
}


//=============================================================================
size_t SE2StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SE2StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SE2StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
auto SE2StateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  return std::make_shared<SE2StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<SE2StateSpace>(shared_from_this()),
    std::move(_rng));
}

//=============================================================================
void SE2StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mTransform = state1->mTransform * state2->mTransform;
}

//=============================================================================
void SE2StateSpace::expMap(
  const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);

  if (_tangent.rows() != 3)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 3"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  double angle = _tangent(0);
  Eigen::Vector2d translation = _tangent.bottomRows(2);

  Isometry2d transform(Isometry2d::Identity());
  transform.linear() = Eigen::Rotation2Dd(angle).matrix();
  transform.translation() = translation;

  out->mTransform = transform; 
}


//=============================================================================
int SE2StateSpace::getDimension() const
{
  return 3;
}

} // namespace statespace
} // namespace aikido
