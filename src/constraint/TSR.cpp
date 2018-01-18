#include <aikido/constraint/TSR.hpp>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>
#include <boost/format.hpp>
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <dart/math/Geometry.hpp>

using boost::format;
using boost::str;
using aikido::statespace::SE3;

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

  // Documentation inherited.
  int getNumSamples() const override;

private:
  // For internal use only.
  TSRSampleGenerator(
      std::unique_ptr<common::RNG> _rng,
      std::shared_ptr<statespace::SE3> _stateSpace,
      const Eigen::Isometry3d& _T0_w,
      const Eigen::Matrix<double, 6, 2>& _Bw,
      const Eigen::Isometry3d& _Tw_e);

  std::unique_ptr<common::RNG> mRng;

  std::shared_ptr<statespace::SE3> mStateSpace;

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

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
TSR::TSR(
    std::unique_ptr<common::RNG> _rng,
    const Eigen::Isometry3d& _T0_w,
    const Eigen::Matrix<double, 6, 2>& _Bw,
    const Eigen::Isometry3d& _Tw_e,
    double _testableTolerance)
  : mT0_w(_T0_w)
  , mBw(_Bw)
  , mTw_e(_Tw_e)
  , mTestableTolerance(_testableTolerance)
  , mRng(std::move(_rng))
  , mStateSpace(std::make_shared<SE3>())
{
  validate();
}

//==============================================================================
TSR::TSR(
    const Eigen::Isometry3d& _T0_w,
    const Eigen::Matrix<double, 6, 2>& _Bw,
    const Eigen::Isometry3d& _Tw_e,
    double _testableTolerance)
  : mT0_w(_T0_w)
  , mBw(_Bw)
  , mTw_e(_Tw_e)
  , mTestableTolerance(_testableTolerance)
  , mRng(
        std::unique_ptr<common::RNG>(
            new common::RNGWrapper<std::default_random_engine>(0)))
  , mStateSpace(std::make_shared<SE3>())
{
  validate();
}

//==============================================================================
TSR::TSR(const TSR& other)
  : mT0_w(other.mT0_w)
  , mBw(other.mBw)
  , mTw_e(other.mTw_e)
  , mTestableTolerance(other.mTestableTolerance)
  , mRng(other.mRng->clone())
  , mStateSpace(std::make_shared<SE3>())
{
  validate();
}

//==============================================================================
TSR::TSR(TSR&& other)
  : mT0_w(other.mT0_w)
  , mBw(other.mBw)
  , mTw_e(other.mTw_e)
  , mTestableTolerance(other.mTestableTolerance)
  , mRng(std::move(other.mRng))
  , mStateSpace(std::make_shared<SE3>())
{
  validate();
}

//==============================================================================
TSR& TSR::operator=(const TSR& other)
{

  mT0_w = other.mT0_w;
  mBw = other.mBw;
  mTw_e = other.mTw_e;
  mTestableTolerance = other.mTestableTolerance;
  mRng = other.mRng->clone();

  // Intentionally don't assign StateSpace.

  return *this;
}

//==============================================================================
TSR& TSR::operator=(TSR&& other)
{
  mT0_w = std::move(other.mT0_w);
  mBw = std::move(other.mBw);
  mTw_e = std::move(other.mTw_e);
  mTestableTolerance = other.mTestableTolerance;
  mRng = std::move(other.mRng);
  mStateSpace = std::move(other.mStateSpace);

  return *this;
}

//==============================================================================
std::shared_ptr<statespace::StateSpace> TSR::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::shared_ptr<statespace::SE3> TSR::getSE3() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<SampleGenerator> TSR::createSampleGenerator() const
{
  validate();

  if (!mRng)
    throw std::invalid_argument("Random generator is nullptr.");

  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      if (!std::isfinite(mBw(i, j)))
        throw std::invalid_argument(
            str(format("Sampling requires finite bounds. Bw[%d, %d] is %f.") % i
                % j
                % mBw(i, j)));
    }
  }

  if (mBw.col(0) == mBw.col(1))
  {
    dtwarn << "[TSR::createSampleGenerator] This is a point TSR that represents"
              " a zero measure set in SE(3). The SampleGenerator<Isometry3d>"
              " returned by this function will sample the same pose infinitely"
              " many times.\n";
  }

  return std::unique_ptr<TSRSampleGenerator>(
      new TSRSampleGenerator(mRng->clone(), mStateSpace, mT0_w, mBw, mTw_e));
}

//==============================================================================
bool TSR::isSatisfied(
    const statespace::StateSpace::State* _s, TestableOutcome* outcome) const
{
  auto defaultOutcomeObject
      = dynamic_cast_or_throw<DefaultTestableOutcome>(outcome);

  Eigen::VectorXd dist;
  getValue(_s, dist);

  bool isSatisfiedResult = dist.norm() < mTestableTolerance;
  if (defaultOutcomeObject)
    defaultOutcomeObject->setSatisfiedFlag(isSatisfiedResult);
  return isSatisfiedResult;
}

//==============================================================================
std::unique_ptr<TestableOutcome> TSR::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
}

//==============================================================================
void TSR::validate() const
{
  // Assertion checks for min, max on bounds
  for (int i = 0; i < 6; i++)
  {
    if (mBw(i, 0) > mBw(i, 1))
    {
      throw std::logic_error(
          str(format("Lower bound exceeds upper bound on dimension %d: %f > %f")
              % i
              % mBw(i, 0)
              % mBw(i, 1)));
    }
  }
}

//==============================================================================
void TSR::setRNG(std::unique_ptr<common::RNG> rng)
{
  mRng = std::move(rng);
}

//==============================================================================
std::size_t TSR::getConstraintDimension() const
{
  return 6;
}

//==============================================================================
void TSR::getValue(
    const statespace::StateSpace::State* _s, Eigen::VectorXd& _out) const
{
  using SE3 = statespace::SE3;
  using SE3State = SE3::State;

  auto se3state = static_cast<const SE3State*>(_s);
  Eigen::Isometry3d se3 = se3state->getIsometry();

  using TransformTraits = Eigen::TransformTraits;

  Eigen::Isometry3d T0_w_inv = mT0_w.inverse(TransformTraits::Isometry);
  Eigen::Isometry3d Tw_e_inv = mTw_e.inverse(TransformTraits::Isometry);
  Eigen::Isometry3d Tw_s = T0_w_inv * se3 * Tw_e_inv;

  Eigen::Vector3d translation = Tw_s.translation();
  Eigen::Vector3d eulerOrig = dart::math::matrixToEulerZYX(Tw_s.linear());
  Eigen::Vector3d eulerZYX = eulerOrig.reverse();

  _out.resize(6);

  for (int i = 0; i < 3; ++i)
  {
    if (translation(i) < mBw(i, 0))
      _out(i) = std::abs(translation(i) - mBw(i, 0));

    else if (translation(i) > mBw(i, 1))
      _out(i) = std::abs(translation(i) - mBw(i, 1));

    else
      _out(i) = 0;
  }

  for (int i = 3; i < 6; ++i)
  {
    // Find n such that: 2*n*pi <= mBw(i, 0) < 2*(n+1)*pi
    int n = mBw(i, 0) / (2 * M_PI);

    // Map eulerZYX(i-3) to [2*n*pi, 2*(n+1)*pi)
    double angle = M_PI * 2 * n + eulerZYX(i - 3);

    // check if angle is within bound
    if ((angle >= mBw(i, 0) && angle <= mBw(i, 1))
        || (angle + M_PI * 2 >= mBw(i, 0) && angle + M_PI * 2 <= mBw(i, 1))
        || (angle - M_PI * 2 >= mBw(i, 0) && angle - M_PI * 2 <= mBw(i, 1)))
    {
      _out(i) = 0;
      continue;
    }

    // Take min-distance between angle and either side of bound
    if (angle < mBw(i, 0))
      _out(i) = std::min(mBw(i, 0) - angle, angle - (mBw(i, 1) - 2 * M_PI));

    else if (mBw(i, 1) < angle)
      _out(i) = std::min(angle - mBw(i, 1), mBw(i, 0) + 2 * M_PI - angle);
  }
}

//==============================================================================
void TSR::getJacobian(
    const statespace::StateSpace::State* _s, Eigen::MatrixXd& _out) const
{
  using SE3 = statespace::SE3;
  using SE3State = SE3::State;

  _out.resize(6, 6);

  auto se3state = static_cast<const SE3State*>(_s);
  Eigen::Isometry3d se3 = se3state->getIsometry();

  Eigen::Vector6d twist = ::dart::math::logMap(se3);

  Eigen::Vector6d posit(twist), negat(twist);

  static constexpr double eps = 1e-5;

  auto se3posit = mStateSpace->createState();
  auto se3negat = mStateSpace->createState();

  // Finite Differencing.
  for (int i = 0; i < 6; ++i)
  {
    posit(i) = twist(i) + eps;
    negat(i) = twist(i) - eps;

    se3posit.setIsometry(::dart::math::expMap(posit));
    se3negat.setIsometry(::dart::math::expMap(negat));

    Eigen::VectorXd positValue, negatValue;
    getValue(se3posit, positValue);
    getValue(se3negat, negatValue);

    Eigen::Vector6d diff = positValue - negatValue;
    _out.col(i) = diff / (2 * eps);

    posit(i) = twist(i);
    negat(i) = twist(i);
  }
}

//==============================================================================
std::vector<ConstraintType> TSR::getConstraintTypes() const
{
  return std::vector<ConstraintType>(6, ConstraintType::INEQUALITY);
}

//==============================================================================
bool TSR::project(
    const statespace::StateSpace::State* /*_s*/,
    statespace::StateSpace::State* /*_out*/) const
{
  // TODO
  return false;
}

//==============================================================================
TSRSampleGenerator::TSRSampleGenerator(
    std::unique_ptr<common::RNG> _rng,
    std::shared_ptr<SE3> _stateSpace,
    const Eigen::Isometry3d& _T0_w,
    const Eigen::Matrix<double, 6, 2>& _Bw,
    const Eigen::Isometry3d& _Tw_e)
  : mRng(std::move(_rng))
  , mStateSpace(std::move(_stateSpace))
  , mT0_w(_T0_w)
  , mBw(_Bw)
  , mTw_e(_Tw_e)
{
  if (!mRng)
  {
    throw std::invalid_argument("Random generator is empty.");
  }

  if (mBw.col(0) == mBw.col(1))
    mPointTSR = true;
  else
    mPointTSR = false;

  mPointTSRSampled = false;
}

//==============================================================================
statespace::StateSpacePtr TSRSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
bool TSRSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  if (mPointTSR && mPointTSRSampled)
    return false;

  using statespace::SE3;

  Eigen::Vector3d translation;
  Eigen::Vector3d angles;

  if (mPointTSR)
  {
    translation = mBw.block(0, 0, 3, 1);
    angles = mBw.block(3, 0, 3, 1);

    mPointTSRSampled = true;
  }
  else
  {
    std::vector<std::uniform_real_distribution<double> > distributions;
    for (int i = 0; i < 6; i++)
      distributions.emplace_back(mBw(i, 0), mBw(i, 1));

    for (int i = 0; i < 3; i++)
      translation(i) = distributions.at(i)(*mRng);

    for (int i = 0; i < 3; i++)
      angles(i) = distributions.at(i + 3)(*mRng);
  }

  Eigen::Isometry3d Tw_s;
  Tw_s.setIdentity();
  Tw_s.translation() = translation;
  Tw_s.linear() = dart::math::eulerZYXToMatrix(angles.reverse());

  Eigen::Isometry3d T0_s(mT0_w * Tw_s * mTw_e);
  mStateSpace->setIsometry(static_cast<SE3::State*>(_state), T0_s);

  return true;
}

//==============================================================================
double TSR::getTestableTolerance()
{
  return mTestableTolerance;
}

//==============================================================================
void TSR::setTestableTolerance(double _testableTolerance)
{
  mTestableTolerance = _testableTolerance;
}

//==============================================================================
bool TSRSampleGenerator::canSample() const
{
  if (mPointTSR && mPointTSRSampled)
    return false;

  return true;
}

//==============================================================================
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
