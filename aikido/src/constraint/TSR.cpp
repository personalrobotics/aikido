#include <aikido/constraint/TSR.hpp>
#include <dart/common/Console.h>
#include <dart/common/StlHelpers.h>
#include <dart/math/Geometry.h>
#include <boost/format.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>

using boost::format;
using boost::str;
using aikido::statespace::SE3StateSpace;

namespace aikido {
namespace constraint {

//=============================================================================
TSR::TSR(std::unique_ptr<util::RNG> _rng,
         const Eigen::Isometry3d& _T0_w,
         const Eigen::Matrix<double, 6, 2>& _Bw,
         const Eigen::Isometry3d& _Tw_e)
: mRng(std::move(_rng))
, mStateSpace(std::make_shared<SE3StateSpace>())
, mT0_w(_T0_w)
, mBw(_Bw)
, mTw_e(_Tw_e)
{
  validate();
}

//=============================================================================
TSR::TSR(const Eigen::Isometry3d& _T0_w,
         const Eigen::Matrix<double, 6, 2>& _Bw,
         const Eigen::Isometry3d& _Tw_e)
: mRng(std::unique_ptr<util::RNG>(
    new util::RNGWrapper<std::default_random_engine>(0)))
, mStateSpace(std::make_shared<SE3StateSpace>())
, mT0_w(_T0_w)
, mBw(_Bw)
, mTw_e(_Tw_e)
{
  validate();
}

//=============================================================================
TSR::TSR(const TSR& other)
: mRng(std::move(other.mRng->clone()))
, mStateSpace(std::make_shared<SE3StateSpace>())
, mT0_w(other.mT0_w)
, mTw_e(other.mTw_e)
, mBw(other.mBw)
{
  validate();
}

//=============================================================================
TSR::TSR(TSR&& other)
: mRng(std::move(other.mRng))
, mStateSpace(std::make_shared<SE3StateSpace>())
, mT0_w(other.mT0_w)
, mTw_e(other.mTw_e)
, mBw(other.mBw)
{
  validate();
}

//=============================================================================
TSR& TSR::operator=(const TSR& other)
{
  mRng = std::move(other.mRng->clone());
  mT0_w = other.mT0_w;
  mTw_e = other.mTw_e;
  mBw = other.mBw;

  // Intentionally don't assign StateSpace. 

  return *this;
}

//=============================================================================
TSR& TSR::operator=(TSR&& other)
{
  mRng = std::move(other.mRng);
  mStateSpace = std::move(other.mStateSpace);
  mT0_w = std::move(other.mT0_w);
  mTw_e = std::move(other.mTw_e);
  mBw = std::move(other.mBw);

  return *this;
}

//=============================================================================
std::shared_ptr<statespace::StateSpace> TSR::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::shared_ptr<statespace::SE3StateSpace> TSR::getSE3StateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> TSR::createSampleGenerator() const
{
  validate();

  if (!mRng)
    throw std::invalid_argument("Random generator is nullptr.");

  for (int i = 0; i < 6; ++i)
  for (int j = 0; j < 2; ++j)
  {
    if (!std::isfinite(mBw(i, j)))
      throw std::invalid_argument(str(
        format("Sampling requires finite bounds. Bw[%d, %d] is %f.")
          % i % j % mBw(i, j)));
  }

  if (mBw.col(0) == mBw.col(1))
    dtwarn << "[TSR::createSampleGenerator] This is a point TSR that represents"
              " a zero measure set in SE(3). The SampleGenerator<Isometry3d>"
              " returned by this function will sample the same pose infinitely"
              " many times.\n";

  return std::unique_ptr<TSRSampleGenerator>(new TSRSampleGenerator(
    mRng->clone(), mStateSpace, mT0_w, mBw, mTw_e));
}

//=============================================================================
void TSR::validate() const
{
  // Assertion checks for min, max on bounds 
  for (int i = 0; i < 6; i++)
  {
    if (mBw(i, 0) > mBw(i, 1))
      throw std::invalid_argument(str(
        format("Lower bound exceeds upper bound on dimension %d: %f > %f")
          % i % mBw(i, 0) % mBw(i, 1)));
  }
}

//=============================================================================
void TSR::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}


//=============================================================================
size_t TSR::getConstraintDimension() const
{
  return 6;
}

//=============================================================================
Eigen::VectorXd TSR::getValue(
  const statespace::StateSpace::State* _s) const
{
  using SE3StateSpace = statespace::SE3StateSpace;
  using SE3State = SE3StateSpace::State;

  auto se3state = static_cast<const SE3State*>(_s);
  Eigen::Isometry3d se3 = se3state->getIsometry();

  using TransformTraits = Eigen::TransformTraits;

  Eigen::Isometry3d T0_w_inv = mT0_w.inverse(TransformTraits::Isometry);
  Eigen::Isometry3d Tw_e_inv = mTw_e.inverse(TransformTraits::Isometry);
  Eigen::MatrixXd Tw_s_m = T0_w_inv.matrix()*se3.matrix()*Tw_e_inv.matrix();

  Eigen::Isometry3d Tw_s;
  Tw_s.matrix() = Tw_s_m;

  Eigen::Vector3d translation = Tw_s.translation(); 
  Eigen::Vector3d eulerZYX =  dart::math::matrixToEulerZYX(Tw_s.linear());

  Eigen::Vector6d distance;

  for(int i = 0; i < 3; ++i)
  {
    if (translation(i) < mBw(i, 0))
      distance(i) = std::abs(translation(i) -  mBw(i, 0));

    else if (translation(i) > mBw(i, 1))
      distance(i) = std::abs(translation(i) -  mBw(i, 1));

    else
      distance(i) = 0; 
  }

  /// TODO: TSR doesn't itself wrap, so what's the right way to compare? 
  for(int i = 3; i < 6; ++i)
  {
    if (eulerZYX(i - 3) <  mBw(i, 0))
      distance(i) = std::abs(eulerZYX(i - 3) - mBw(i, 0)); 

    else if (eulerZYX(i - 3) >  mBw(i, 1))
      distance(i) = std::abs(eulerZYX(i - 3) - mBw(i, 1));

    else
      distance(i) = 0; 

  }

  // TODO: Compare 8 equivalent representations

  return distance;

}

//=============================================================================
Eigen::MatrixXd TSR::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  // I'm just doing finite difference here.
  using SE3StateSpace = statespace::SE3StateSpace;
  using SE3State = SE3StateSpace::State;

  Eigen::Matrix6d jacobian;

  auto se3state = static_cast<const SE3State*>(_s);
  Eigen::Isometry3d se3 = se3state->getIsometry();

  Eigen::Vector6d twist = ::dart::math::logMap(se3);

  Eigen::Vector6d posit(twist), negat(twist);

  double eps = 1e-5;

  auto se3posit = mStateSpace->createState();
  auto se3negat = mStateSpace->createState();

  for(int i = 0; i < 6; ++i)
  {
    posit(i) = twist(i) + eps;
    negat(i) = twist(i) - eps;

    se3posit.setIsometry(::dart::math::expMap(posit));
    se3negat.setIsometry(::dart::math::expMap(negat));

    Eigen::Vector6d diff = getValue(se3posit) - getValue(se3negat);
    jacobian.col(i) = diff/(2*eps);

    posit(i) = twist(i);
    negat(i) = twist(i);
  }

  return jacobian;
}

//=============================================================================
std::vector<ConstraintType> TSR::getConstraintTypes() const
{
  return std::vector<ConstraintType>(6, ConstraintType::INEQ);

}

} // namespace constraint
} // namespace aikido
