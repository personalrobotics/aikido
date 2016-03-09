#include <aikido/constraint/TSR.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>

namespace aikido {
namespace constraint {

//=============================================================================
TSR::TSR(std::unique_ptr<util::RNG> _rng,
         const Eigen::Isometry3d& _T0_w,
         const Eigen::Matrix<double, 6, 2>& _Bw,
         const Eigen::Isometry3d& _Tw_e)
: mRng(std::move(_rng))
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
, mT0_w(_T0_w)
, mBw(_Bw)
, mTw_e(_Tw_e)
{
  validate();
}

//=============================================================================
TSR::TSR(const TSR& other)
: mRng(std::move(other.mRng->clone()))
, mT0_w(other.mT0_w)
, mTw_e(other.mTw_e)
, mBw(other.mBw)
{
  validate();
}


//=============================================================================
TSR::TSR(TSR&& other)
: mRng(std::move(other.mRng))
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
  
  return *this;
}

//=============================================================================
TSR& TSR::operator=(TSR&& other)
{
  mRng = std::move(other.mRng);
  mT0_w = std::move(other.mT0_w);
  mTw_e = std::move(other.mTw_e);
  mBw = std::move(other.mBw);

  return *this;
}


//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> 
TSR::createSampleGenerator() const
{
  validate();

  return TSRSampleGeneratorUniquePtr(new TSRSampleGenerator(mRng->clone(),
                                                            mT0_w,
                                                            mBw,
                                                            mTw_e));
}

//=============================================================================
void TSR::validate() const
{
  if (!mRng)
  {
    throw std::invalid_argument(
      "Random generator is empty.");
  }

  // Assertion checks for min, max on bounds 
  for(int i = 0; i < 6; i++)
  {
    if(mBw(i, 0) > mBw(i, 1))
    {
      throw std::invalid_argument(
        "Lower bound must be less than upper bound.");
    }

    if (!(std::isfinite(mBw(i, 0)) && std::isfinite(mBw(i, 1))))
    {
      throw std::invalid_argument("Bounds must be finite.");
    }

  }

}


//=============================================================================
void TSR::setRNG(std::unique_ptr<util::RNG> rng)
{
  mRng = std::move(rng);
}

} // namespace constraint
} // namespace aikido
