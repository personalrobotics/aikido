#include <aikido/sampleable/TSR.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>

namespace aikido {
namespace sampleable{

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
  wrap();
  validate();
}

//=============================================================================
TSR::TSR(const Eigen::Isometry3d& _T0_w,
         const Eigen::Matrix<double, 6, 2>& _Bw,
         const Eigen::Isometry3d& _Tw_e)
: mRng(std::unique_ptr<util::RNG>(new util::RNGWrapper<std::default_random_engine>(0)))
, mT0_w(_T0_w)
, mBw(_Bw)
, mTw_e(_Tw_e)
{
  wrap();
  validate();
}

//=============================================================================
TSR::TSR(const TSR& other)
: mRng(std::move(other.mRng->clone()))
, mT0_w(other.mT0_w)
, mTw_e(other.mTw_e)
, mBw(other.mBw)
{
  wrap();
  validate();
}


//=============================================================================
TSR::TSR(TSR&& other)
: mRng(std::move(other.mRng))
, mT0_w(other.mT0_w)
, mTw_e(other.mTw_e)
, mBw(other.mBw)
{
  wrap();
  validate();
}


//=============================================================================
TSR& TSR::operator=(const TSR& other)
{
  mRng = std::move(other.mRng->clone());
  mT0_w = other.mT0_w;
  mTw_e = other.mTw_e;
  mBw = other.mBw;
  
  wrap();
  validate();

  return *this;
}

//=============================================================================
TSR& TSR::operator=(TSR&& other)
{
  mRng = std::move(other.mRng);
  mT0_w = other.mT0_w;
  mTw_e = other.mTw_e;
  mBw = other.mBw;

  wrap();
  validate();

  return *this;
}


//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> TSR::sampler() const
{
  return TSRSampleGeneratorUniquePtr(new TSRSampleGenerator(mRng->clone(),
                                                            mT0_w,
                                                            mBw,
                                                            mTw_e));
}

//=============================================================================
void TSR::validate()
{
  // Assertion checks for min, max on bounds 
  for(int i = 0; i < 6; i++)
  {
    if(mBw(i, 0) > mBw(i, 1))
    {
      throw std::invalid_argument(
        "Lower bound must be less than upper bound.");
    }
  }
}

void TSR::wrap(){
  // Wrap angles to the interval [-PI, PI)
  double lower = -M_PI;
  for(int i = 3; i < 6; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      mBw(i,j) = fmod(mBw(i, j) - lower, 2*M_PI) + lower;
    }
  }
}

} // namespace sampleable
} // namespace aikido
