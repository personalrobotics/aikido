#include <aikido/constraint/TSRChain.hpp>

using aikido::statespace::SE3;

const static size_t TSR_CHAIN_DIMENSION = 6;

namespace aikido {
namespace constraint {

class TSRChainSampleGenerator : public SampleGenerator
{
public:
  TSRChainSampleGenerator(const TSRChainSampleGenerator&) = delete;
  TSRChainSampleGenerator(TSRChainSampleGenerator&& other) = delete;
  TSRChainSampleGenerator& operator=(const TSRChainSampleGenerator& other) = delete;
  TSRChainSampleGenerator& operator=(TSRChainSampleGenerator&& other) = delete;
  virtual ~TSRChainSampleGenerator() = default; 

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
  TSRChainSampleGenerator();

  std::shared_ptr<statespace::SE3> mStateSpace;

  // True for point TSR.
  bool mPointTSR;

  // True if point TSR and has already been sampled.
  bool mPointTSRSampled;


  friend class TSRChain;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


TSRChain::TSRChain(const TSRChainConstraintType& type,
                   const std::vector<TSR>& tsrs)
         : mStateSpace(std::make_shared<SE3>()),
           mTSRChainConstraintType( type ),
           mTSRs( tsrs )
{
  validate();
}

//=============================================================================
TSRChain::TSRChain(const TSRChain& other)
         : mStateSpace(std::make_shared<SE3>()),
           mTSRChainConstraintType( other.mTSRChainConstraintType )
{
  mTSRs.clear();
  for(unsigned int i=0; i < other.mTSRs.size(); i++)
  {
    mTSRs.push_back( other.mTSRs[i] );
  }
  validate();
}

//=============================================================================
TSRChain::TSRChain(TSRChain&& other)
         : mStateSpace(std::make_shared<SE3>()),
           mTSRChainConstraintType( other.mTSRChainConstraintType )
{
  mTSRs.clear();
  for(unsigned int i=0; i < other.mTSRs.size(); i++)
  {
    mTSRs.push_back( other.mTSRs[i] );
  }
  validate();
}

//=============================================================================
TSRChain& TSRChain::operator=(const TSRChain& other)
{
  mTSRChainConstraintType = other.mTSRChainConstraintType;
  mTSRs.clear();
  for(unsigned int i=0; i < other.mTSRs.size(); i++)
  {
    mTSRs.push_back( other.mTSRs[i] );
  }
  // Intentionally don't assign StateSpace.
  return *this;
}

//=============================================================================
TSRChain& TSRChain::operator=(TSRChain&& other)
{
  mStateSpace = std::move(other.mStateSpace);
  mTSRChainConstraintType = other.mTSRChainConstraintType;
  mTSRs.clear();
  for(unsigned int i=0; i < other.mTSRs.size(); i++)
  {
    mTSRs.push_back( other.mTSRs[i] );
  }
  return *this;
}


//=============================================================================
std::shared_ptr<statespace::StateSpace> TSRChain::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::shared_ptr<statespace::SE3> TSRChain::getSE3() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> TSRChain::createSampleGenerator() const
{
  validate();

  return std::unique_ptr<TSRChainSampleGenerator>(
      new TSRChainSampleGenerator());
}

//=============================================================================
bool TSRChain::isSatisfied(const statespace::StateSpace::State* _s) const
{
  static constexpr double eps = 1e-6;
  Eigen::VectorXd dist;
  getValue(_s, dist);
  return dist.norm() < eps;
}

//=============================================================================
void TSRChain::validate() const
{

}

//=============================================================================
size_t TSRChain::getConstraintDimension() const { return TSR_CHAIN_DIMENSION; }

//=============================================================================
void TSRChain::getValue(const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _out) const
{

}

//=============================================================================
void TSRChain::getJacobian(const statespace::StateSpace::State* _s,
  Eigen::MatrixXd& _out) const
{

}

//=============================================================================
std::vector<ConstraintType> TSRChain::getConstraintTypes() const
{
  return std::vector<ConstraintType>(TSR_CHAIN_DIMENSION, 
                                     ConstraintType::INEQUALITY);
}

//=============================================================================
void TSRChain::append(const TSR& tsr)
{
  mTSRs.push_back(tsr);
}

//=============================================================================
TSRChainSampleGenerator::TSRChainSampleGenerator()
{

  mPointTSR = false;
  mPointTSRSampled = false;
}

//=============================================================================
statespace::StateSpacePtr TSRChainSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool TSRChainSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  return false;
}


//=============================================================================
bool TSRChainSampleGenerator::canSample() const
{
  if (mPointTSR && mPointTSRSampled)
    return false;

  return true;
}


//=============================================================================
int TSRChainSampleGenerator::getNumSamples() const
{
  if (mPointTSR && !mPointTSRSampled)
    return 1;

  if (mPointTSR && mPointTSRSampled)
    return 0;

  return NO_LIMIT;
}

} // namespace constraint
} // namespace aikido
