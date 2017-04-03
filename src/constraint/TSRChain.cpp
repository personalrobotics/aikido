#include <boost/format.hpp>
#include <aikido/constraint/TSRChain.hpp>

using boost::format;
using boost::str;
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

  /// Return a transform sampled from this TSR chain.
  ///
  /// This function creates a sample `Tw_s` by recursively sampling from a chain
  /// of TSRs, and returns the result:
  /// `CiT0_w * CiTw_s * CiTw_e`.
  ///
  /// \param _state State to store a sampling. 
  /// \return Whether sampling succeeds.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  bool canSample() const override;

  // Documentation inherited.
  int getNumSamples() const override;

private:
  // For internal use only.
  TSRChainSampleGenerator(std::shared_ptr<statespace::SE3> _stateSpace,
                          const std::vector<TSR>& tsrs);

  std::vector<TSR> mTSRs;

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
      new TSRChainSampleGenerator( mStateSpace, mTSRs ));
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
  //TODO: add possible validation
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
TSRChainSampleGenerator::TSRChainSampleGenerator(
      std::shared_ptr<statespace::SE3> _stateSpace,
      const std::vector<TSR>& tsrs)
  : mStateSpace(std::move(_stateSpace))
{
  mTSRs.clear();
  for(unsigned int i=0; i < tsrs.size(); i++)
  {
    tsrs[i].validate();
    mTSRs.push_back(tsrs[i]);
  }
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
  if (mTSRs.size() < 1)
  {
    return false;
  }
  
  Eigen::Isometry3d Tw_s;
  Eigen::Isometry3d T0_w;

  for(unsigned int i=0; i < mTSRs.size(); i++)
  {
    auto tsr = mTSRs[i];
    Tw_s = tsr.sampleTransformation();

    if (i==0)
    {
       T0_w = tsr.mT0_w * Tw_s * tsr.mTw_e;;
    }
    else
    {
       T0_w = T0_w * Tw_s * tsr.mTw_e;  
    }
  }

  mStateSpace->setIsometry(static_cast<SE3::State*>(_state), T0_w);
  return true;
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
