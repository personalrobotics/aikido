#include <aikido/constraint/TSR.hpp>
#include <dart/common/Console.h>
#include <boost/format.hpp>
#include <dart/math/Geometry.h>
#include <aikido/util/PseudoInverse.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>


namespace aikido {
namespace constraint {

//=============================================================================
TSRConstraint::TSRConstraint(const TSRPtr& _tsr, int _maxIteration)
: mTsr(_tsr)
, mMaxIteration(_maxIteration)
{
}

//=============================================================================
bool TSRConstraint::contains(const state::StatePtr& _s) const
{
  Eigen::VectorXd values = getValue(_s);
  return values.isApproxToConstant(0);
}

//=============================================================================
boost::optional<state::StatePtr> TSRConstraint::project(
  const state::StatePtr& _s)
{
  using namespace state;

  int iteration = 0;

  // Copy of _s's state
  StatePtr sCopy = _s->clone();

  if(!sCopy)
  {
    throw std::invalid_argument("_s not clonable.");
  } 

  /// Newton's method on mDifferentiable
  while(!contains(sCopy) && iteration < mMaxIteration)
  {
    Eigen::VectorXd value = getValue(sCopy);
    JacobianPtr jac = getJacobian(sCopy);
   
    // SE3 
    SE3StatePtr sSE3 = convertToSE3(sCopy);
    SE3JacobianPtr jacSE3 = std::dynamic_pointer_cast<SE3Jacobian>(jac);
    if (sSE3 && jacSE3)
    {
      /// Jacobian use (w, v) order, so flip.
      Eigen::VectorXd flipVal(6);
      flipVal.head(3) = value.tail(3);
      flipVal.tail(3) = value.head(3);

      Eigen::Vector6d _ds = -1*util::pseudoinverse(jacSE3->mJacobian)*flipVal;

      Eigen::Isometry3d ds = dart::math::expMap(_ds);
      sSE3->update(std::make_shared<SE3State>(ds));

      continue;

    }else
    {
      throw std::invalid_argument("_s is not SE3StatePtr.");
    }
  }

  return sCopy;
}

//=============================================================================
size_t TSRConstraint::getConstraintDimension() const
{
  return 6;
}


//=============================================================================
state::SE3StatePtr TSRConstraint::convertToSE3(const state::StatePtr& _s) const
{
  using namespace state;

  SE3StatePtr sSE3 = std::dynamic_pointer_cast<SE3State>(_s);
  if (!sSE3)
  {
    CompoundStatePtr sComp = std::dynamic_pointer_cast<CompoundState>(_s);
    if (!sComp)
      throw std::invalid_argument("_s is neither SE3State nor CompoundState.");

    if (sComp->mComponents.size() != 1)
      throw std::invalid_argument("_s has more than 1 component.");

    sSE3 = std::dynamic_pointer_cast<SE3State>(sComp->mComponents.at(0));

    if(!sSE3)
      throw std::invalid_argument("CompoundState _s does not contain SE3.");
  }

  return sSE3;
}

//=============================================================================
Eigen::VectorXd TSRConstraint::getValue(const state::StatePtr& _s) const
{
  using namespace state;
  using TransformTraits = Eigen::TransformTraits;

  SE3StatePtr sSE3 = convertToSE3(_s);

  Eigen::Isometry3d T0_w_inv = mTsr->mT0_w.inverse(TransformTraits::Isometry);
  Eigen::Isometry3d Tw_e_inv = mTsr->mTw_e.inverse(TransformTraits::Isometry);
  Eigen::MatrixXd Tw_s_m = T0_w_inv.matrix()
                          *(sSE3->mTransform.matrix())
                          *Tw_e_inv.matrix();

  Eigen::Isometry3d Tw_s;
  Tw_s.matrix() = Tw_s_m;

  Eigen::Vector3d translation = Tw_s.translation(); 
  Eigen::Vector3d eulerZYX = ::dart::math::matrixToEulerZYX(Tw_s.rotation());

  Eigen::VectorXd distance(6);

  for(int i = 0; i < 3; ++i)
  {
    if (translation(i) < mTsr->mBw(i, 0))
      distance(i) = std::abs(translation(i) -  mTsr->mBw(i, 0));

    else if (translation(i) > mTsr->mBw(i, 1))
      distance(i) = std::abs(translation(i) -  mTsr->mBw(i, 1));

    else
      distance(i) = 0; 
  }


  /// TSR doesn't itself wrap, so what's the right way to compare? 
  for(int i = 3; i < 6; ++i)
  {
    if (eulerZYX(i - 3) <  mTsr->mBw(i, 0))
      distance(i) = std::abs(eulerZYX(i - 3) - mTsr->mBw(i, 0)); 

    else if (eulerZYX(i - 3) >  mTsr->mBw(i, 1))
      distance(i) = std::abs(eulerZYX(i - 3) -  mTsr->mBw(i, 1));

    else
      distance(i) = 0; 

  }

  return distance;
}

//=============================================================================
state::JacobianPtr TSRConstraint::getJacobian(const state::StatePtr& _s) const
{
  using namespace state;
  SE3JacobianPtr jac = std::make_shared<SE3Jacobian>(Eigen::MatrixXd::Identity(6,6));

  SE3StatePtr sSE3 = std::dynamic_pointer_cast<SE3State>(_s);
  if (!sSE3)
  {
    CompoundStatePtr sComp = std::dynamic_pointer_cast<CompoundState>(_s);
    if (!sComp)
      throw std::invalid_argument("_s is neither SE3State nor CompoundState.");

    if (sComp->mComponents.size() != 1)
      throw std::invalid_argument("_s has more than 1 component.");

    sSE3 = std::dynamic_pointer_cast<SE3State>(sComp->mComponents.at(0));

    if(!sSE3)
      throw std::invalid_argument("CompoundState _s does not contain SE3.");

    std::vector<JacobianPtr> jacs;
    jacs.push_back(jac);

    return std::make_shared<CompoundJacobian>(jacs);    
  }

  return jac;
}

//=============================================================================
std::vector<ConstraintType> TSRConstraint::getConstraintTypes() const
{
  std::vector<ConstraintType> constraints(3, ConstraintType::EQ);

  return constraints;
}

}
}


