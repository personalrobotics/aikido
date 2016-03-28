#include <aikido/state/Jacobian.hpp>

namespace aikido {
namespace state{


//=============================================================================
RealVectorJacobian::RealVectorJacobian(const Eigen::MatrixXd& _jacobian)
: mJacobian(_jacobian)
{

}

//=============================================================================
SO2Jacobian::SO2Jacobian(const Eigen::VectorXd& _jacobian)
: mJacobian(_jacobian)
{

}


//=============================================================================
SE2Jacobian::SE2Jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 3>& _jacobian)
: mJacobian(_jacobian)
{

}

//=============================================================================
SO3Jacobian::SO3Jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 3>& _jacobian)
: mJacobian(_jacobian)
{

}

//=============================================================================
SE3Jacobian::SE3Jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 6> _jacobian)
: mJacobian(_jacobian)
{

}

//=============================================================================
CompoundJacobian::CompoundJacobian(std::vector<JacobianPtr> _jacobian)
: mJacobian(_jacobian)
{

}


//=============================================================================
JacobianPtr RealVectorJacobian::append(const JacobianPtr& _jac)
{
  
  RealVectorJacobianPtr j = std::dynamic_pointer_cast<RealVectorJacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not RealVectorJacobian.");
  }

  Eigen::MatrixXd jacobians(mJacobian.rows()+ j->mJacobian.rows(),
                            mJacobian.cols());

  jacobians.topRows(mJacobian.rows()) = mJacobian;
  jacobians.bottomRows(j->mJacobian.rows()) = j->mJacobian;

  return std::make_shared<RealVectorJacobian>(jacobians);
}

//=============================================================================
JacobianPtr SO2Jacobian::append(const JacobianPtr& _jac)
{
  
  SO2JacobianPtr j = std::dynamic_pointer_cast<SO2Jacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not SO2Jacobian.");
  }

  Eigen::VectorXd jacobians(mJacobian.rows()+ j->mJacobian.rows());

  jacobians.topRows(mJacobian.rows()) = mJacobian;
  jacobians.bottomRows(j->mJacobian.rows()) = j->mJacobian;

  return std::make_shared<SO2Jacobian>(jacobians);
}


//=============================================================================
JacobianPtr SE2Jacobian::append(const JacobianPtr& _jac)
{
  
  SE2JacobianPtr j = std::dynamic_pointer_cast<SE2Jacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not SE2Jacobian.");
  }

  Eigen::MatrixXd jacobians(mJacobian.rows()+ j->mJacobian.rows(),
                            mJacobian.cols());

  jacobians.topRows(mJacobian.rows()) = mJacobian;
  jacobians.bottomRows(j->mJacobian.rows()) = j->mJacobian;

  return std::make_shared<SE2Jacobian>(jacobians);
}


//=============================================================================
JacobianPtr SO3Jacobian::append(const JacobianPtr& _jac)
{
  
  SO3JacobianPtr j = std::dynamic_pointer_cast<SO3Jacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not SO3Jacobian.");
  }

  Eigen::MatrixXd jacobians(mJacobian.rows()+ j->mJacobian.rows(),
                            mJacobian.cols());

  jacobians.topRows(mJacobian.rows()) = mJacobian;
  jacobians.bottomRows(j->mJacobian.rows()) = j->mJacobian;

  return std::make_shared<SO3Jacobian>(jacobians);
}


//=============================================================================
JacobianPtr SE3Jacobian::append(const JacobianPtr& _jac)
{
  
  SE3JacobianPtr j = std::dynamic_pointer_cast<SE3Jacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not SE3Jacobian.");
  }

  Eigen::MatrixXd jacobians(mJacobian.rows()+ j->mJacobian.rows(),
                            mJacobian.cols());

  jacobians.topRows(mJacobian.rows()) = mJacobian;
  jacobians.bottomRows(j->mJacobian.rows()) = j->mJacobian;

  return std::make_shared<SE3Jacobian>(jacobians);
}


//=============================================================================
JacobianPtr CompoundJacobian::append(const JacobianPtr& _jac)
{
  
  CompoundJacobianPtr j = std::dynamic_pointer_cast<CompoundJacobian>(_jac);

  if (!j)
  {
    throw std::invalid_argument("_jac is not CompoundJacobian.");
  }

  std::vector<JacobianPtr> jacobians;
  jacobians.reserve(mJacobian.size());

  for(int i = 0; i < jacobians.size(); ++i)
  {
    JacobianPtr innerJ = mJacobian.at(i)->append(j->mJacobian.at(i));
    jacobians.emplace_back(innerJ);
  }

  return std::make_shared<CompoundJacobian>(jacobians);
}


}
}
