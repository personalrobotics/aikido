#ifndef AIKIDO_STATE_JACOBIAN_H
#define AIKIDO_STATE_JACOBIAN_H

#include "State.hpp"
#include <memory>
#include <Eigen/Dense>

namespace aikido {
namespace state{

class Jacobian;
class RealVectorJacobian;
class SO2Jacobian;
class SO3Jacobian;
class SE2Jacobian;
class SE3Jacobian;
class CompoundJacobian;

using JacobianPtr = std::shared_ptr<Jacobian>;
using RealVectorJacobianPtr = std::shared_ptr<RealVectorJacobian>;
using SO2JacobianPtr = std::shared_ptr<SO2Jacobian>;
using SO3JacobianPtr = std::shared_ptr<SO3Jacobian>;
using SE2JacobianPtr = std::shared_ptr<SE2Jacobian>;
using SE3JacobianPtr = std::shared_ptr<SE3Jacobian>;
using CompoundJacobianPtr = std::shared_ptr<CompoundJacobian>;

class Jacobian{
public:
  virtual ~Jacobian(){};

  /// Returns a concatenation of this and _jac. 
  /// This and _jac must be same type.
  /// The returned Jac will be also the same type. 
  virtual JacobianPtr append(const JacobianPtr& _jac) = 0; 
};

class RealVectorJacobian: public Jacobian
{
public:
  RealVectorJacobian(const Eigen::MatrixXd& _jacobian);
  virtual ~RealVectorJacobian()=default;


  /// Returns (n1+n2)xm matrix where n1 is the this' constriants, 
  /// n2 is the constraints of _jac.
  JacobianPtr append(const JacobianPtr& _jac) override;

  /// nxm matrix for n constraints, m-dimension state
  const Eigen::MatrixXd mJacobian;
};

class SO2Jacobian: public Jacobian
{
public:
  SO2Jacobian(const Eigen::VectorXd& _jacobian);
  virtual ~SO2Jacobian()=default;

  /// Returns (n1+n2)x1 vector
  JacobianPtr append(const JacobianPtr& _jac) override;

  /// nx1 vector for n constraints
  const Eigen::VectorXd mJacobian;
};

class SE2Jacobian: public Jacobian
{
public:
  SE2Jacobian(const Eigen::Matrix<double, Eigen::Dynamic, 3>& _jacobian);
  virtual ~SE2Jacobian()=default;

  /// Returns (n1+n2)x3 matrix
  JacobianPtr append(const JacobianPtr& _jac) override;

  /// nx3 matrix (w, v_x, v_y)
  const Eigen::Matrix<double, Eigen::Dynamic, 3> mJacobian;
};

class SO3Jacobian: public Jacobian
{
public:
  SO3Jacobian(const Eigen::Matrix<double, Eigen::Dynamic, 3>& _jacobian);
  virtual ~SO3Jacobian()=default;

  /// Returns (n1+n2)x3 matrix
  JacobianPtr append(const JacobianPtr& _jac) override;

  /// nx3 matrix for n constriants
  const Eigen::Matrix<double, Eigen::Dynamic, 3> mJacobian;
};

class SE3Jacobian: public Jacobian
{
public:
  SE3Jacobian(const Eigen::Matrix<double, Eigen::Dynamic, 6> _jacobian);
  virtual ~SE3Jacobian()=default;

  /// Returns (n1+n2)x3 matrix
  JacobianPtr append(const JacobianPtr& _jac) override;
  /// nx6, 6 coordinates are for twist (w, v)
  const Eigen::Matrix<double, Eigen::Dynamic, 6> mJacobian;
};


/// Jacobian for CompoundState. kth JacobianPtr maps to kth StatePtr.
class CompoundJacobian: public Jacobian
{
public:
  CompoundJacobian(std::vector<JacobianPtr> _jacobian);
  virtual ~CompoundJacobian()=default;

  /// Returns CompoundJacobian where each inner Jacobian is
  /// a concatenation of two jacobians. 
  /// Both this and _jac should have same number of elements in mJacobian, 
  /// and each pair of elements should correnspond to the same state.
  JacobianPtr append(const JacobianPtr& _jac) override;

  const std::vector<JacobianPtr> mJacobian;
};


}
}

#endif
