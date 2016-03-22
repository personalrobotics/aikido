#ifndef AIKIDO_STATE_STATE_H
#define AIKIDO_STATE_STATE_H

#include <Eigen/Dense>

namespace aikido {
namespace state{

class State{

};

class SO2State: public State
{
public:
  SO2State(const double _angle=0);

  double mAngle;
};

class SO3State: public State
{
public:
  SO3State(const Eigen::Quaternion& _rotation=Eigen::Quaternion::Identity());

  Eigen::Quaternion mRotation;
};

class SE2State: public State
{
public:
  SE2State(const double _angle=0,
           const Eigen::Vector2d& _translation=Eigen::Vector2d::Zero());

  Eigen::Rotation2d mRotation;
  Eigen::Vector2d mTranslation;

};

class SE3State: public State
{
public:
  SE3State(const Eigen::Quaternion& _rotation=Eigen::Quaternion::Identity(),
           const Eigen::Vector3d _translation=Eigen::Vector3d::Zero());

  Eigen::Quaternion mRotation;
  Eigen::Vector3d mTranslation;
};

class RealVectorState: public State
{
public:
  RealVectorState(const Eigen::VectorXd& _q);

  Eigen::VectorXd mQ;
};

class CompoundState: public State
{
public:
  CompoundState(std::vector<const State&> _components);
  
  std::vector<const State&> mComponents;
};

} // state
} // aikido

#endif
