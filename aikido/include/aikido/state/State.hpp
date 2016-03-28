#ifndef AIKIDO_STATE_STATE_H
#define AIKIDO_STATE_STATE_H

#include <memory>
#include <Eigen/Dense>

namespace aikido {
namespace state{

class State;
class SO2State;
class SO3State;
class SE2State;
class SE3State;
class RealVectorState;
class CompoundState;

using StatePtr = std::shared_ptr<State>;
using SO2StatePtr = std::shared_ptr<SO2State>;
using SO3StatePtr = std::shared_ptr<SO3State>;
using SE2StatePtr = std::shared_ptr<SE2State>;
using SE3StatePtr = std::shared_ptr<SE3State>;
using RealVectorStatePtr = std::shared_ptr<RealVectorState>;
using CompoundStatePtr = std::shared_ptr<CompoundState>;


class State{
public:
  /// Updates state. State type should match the actual state type.
  virtual void update(const StatePtr& _ds) = 0;
  virtual StatePtr clone() = 0;
};


class SO2State: public State
{
public:
  SO2State(const Eigen::Rotation2D<double>& _rotation);

  /// this <- this*ds 
  void update(const StatePtr& _ds) override;
  StatePtr clone() override;

  Eigen::Rotation2D<double> mRotation;

};


class SO3State: public State
{
public:
  SO3State(const Eigen::Quaternion<double>& _rotation);

  void update(const StatePtr& _ds) override;
  StatePtr clone() override;
  
  Eigen::Quaternion<double> mRotation;
};

class SE2State: public State
{
public:
  SE2State(Eigen::Isometry2d& _transform);

  void update(const StatePtr& _ds) override;
  StatePtr clone() override;
  
  Eigen::Isometry2d mTransform;

};

class SE3State: public State
{
public:
  SE3State(const Eigen::Isometry3d& _transform);

  void update(const StatePtr& _ds) override;
  StatePtr clone() override;
  
  Eigen::Isometry3d mTransform;

};


class RealVectorState: public State
{
public:
  RealVectorState(const Eigen::VectorXd& _q);
  
  /// single-value state
  RealVectorState(double _val);

  void update(const StatePtr& _ds) override;
  StatePtr clone() override;
  
  Eigen::VectorXd mQ;
};


class CompoundState: public State
{
public:
  CompoundState(std::vector<StatePtr> & _components);

  // TODO
  // CompoundState(const CompoundState& other);
  // CompoundState(CompoundState&& other);
  // CompoundState& operator=(const CompoundState& other);
  // CompoundState& operator=(CompoundState&& other);


  void update(const StatePtr& _ds) override;
  StatePtr clone() override;
  

  /// Flattened. 
  std::vector<StatePtr> mComponents;

private:
  /// flatten mComponents so that none of the elements are CompoundState.
  std::vector<StatePtr> flatten(
    std::vector<StatePtr>& _components);
};

} // state
} // aikido

#endif
