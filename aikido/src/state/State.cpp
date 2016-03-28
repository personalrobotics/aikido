#include <aikido/state/State.hpp>

namespace aikido {
namespace state{


//=============================================================================
SO2State::SO2State(const Eigen::Rotation2D<double>& _rotation)
: mRotation(_rotation)
{
}


//=============================================================================
SO3State::SO3State(const Eigen::Quaternion<double>& _rotation)
: mRotation(_rotation)
{
}

//=============================================================================
SE2State::SE2State(Eigen::Isometry2d& _transform)
: mTransform(_transform)
{
}

//=============================================================================
SE3State::SE3State(const Eigen::Isometry3d& _transform)
: mTransform(_transform)
{
}

//=============================================================================
RealVectorState::RealVectorState(const Eigen::VectorXd& _q)
: mQ(_q)
{
}


//=============================================================================
RealVectorState::RealVectorState(double _val)
: mQ(Eigen::VectorXd(1))
{
  mQ(0) = _val;
}

//=============================================================================
CompoundState::CompoundState(std::vector<std::shared_ptr<State>>& _components)
:mComponents(_components)
{
 mComponents = flatten(_components); 
}

//=============================================================================
std::vector<std::shared_ptr<State>> CompoundState::flatten(
  std::vector<std::shared_ptr<State>>& _components)
{
  std::vector<std::shared_ptr<State>> components;

  for(auto&& _s : _components)
  {
    std::shared_ptr<CompoundState> s = std::dynamic_pointer_cast<
                                        CompoundState>(_s);

    if(!s)
    {
      components.push_back(_s);
      continue;
    }

    /// does this work?!
    components.insert(components.end(),
                      s->mComponents.begin(),
                      s->mComponents.end());

  }

  return components;
}


//=============================================================================
void SO2State::update(const StatePtr& _ds) 
{
  SO2StatePtr ds = std::dynamic_pointer_cast<SO2State>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not SO2StatePtr.");
  }

  mRotation = mRotation*(ds->mRotation);
}

//=============================================================================
StatePtr SO2State::clone()
{
  return std::make_shared<SO2State>(*this);
}

//=============================================================================
void SO3State::update(const StatePtr& _ds) 
{
  SO3StatePtr ds = std::dynamic_pointer_cast<SO3State>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not SO3StatePtr.");
  }

  mRotation = mRotation*(ds->mRotation);
}

//=============================================================================
StatePtr SO3State::clone()
{
  return std::make_shared<SO3State>(*this);
}

//=============================================================================
void SE2State::update(const StatePtr& _ds) 
{
  SE2StatePtr ds = std::dynamic_pointer_cast<SE2State>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not SE2StatePtr.");
  }

  mTransform = mTransform*(ds->mTransform);
}


//=============================================================================
StatePtr SE2State::clone()
{
  return std::make_shared<SE2State>(*this);
}

//=============================================================================
void SE3State::update(const StatePtr& _ds) 
{
  SE3StatePtr ds = std::dynamic_pointer_cast<SE3State>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not SE3StatePtr.");
  }

  mTransform = mTransform*(ds->mTransform);
}



//=============================================================================
StatePtr SE3State::clone()
{
  return std::make_shared<SE3State>(*this);
}

//=============================================================================
void RealVectorState::update(const StatePtr& _ds) 
{
  RealVectorStatePtr ds = std::dynamic_pointer_cast<RealVectorState>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not RealVectorStatePtr.");
  }

  if (mQ.rows() != ds->mQ.rows())
  {
    throw std::invalid_argument("_ds' dimension is not same as this state.");
  }

  mQ = mQ + ds->mQ;
}

//=============================================================================
StatePtr RealVectorState::clone()
{
  return std::make_shared<RealVectorState>(*this);
}

//=============================================================================
void CompoundState::update(const StatePtr& _ds) 
{
  CompoundStatePtr ds = std::dynamic_pointer_cast<CompoundState>(_ds);
  if (!ds)
  {
    throw std::invalid_argument("_ds is not CompoundStatePtr.");
  }

  if (ds->mComponents.size() != mComponents.size())
  {
    throw std::invalid_argument("_ds have different number of compoments.");

  }

  for(int i = 0; i< mComponents.size(); i++)
  {
    mComponents.at(i)->update(ds->mComponents.at(i));
  }

}


//=============================================================================
StatePtr CompoundState::clone()
{
  // TODO
  return std::make_shared<CompoundState>(*this);
}


}
}
