#include "aikido/control/Executor.hpp"

#include <chrono>
#include <vector>
#include <set>

#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/pointers.hpp"
#include <dart/dart.hpp>

namespace aikido {
namespace control {

// Init static DofManager to empty vectors
std::unordered_map<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>> Executor::mDofManager = std::unordered_map<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>
{
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::STATE, std::set<dart::dynamics::DegreeOfFreedom*>()},
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::POSITION, std::set<dart::dynamics::DegreeOfFreedom*>()},
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::VELOCITY, std::set<dart::dynamics::DegreeOfFreedom*>()},
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::EFFORT, std::set<dart::dynamics::DegreeOfFreedom*>()},
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::TRAJECTORY, std::set<dart::dynamics::DegreeOfFreedom*>()},
    std::pair<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>>{ExecutorType::MODE, std::set<dart::dynamics::DegreeOfFreedom*>()}
};

//==============================================================================
Executor::Executor(
  const std::set<ExecutorType>& types,
  const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
  const std::chrono::milliseconds threadRate)
: mThreadRate(threadRate), mThread(nullptr), mTypes(types), mDofs(dofs)
{

    registerDofs();
    if(!mDofsRegistered) {
        dtwarn << "Could not register DoFs, resources locked by another executor." << std::endl;
    }
}

//==============================================================================
Executor::Executor(
  const ExecutorType type,
  const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
  std::chrono::milliseconds threadRate)
: Executor(std::set<ExecutorType>{type}, dofs, threadRate)
{
}

//==============================================================================
Executor::~Executor()
{
releaseDofs();
}

//==============================================================================
std::set<ExecutorType> Executor::getTypes() const
{
return mTypes;
}

//==============================================================================
std::vector<dart::dynamics::DegreeOfFreedom*> Executor::getDofs() const
{
return mDofs;
}

//==============================================================================
void Executor::start()
{
if (!mThread && mDofsRegistered)
{
  mThread = std::make_unique<aikido::common::ExecutorThread>(
      std::bind(&Executor::spin, this), mThreadRate);
}
}

//==============================================================================
void Executor::stop()
{
if (mThread)
{
  mThread->stop();
  mThread.reset();
}
}

//==============================================================================
bool Executor::registerDofs()
{
    // Return if already locked
    if(mDofsRegistered) return true;

    std::lock_guard<std::mutex> lock(mMutex);

    // Check if resources are locked
    // If they are, we can't register
    for(auto type : getTypes()) {
        // No need to lock READONLY type
        if(type == ExecutorType::READONLY) continue;

        auto lockedDofs = mDofManager[type];
        for(auto dof : getDofs()) {
          if(lockedDofs.find(dof) != lockedDofs.end()) {
            // Lock already exists
            dtwarn << "Could not register DoF: " << dof->getName()
                  << "; Already locked." << std::endl;
            return false;
          }
        }
    }

    // Lock resources
    for(auto type : getTypes()) {
        // No need to lock READONLY type
        if(type == ExecutorType::READONLY) continue;

        for(auto dof : getDofs()) {
          mDofManager[type].insert(dof);
        }
    }

    mDofsRegistered = true;
    return true;
}

//==============================================================================
void Executor::releaseDofs()
{
    // Return if already unlocked
    if(mDofsRegistered) return;

    std::lock_guard<std::mutex> lock(mMutex);

    // Unlock resources
    for(auto type : getTypes()) {
        // No need to lock READONLY type
        if(type == ExecutorType::READONLY) continue;

        for(auto dof : getDofs()) {
          mDofManager[type].erase(dof);
        }
    }

    mDofsRegistered = false;
}

} // namespace control
} // namespace aikido
