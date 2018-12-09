#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/planner/dart/DartProblem.hpp"
#include "aikido/planner/dart/DartPlanner.hpp"

#include <thread>
#include <future>

namespace aikido {
namespace planner {
namespace dart{

// Planning call for individual planner.
static void _plan(
      const PlannerPtr& planner,
      const std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>& promise,
      const ConstProblemPtr& problem,
      const std::shared_ptr<Planner::Result>& result)
{
  trajectory::TrajectoryPtr trajectory = planner->plan(*problem, result.get());
  if (trajectory)
  {
    promise->set_value(trajectory);
    return;
  }

  promise->set_value(nullptr);
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    std::vector<::dart::collision::CollisionDetectorPtr> collisionDetectors,
    const std::vector<PlannerPtr>& planners)
: ParallelMetaPlanner(std::move(stateSpace), planners)
, mMetaSkeleton(metaSkeleton)
, mCollisionDetectors(std::move(collisionDetectors))
, mRunning(false)
{
  // Do nothing
  // TODO: need to acquire metaSkeletons from the planners
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      std::vector<::dart::collision::CollisionDetectorPtr> collisionDetectors,
      const PlannerPtr& planner,
      const std::vector<common::RNG*> rngs)
: ParallelMetaPlanner(std::move(stateSpace))
, mMetaSkeleton(metaSkeleton)
, mCollisionDetectors(std::move(collisionDetectors))
, mRunning(false)
{
  if (rngs.size() > 0 && mCollisionDetectors.size() != rngs.size())
  {
    std::stringstream ss;
    ss << "Number of collision detectors [" << mCollisionDetectors.size() << "] does not match number of RNGs [ "
      << rngs.size() << "]."  << std::endl;
    throw std::invalid_argument(ss.str());
  }

  auto castedPlanner = std::dynamic_pointer_cast<const DartPlanner>(planner);

  auto numCopies = mCollisionDetectors.size();
  mPlanners.reserve(numCopies);
  mClonedMetaSkeletons.reserve(numCopies);

  for (std::size_t i = 0; i < mCollisionDetectors.size(); ++i)
  {
    std::cout << "Cloning " << i << "th planner"  << std::endl;
    std::cout << "Cloning metaskeleton"  << std::endl;

    auto clonedMetaSkeleton = util::clone(mMetaSkeleton);
    mClonedMetaSkeletons.emplace_back(clonedMetaSkeleton);
    if (rngs.size() == numCopies)
    {
      if (castedPlanner)
        mPlanners.emplace_back(castedPlanner->clone(clonedMetaSkeleton, rngs[i]));
      else
        mPlanners.emplace_back(planner->clone(rngs[i]));
    }
    else
    {
      if (castedPlanner)
        mPlanners.emplace_back(castedPlanner->clone(clonedMetaSkeleton));
      else
        mPlanners.emplace_back(planner->clone());
    }
  }

}

//==============================================================================
trajectory::TrajectoryPtr ConcreteParallelMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mRunning)
    {
      throw std::runtime_error("Currently planning another problem");
    }

    mRunning = true;
  }

  std::vector<std::future<trajectory::TrajectoryPtr>> futures;
  std::vector<std::thread> threads;
  std::vector<ProblemPtr> clonedProblems;
  std::vector<std::shared_ptr<Result>> results;

  // Check if problem is DartProblem
  const DartProblem* dart_problem = dynamic_cast<const DartProblem*>(&problem);
  std::vector<std::shared_ptr<Problem>> shared_problems;
  std::shared_ptr<Problem> shared_problem;
  if (!dart_problem)
    shared_problem = problem.clone();

  results.reserve(mPlanners.size());

  for (std::size_t i = 0; i < mPlanners.size(); ++i)
  {
    if (!mPlanners[i]->canSolve(problem))
    {
      std::cout << "Skip [" << i << "]th planner" << std::endl;
      continue;
    }

    auto result = std::make_shared<Result>();

    results.push_back(result);
    auto promise = std::make_shared<std::promise<trajectory::TrajectoryPtr>>();

    if (dart_problem)
    {
      auto shared_problem = dart_problem->clone(mCollisionDetectors[i],
          mClonedMetaSkeletons[i]);
      std::cout << "Construct thread with dart problem" << std::endl;
      threads.push_back(std::thread(_plan, mPlanners[i], promise, shared_problem, result));
    }
    else
    {
      std::cout << "Not dart problem" << std::endl;
      threads.push_back(std::thread(_plan, mPlanners[i], promise, shared_problem, result));
    }

    futures.push_back(promise->get_future());
  }

  std::vector<bool> future_retrieved(results.size(), false);
  do
  {
    std::size_t num_failures = 0;
    for(std::size_t i = 0; i < futures.size(); ++i)
    {
      if (future_retrieved[i])
        continue;

      auto status = futures[i].wait_for(std::chrono::milliseconds(1000));
      if (status == std::future_status::ready)
      {
        std::cout << i << "th future ready" << std::endl;

        future_retrieved[i] = true;
        auto trajectory = futures[i].get();
        if (trajectory)
        {
          std::cout << i << "th future succeeded." << std::endl;
          // TODO: kill them forcefully (need a stopable thread)
          for(auto planner : mPlanners)
            planner->stopPlanning();

          for(auto& thread: threads)
            if (thread.joinable())
              thread.join();
          {
            std::lock_guard<std::mutex> lock(mMutex);
            mRunning = false;
          }
          // TODO:: copy result
          std::cout << "Problem use count " << shared_problem.use_count() << std::endl;
          std::cout << "Returning trajectory" << std::endl;
          return trajectory;
        }
        else
        {
          std::cout << i << "th future failed." << std::endl;
          num_failures++;
        }
      }else
      {
        std::cout << i << "th future not ready" << std::endl;
      }

    }
    if (num_failures == mPlanners.size())
    {
      // TODO: make this into a separate function
      for(auto& thread: threads)
        if (thread.joinable())
          thread.join();
      {
        std::lock_guard<std::mutex> lock(mMutex);
        mRunning = false;
      }
      std::cout << "Failed by all planners" << std::endl;
      return nullptr;
    }
  }while(true);

  // TODO: kill them forcefully (need a stopable thread)
  for(auto& thread: threads)
    if (thread.joinable())
      thread.join();
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mRunning = false;
  }

  return nullptr;

}

//==============================================================================
PlannerPtr ConcreteParallelMetaPlanner::clone(common::RNG* rng) const
{
  throw std::runtime_error("Cloning MetaPlanner is not suported.");
}

//==============================================================================
PlannerPtr ConcreteParallelMetaPlanner::clone(
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      common::RNG* rng) const
{
  throw std::runtime_error("Cloning MetaPlanner is not suported.");
}

//==============================================================================
bool ConcreteParallelMetaPlanner::stopPlanning()
{
  return false;
}

} // namespace dart
} // namespace planner
} // namespace aikido
