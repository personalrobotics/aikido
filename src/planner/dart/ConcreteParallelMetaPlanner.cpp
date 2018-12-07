#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"
#include "aikido/planner/dart/util.hpp"

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
    const std::vector<PlannerPtr>& planners)
: ParallelMetaPlanner(std::move(stateSpace), planners)
, mMetaSkeleton(metaSkeleton)
, mRunning(false)
{
  // Do nothing
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      const PlannerPtr& planner,
      std::size_t numCopies,
      const std::vector<common::RNG*> rngs)
: ParallelMetaPlanner(std::move(stateSpace))
, mMetaSkeleton(metaSkeleton)
, mRunning(false)
{
  if (rngs.size() > 0 && numCopies != rngs.size())
  {
    std::stringstream ss;
    ss << "numCopies [" << numCopies << "] does not match number of RNGs [ "
      << rngs.size() << "]."  << std::endl;
    throw std::invalid_argument(ss.str());
  }

  mPlanners.reserve(numCopies);
  for (std::size_t i = 0; i < numCopies; ++i)
  {
    std::cout << "Cloning " << i << "th planner"  << std::endl;
    if (rngs.size() == numCopies)
      mPlanners.emplace_back(planner->clone(rngs[i]));
    else
      mPlanners.emplace_back(planner->clone());
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

  auto shared_problem = problem.clone();

  results.reserve(mPlanners.size());

  for (const auto& planner : mPlanners)
  {
    auto result = std::make_shared<Result>();

    results.push_back(result);
    auto promise = std::make_shared<std::promise<trajectory::TrajectoryPtr>>();

    std::cout << "Construct thread" << std::endl;
    threads.push_back(std::thread(_plan, planner, promise, shared_problem, result));
    futures.push_back(promise->get_future());
  }

  std::vector<bool> future_retrieved(mPlanners.size(), false);
  do
  {
    std::size_t num_failures = 0;
    for(std::size_t i = 0; i < mPlanners.size(); ++i)
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

} // namespace dart
} // namespace planner
} // namespace aikido
