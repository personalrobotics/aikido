#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"
#include "aikido/planner/dart/DartProblem.hpp"
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
  trajectory::TrajectoryPtr trajectory = planner->plan(
      *problem,
      result.get());
  if (trajectory)
    promise->set_value(trajectory);

  promise->set_value(nullptr);
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    const std::vector<PlannerPtr>& planners)
: ParallelMetaPlanner(std::move(stateSpace), planners),
  mMetaSkeleton(metaSkeleton)
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
: ParallelMetaPlanner(std::move(stateSpace)),
  mMetaSkeleton(metaSkeleton)
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
  using aikido::planner::DartProblem;

  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mRunning)
    {
      throw std::runtime_error("Currently planning another problem");
    }

    mRunning = true;
  }

  std::vector<std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>>
    promises;
  std::vector<std::thread> threads;
  std::vector<ProblemPtr> clonedProblems;
  std::vector<std::shared_ptr<Result>> results;

  // Problem is not cloned but just shared.
  auto shared_problem = std::shared_ptr<const Problem>(&problem);

  results.reserve(mPlanners.size());

  for (const auto& planner : mPlanners)
  {
    auto result = std::make_shared<Result>();

    results.push_back(result);
    auto promise = std::make_shared<std::promise<trajectory::TrajectoryPtr>>();

    auto thread = std::thread(_plan, planner,
        promise, shared_problem, result);
    promises.push_back(promise);
  }

  do
  {
    std::size_t num_failures = 0;
    for(std::size_t i = 0; i < mPlanners.size(); ++i)
    {
      auto future = promises[i]->get_future();
      auto status = future.wait_for(std::chrono::milliseconds(1));
      if (status == std::future_status::ready)
      {
        auto trajectory = future.get();
        if (trajectory != nullptr)
        {
          // TODO:: copy result
          return trajectory;
        }
        else
          num_failures++;
      }
    }
    if (num_failures == mPlanners.size())
      return nullptr;
  }while(true);

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
