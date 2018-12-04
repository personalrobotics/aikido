#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"
#include "aikido/planner/dart/DartProblem.hpp"

#include <thread>
#include <future>

namespace aikido {
namespace planner {

// Planning call for individual planner.
static void _plan(
      const PlannerPtr& planner,
      const std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>& promise,
      const ProblemPtr& problem,
      const std::shared_ptr<Planner::Result>& result)
{
  trajectory::TrajectoryPtr trajectory = planner->plan(
      *problem.get(),
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
: ParallelMetaPlanner(std::move(stateSpace),
    std::vector<PlannerPtr>()),
  mMetaSkeleton(metaSkeleton)
{
  int numPlanners = planners.size();
  mPlanners.reserve(numPlanners);

  for(const auto& planner: planners)
  {
    mPlanners.emplace_back(planner->clone());
    //TODO
    //mMetaSkeletons.emplace_back(metaSkeleton->clone());
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

  auto dartProblem = dynamic_cast<DartProblem const*>(&problem);
  if (!dartProblem)
  {
    throw std::runtime_error("Problem is not DartProblem");
  }

  std::vector<std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>>
    promises;
  std::vector<std::thread> threads;
  std::vector<ProblemPtr> clonedProblems;
  std::vector<std::shared_ptr<Result>> results;

  results.reserve(mPlanners.size());

  for (const auto& planner : mPlanners)
  {
    // TODO: this should use mMetaSkeleton->clone()
    auto clonedProblem = dartProblem->clone(mMetaSkeleton);
    auto result = std::make_shared<Result>();

    clonedProblems.push_back(clonedProblem);
    results.push_back(result);
    auto promise = std::make_shared<std::promise<trajectory::TrajectoryPtr>>();

    auto thread = std::thread(_plan, planner,
        promise, clonedProblem, result);
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

/*==============================================================================
void ConcreteParallelMetaPlanner::_plan(
      const PlannerPtr& planner,
      const std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>& promise,
      const ProblemPtr& problem, const std::shared_ptr<Result>& result
    )
{
  return;
}
*/

} // namespace planner
} // namespace aikido
