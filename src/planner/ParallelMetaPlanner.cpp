#include "aikido/planner/ParallelMetaPlanner.hpp"
#include <thread>
#include <future>

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr ParallelMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mRunning)
    {
      throw std::runtime_error("Currently planning another problem");
    }
  }

  std::vector<std::promise<trajectory::TrajectoryPtr>> promises;
  std::vector<std::thread> threads;
  std::vector<Problem> clonedProblems;
  std::vector<Result> clonedResults;
  for (const auto& planner : mPlanners)
  {
    auto clonedProblem = problem->clone();
    auto clonedResult = result->clone();
    auto promise = std::promise<trajectory::TrajectoryPtr>();

    clonedProblems.push_back(clonedProblem);
    clonedResults.push_back(clonedResult);
    promises.push_abck(promise);

    std::thread thread(_plan, promise, planner, clonedProblem, clonedResult);
    threads.push_back(thread);
  }

  do
  {
    int num_failures = 0;
    for(int i = 0; i < mPlanners.size(); ++i)
    {
      auto future = promises[i].get_future();
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

}

//==============================================================================
void ParallelMetaPlanner::_plan(
    const Planner& planner, std::promise<trajectory::TrajectoryPtr> promise,
    const Problem& problem, Result* result=nullptr,
    )
{
    auto trajectory = planner->plan(problem, result);
    if (trajectory)
      promise.set_value(trajectory);

    promise.set_value(nullptr):

  return;
}


} // namespace planner
} // namespace aikido
