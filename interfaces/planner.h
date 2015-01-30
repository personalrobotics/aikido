#ifndef _R3_PLANNER_H_
#define _R3_PLANNER_H_

#include "trajectory.h"
#include <future>

namespace r3 
{
  /**
   * A path planner is a function that takes some set of parameters and outputs an untimed path.
   */
  class PathPlanner<T>
  {
    std::future<Path> Plan(const T *params);
  }
  
  /**
   * A motion planner is a function that takes some set of parameters and outputs a timed trajectory.
   */
  class MotionPlanner<T>
  {
    std::future<Trajectory> Plan(const T *params);
  }
  
  /** 
   * An optimizer is a function that takes an existing timed trajectory and some parameters and returns
   * a new timed trajectory that is 'better' with respect to the parameters.
   */
  class Optimizer<T>
  {
    std::future<Trajectory> Optimize(Trajectory t, const T &params);
  }
}

#endif // _R3_PLANNER_H_
