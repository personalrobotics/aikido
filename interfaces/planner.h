#ifndef _R3_PLANNER_H_
#define _R3_PLANNER_H_

#include <r3.h>
#include "trajectory.h"

namespace r3 
{
  /**
   * A path planner is a function that takes some set of parameters and outputs an untimed path.
   */
  class PathPlanner<T>
  {
    Path Plan(const T *params);
  }
  
  /**
   * A motion planner is a function that takes some set of parameters and outputs a timed trajectory.
   */
  class MotionPlanner<T>
  {
    Trajectory Plan(const T *params);
  }
  
  /** 
   * An optimizer is a function that takes an existing timed trajectory and some parameters and returns
   * a new timed trajectory that is 'better' with respect to the parameters.
   */
  class Optimizer<T>
  {
    Trajectory Optimize(Trajectory t, const T &params);
  }
}

#endif // _R3_PLANNER_H_
