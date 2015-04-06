#ifndef _R3_TRAJECTORY_H_
#define _R3_TRAJECTORY_H_

#include "state.h"

namespace r3 
{
  /**
   * A path is an untimed list of states that a system will pass through.
   */
  class Path
  {
    std::vector<State> waypoints;
  };

  /**
   * A trajectory is a function that maps from time to system state.
   * Trajectories can have any internal representation. The only requirement is
   * that they can be sampled at any time over their duration to return a State.
   */
  template <typename State>
  class Trajectory
  {
    State Sample(double time);
    void Duration();
    unsigned int Order();
  };
  
  class RealTrajectory
  : Trajectory<double>
  {};

  class QuaternionTrajectory
  : Trajectory<Quaternion>
  {};
  
  template <typename State>
  class DOFTrajectory
  : std::vector<State>
  {};
}

#endif // _R3_TRAJECTORY_H_
