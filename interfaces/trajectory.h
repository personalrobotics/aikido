class State
{
  /** The dimension of the state space */
  int dim;
  
  /** The order (number of derivatives) specified by the state */
  int order;
  
  /** A 2D matrix of [order][dim], where each row is the n-th order state */
  double[][] values;
}

/**
 * A path is an untimed list of states that a system will pass through.
 */
class Path
{
  std::vector<State> waypoints;
}

/**
 * A trajectory is a function that maps from time to system state.
 * Trajectories can have any internal representation. The only requirement is
 * that they can be sampled at any time over their duration to return a State.
 */
class Trajectory
{
  State Sample(double time);
  void Duration();
}
