#ifndef _R3_STATE_H_
#define _R3_STATE_H_

namespace r3 
{
  /**
   * A state is a continuous, often-differentiable vector that represents a snapshot of some system.
   * This might be a pose in 3D space, or the joint configuration of a set of links.
   */
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
   * A StateRepresentation is an object that can be configured from a particular type of State.
   */
  class StateProvider<State>
  {
    State get(State s&);
    set(const State &s);
  }
}

#endif // _R3_STATE_H_
