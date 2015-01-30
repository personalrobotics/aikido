#ifndef _R3_ENVIRONMENT_H_
#define _R3_ENVIRONMENT_H_

namespace r3 
{
  /**
   * An environment represents a system that can be projected in some way into a continuous state vector.
   *
   * This mapping may be arbitrarily complex and large, and is generally factorizable into nearly independent
   * component states.  Thus, an environment will typically expose some interfaces for representing 
   * sematic subspaces of its full joint state.
   */
  class Environment
  {
    std::future<Path> Plan(const T *params);
  }
  
  class Environment3D
  : public StateProvider<State>
  {
    
  }
}

#endif // _R3_ENVIRONMENT_H_
