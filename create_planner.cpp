#include <r3.h>
#include <boost/python> // @TODO: what is the right include for this?

class CustomPlanner : public r3::Planner 
{
  // Let's define a custom property.
  public int count = 0;
  
  // Let's define a custom member function.
  public int increment(int x) {
    count += x;
    return count;
  }
  
  // This is a part of the standard planner interface.
  public r3::Trajectory plan(r3::Environment env, std::vector<int> dof_indices,
                             std::vector<float> start_dofs, std::vector<float> goal_dofs) {
    return r3::Trajectory::EmptyTrajectory;
  }
}

// Expose this class to Boost::Python
class_<CustomPlanner>("CustomPlanner")
        .def("increment", &CustomPlanner::increment)
        ;

// @TODO: What else do we need to do?
