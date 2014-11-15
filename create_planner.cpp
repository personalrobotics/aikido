#include <r3.h>

class CustomPlanner : public r3::Planner 
{
  // Let's define a custom property.
  public int count = 0;
  
  // Let's define a custom member function.
  public int increment(int x) {
    count += x;
    return count;
  }
}

class_<CustomPlanner>("CustomPlanner")
        .def("increment", &CustomPlanner::increment)
        ;

// What else do we need to do?
