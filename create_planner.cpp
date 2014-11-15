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
}

// Expose this class to Boost::Python
class_<CustomPlanner>("CustomPlanner")
        .def("increment", &CustomPlanner::increment)
        ;

// @TODO: What else do we need to do?
