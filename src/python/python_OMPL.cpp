#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/World.h>
#include <r3/ompl/plan.h>

void python_OMPL()
{
    using namespace ::boost::python;
    using ::r3::ompl::Plan;

    def("ompl_plan", &Plan);
}
