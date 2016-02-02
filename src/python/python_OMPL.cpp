#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/World.h>
#include <aikido/ompl/plan.h>

void python_OMPL()
{
    using namespace ::boost::python;
    using ::aikido::ompl::Plan;

    def("ompl_plan", &Plan);
}
