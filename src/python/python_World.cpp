#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/World.h>

void python_World()
{
    using namespace ::boost::python;
    using ::dart::simulation::World;

    class_<World>("World")
        .def("add_skeleton", &World::addSkeleton)
        ;
}
