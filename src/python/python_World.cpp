#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/World.h>

void python_World()
{
    using namespace ::boost::python;
    using ::dart::dynamics::Skeleton;
    using ::dart::simulation::World;

    class_<World>("World")
        .def("add_skeleton", &World::addSkeleton)
        .def("get_skeleton",
            make_function(
                static_cast<Skeleton *(World::*)(size_t) const>(
                    &World::getSkeleton),
                return_value_policy<reference_existing_object>()))
        .def("get_skeleton_by_name",
            make_function(
                static_cast<Skeleton *(World::*)(std::string const &) const>(
                    &World::getSkeleton),
                return_value_policy<reference_existing_object>()))
        ;
}
