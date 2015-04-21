#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>

void python_Skeleton()
{
    using namespace ::boost::python;
    using ::dart::dynamics::DegreeOfFreedom;
    using ::dart::dynamics::Skeleton;

    class_<Skeleton, Skeleton *>("Skeleton")
        .add_property("name",
            make_function(&Skeleton::getName,
                          return_value_policy<copy_const_reference>()),
            &Skeleton::setName)
        .add_property("is_enabled_self_collision_check",
            &Skeleton::isEnabledSelfCollisionCheck)
        .add_property("is_enabled_adjacent_body_check",
            &Skeleton::isEnabledAdjacentBodyCheck)
        .add_property("position", &Skeleton::setPosition, &Skeleton::getPosition)
        .def("get_dof",
            make_function(
                static_cast<DegreeOfFreedom *(Skeleton::*)(size_t)>(
                    &Skeleton::getDof),
                return_value_policy<reference_existing_object>()))
        .def("get_dof_by_name",
            make_function(
                static_cast<DegreeOfFreedom *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getDof),
                return_value_policy<reference_existing_object>()))
        .def("enable_self_collision", &Skeleton::enableSelfCollision)
        .def("disable_self_collision", &Skeleton::disableSelfCollision)
        ;
}
