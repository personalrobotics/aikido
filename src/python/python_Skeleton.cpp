#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>

void python_Skeleton()
{
    using namespace ::boost::python;
    using ::dart::dynamics::Skeleton;

    class_<Skeleton, Skeleton *>("Skeleton")
        .add_property("name",
            make_function(&Skeleton::getName,
                          return_value_policy<copy_const_reference>()),
            &Skeleton::setName)
        .def("set_position", &Skeleton::setPosition)
        .def("get_position", &Skeleton::getPosition)
        ;
}
