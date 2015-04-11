#include <boost/python.hpp>
#include <r3/Skeleton.h>

void python_Skeleton()
{
    using ::boost::python::class_;
    using ::r3::Skeleton;

    class_<Skeleton>("Skeleton")
        .add_property("name", &Skeleton::name)
        ;
}
