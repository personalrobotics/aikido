#include <boost/python.hpp>
#include <r3/Skeleton.h>

void python_Skeleton()
{
    using namespace ::boost::python;
    using ::r3::Skeleton;

    class_<Skeleton>("Skeleton")
        ;
}
