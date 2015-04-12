#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/simulation.h>
#include <dart/utils/urdf/urdf.h>

void python_DartLoader()
{
    using namespace ::boost::python;
    using ::dart::utils::DartLoader;

    class_<DartLoader>("DartLoader")
        .def("parse_skeleton", &DartLoader::parseSkeleton,
             return_value_policy<reference_existing_object>())
        .def("parse_world", &DartLoader::parseWorld,
             return_value_policy<manage_new_object>())
        .def("add_package_directory", &DartLoader::addPackageDirectory)
        ;
}
