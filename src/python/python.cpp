#include <boost/python.hpp>
#include <boost/eigen_numpy.h>

void python_DartLoader();
void python_DegreeOfFreedom();
void python_SimWindow();
void python_Skeleton();
void python_World();

BOOST_PYTHON_MODULE(R3_MODULE_NAME)
{
    SetupEigenConverters();

    python_DartLoader();
    python_DegreeOfFreedom();
    python_SimWindow();
    python_Skeleton();
    python_World();
}
