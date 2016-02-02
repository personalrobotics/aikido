#include <boost/python.hpp>
#include <boost/eigen_numpy.h>

void python_OMPL();

BOOST_PYTHON_MODULE(AIKIDO_MODULE_NAME)
{
    SetupEigenConverters();

    python_OMPL();
}
