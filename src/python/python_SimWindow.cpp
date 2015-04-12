#include <boost/bind.hpp>
#include <boost/python.hpp>
#include <dart/simulation/simulation.h>
#include <dart/gui/gui.h>

using ::dart::gui::SimWindow;
using ::dart::simulation::World;

static SimWindow * SimWindow_constructor()
{
  char **argv = NULL;
  int argc = 0;

  glutInit(&argc, argv);

  return new SimWindow;
}

static World *SimWindow_get_world(SimWindow *window)
{
    throw std::runtime_error("get_world() is not implemented.");
}

static void SimWindow_spin(SimWindow *window)
{
    glutMainLoop();
}

void python_SimWindow()
{
    using namespace ::boost::python;

    class_<SimWindow>("SimWindow", no_init)
        .def("__init__", make_constructor(&SimWindow_constructor))
        .def("init_window", &SimWindow::initWindow)
        .def("spin", &SimWindow_spin)
        .add_property("world", 
            make_function(&SimWindow_get_world,
                          return_value_policy<manage_new_object>()),
            &SimWindow::setWorld)
        ;
}
