#include <ros/ros.h>
#include <boost/python.hpp>
#include <dart_rviz/InteractiveMarkerViewer.h>

void init_node()
{
  static int argc = 0;
  static char **argv = nullptr;

  ros::init(argc, argv, "dart_rviz",
    ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
}

void spin_once()
{
  ros::spinOnce();
}

BOOST_PYTHON_MODULE(PROJECT_NAME)
{
  using namespace boost::python;

  using boost::noncopyable;
  using dart::rviz::InteractiveMarkerViewer;

  class_<InteractiveMarkerViewer, noncopyable>("InteractiveMarkerViewer",
     init<std::string const &>())
    .def("add_skeleton", &InteractiveMarkerViewer::addSkeleton)
    .def("update", &InteractiveMarkerViewer::update)
    ;

  def("init_node", &init_node);
  def("spin_once", &spin_once);
}
