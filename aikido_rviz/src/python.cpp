#include <ros/ros.h>
#include <boost/python.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.h>

namespace {

void init_node(const std::string& node_name)
{
  static int argc = 0;
  static char **argv = nullptr;

  ros::init(argc, argv, node_name,
    ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
}

void spin_once()
{
  ros::spinOnce();
}

} // namespace

BOOST_PYTHON_MODULE(PROJECT_NAME)
{
  using namespace boost::python;

  using boost::noncopyable;
  using aikido::rviz::InteractiveMarkerViewer;

  class_<InteractiveMarkerViewer, noncopyable>("InteractiveMarkerViewer",
     init<std::string const &>())
    .def("addSkeleton", &InteractiveMarkerViewer::addSkeleton)
    .def("update", &InteractiveMarkerViewer::update)
    ;

  def("init_node", &init_node);
  def("spin_once", &spin_once);
}
