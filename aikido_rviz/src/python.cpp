#include <ros/ros.h>
#include <boost/python.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/rviz/SkeletonMarker.hpp>

using aikido::rviz::InteractiveMarkerViewer;

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

void InteractiveMarkerViewer_addSkeleton(
  InteractiveMarkerViewer* viewer, const dart::dynamics::SkeletonPtr& skeleton)
{
  viewer->addSkeleton(skeleton);
}

} // namespace

BOOST_PYTHON_MODULE(PROJECT_NAME)
{
  using namespace boost::python;

  using boost::noncopyable;

  boost::python::import("dartpy");

  class_<InteractiveMarkerViewer, noncopyable>("InteractiveMarkerViewer",
     init<std::string const &>())
    // TODO: This wrapper is necessary to strip the return value, which is a
    // std::shared_ptr, from addSkeleton. Boost.Python on Ubuntu 14.04 does not
    // have native support for std::shared_ptr. We have a workaround for this
    // in dartpy, but it is not easily available here.
    .def("addSkeleton", &InteractiveMarkerViewer_addSkeleton)
    .def("update", &InteractiveMarkerViewer::update)
    ;

  def("init_node", &init_node);
  def("spin_once", &spin_once);
}
