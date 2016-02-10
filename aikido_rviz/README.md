# aikido_rviz
Utility for publishing [KIDO](http://dartsim.github.io/) `Skeleton`s as ROS
[`InteractiveMarker`s](http://wiki.ros.org/interactive_markers) that can be
visualized in [RViz](http://wiki.ros.org/rviz).

## Example

The following code publishes `skeleton` as `InteractiveMarker`s on the
`my_marker_topic` ROS topic:
```c++
#include <aikido/rviz/InteractiveMarkerViewer.h>

// ...

ros::init_node(argc, argv, "my_node_name");

aikido::rviz::InteractiveMarkerViewer viewer("my_marker_topic");
viewer.addSkeleton(skeleton);
viewer.update();
```

You can view the `Skeleton` in RViz by adding an `Interactive Marker` display
configured to listen to the `my_marker_topic/update` topic.

## License

`aikido_rviz` is currently unreleased.

## Contributors

`aikido_rviz` was developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the
[Robotics Institute](https://www.ri.cmu.edu) at
[Carnegie Mellon University](http://www.cmu.edu). This library is developed and
maintained by
[Michael Koval](https://mkoval.org)
([**@mkoval**](https://github.com/mkoval)).
