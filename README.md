# AIKIDO - AI for KIDO

> :warning: **Warning:** AIKIDO is under heavy development. These instructions are
> primarily for reference by the developers.

AIKIDO is a C++ library, complete with Python bindings, for solving robotic motion
planning and decision making problems. This library is tightly integrated with
[DART](http://dartsim.github.io/) for kinematic/dynamics calculations and
[OMPL](http://ompl.kavrakilab.org/) for motion planning. AIKIDO optionally
integrates with [ROS](http://ros.org/), through the suite of `aikido_ros` packages,
for execution on a real robot.

### Dependencies
AIKIDO depends on [CMake](http://www.cmake.org/), [Boost](http://www.boost.org/),
[DART](http://dartsim.github.io/) (version 5.0 or above),
[OMPL](http://ompl.kavrakilab.org/), and the Python development headers
(`python-dev` on Debian systems). DART and AIKIDO both make heavy use of C++11 and
require a modern compiler.

### Installation (Standalone)
Once the dependencies are installed, you can build AIKIDO using CMake:
```shell
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

### Installation (Catkin)
It is also possible to build AIKIDO as a
[third-party package](http://www.ros.org/reps/rep-0136.html) inside a
[Catkin workspace](http://wiki.ros.org/catkin/workspaces). To do so, clone AIKIDO
into your Catkin workspace and use the `catkin build` command like normal.

If you are using the older `catkin_make` command, then
you must build your workspace with `catkin_make_isolated`. This may
dramatically increase your build time, so we *strongly recommend* that you use
`catkin build`, which is provided by the [`catkin_tools`
package](http://catkin-tools.readthedocs.org/en/latest/), if possible.

### License
AIKIDO is licensed under a BSD license. See [LICENSE](./LICENSE) for more information.

### Authors
AIKIDO was developed by Michael Koval ([**@mkoval**](https://github.com/mkoval))
and Pras Velagapudi ([**@psigen**](https://github.com/psigen)) in the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the
[Robotics Institute](http://ri.cmu.edu/) at
[Carnegie Mellon University](http://www.cmu.edu/).
