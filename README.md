# AIKIDO - AI for KIDO [![Build Status](https://travis-ci.org/personalrobotics/aikido.svg?branch=master)](https://travis-ci.org/personalrobotics/aikido) [![Coverage Status](https://coveralls.io/repos/github/personalrobotics/aikido/badge.svg?branch=master)](https://coveralls.io/github/personalrobotics/aikido?branch=master)

> :warning: **Warning:** AIKIDO is under heavy development. These instructions are
> primarily for reference by the developers.

AIKIDO is a C++ library, complete with Python bindings, for solving robotic motion
planning and decision making problems. This library is tightly integrated with
[DART] for kinematic/dynamics calculations and [OMPL] for motion planning. AIKIDO
optionally integrates with [ROS], through the suite of `aikido_ros` packages, for
execution on real robots.

### Dependencies
AIKIDO depends on [CMake], [Boost], [DART] \(version 6.1 or above), [OMPL], and the
Python development headers (`python-dev` on Debian systems). [DART] and AIKIDO both
make heavy use of C++11 and require a modern compiler.

### Installation

#### Install on macOS using [Homebrew]
```shell
# install Homebrew package manager
$ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
# add Homebrew tap for Personal Robotics Lab software
$ brew tap personalrobotics/tap
# install Aikido
$ brew install aikido0
```

#### Build from source (Standalone)
To build ROS-dependent AIKIDO components, necessary environment variables need to be set. The default would be sourcing ROS's `setup.bash` as
```bash
$ source /opt/ros/indigo/setup.bash
```
If you want to build AIKIDO upon your newer version of ROS packages, then source `setup.bash` in your workspace as
```bash
$ source /path/to/your/workspace/devel/setup.bash
```

Once the dependencies are installed, you can build AIKIDO using [CMake]:
```shell
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

#### Build from source (Catkin)
It is also possible to build AIKIDO as a [third-party package][REP-136] inside a
[Catkin workspace][Catkin Workspaces]. To do so, clone AIKIDO into your Catkin
workspace and use the `catkin build` command like normal.

If you are using the older `catkin_make` command, then you must build your workspace
with `catkin_make_isolated`. This may dramatically increase your build time, so we
*strongly recommend* that you use `catkin build`, which is provided by the
[`catkin_tools` package][Catkin Tools], if possible.

### Code Style
Please follow the [Aikido code style](https://github.com/personalrobotics/aikido/blob/master/STYLE.md) when you making a contribution.

### License
Aikido is licensed under a BSD license. See [LICENSE](./LICENSE) for more
information.

### Authors
Aikido is developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the
[Robotics Institute](http://ri.cmu.edu/) at
[Carnegie Mellon University](http://www.cmu.edu/). The library was started by 
Michael Koval ([**@mkoval**](https://github.com/mkoval))
and Pras Velagapudi ([**@psigen**](https://github.com/psigen)). It has received
major contributions from
Shushman Choudhury ([**@Shushman**](https://github.com/Shushman)),
Aaron Johnson ([**@aaronjoh**](https://github.com/aaronjoh)),
Jennifer King ([**@jeking**](https://github.com/jeking04)),
Gilwoo Lee ([**@lgw903**](https://github.com/lgw903)),
and Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)). We
also would like to thank
Michael Grey ([**@mxgrey**](https://github.com/mxgrey))
and J.S. Lee ([**@jslee02**](https://github.com/jslee02))
for making changes to DART to better support Aikido.


[DART]: http://dartsim.github.io/
[OMPL]: http://ompl.kavrakilab.org/
[ROS]: http://ros.org/
[CMake]: http://www.cmake.org/
[Boost]: http://www.boost.org/
[REP-136]: http://www.ros.org/reps/rep-0136.html
[Catkin Workspaces]: http://wiki.ros.org/catkin/workspaces
[Catkin Tools]: http://catkin-tools.readthedocs.org/en/latest/
[Homebrew]: https://brew.sh/
