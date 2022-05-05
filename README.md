# AIKIDO - AI for KIDO [![Build Status](https://github.com/personalrobotics/aikido/actions/workflows/build-test.yml/badge.svg?branch=master)](https://github.com/personalrobotics/aikido/actions) [![codecov](https://codecov.io/gh/personalrobotics/aikido/branch/master/graph/badge.svg)](https://codecov.io/gh/personalrobotics/aikido) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/6d2da97f57904c96a01a5993cbbc4f51)](https://www.codacy.com/app/personalrobotics/aikido?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=personalrobotics/aikido&amp;utm_campaign=Badge_Grade)[![DOI](https://zenodo.org/badge/34424077.svg)](https://zenodo.org/badge/latestdoi/34424077)


> :warning: **Warning:** AIKIDO is under heavy development. These instructions are
> primarily for reference by the developers.

AIKIDO is a C++ library, complete with Python bindings, for solving robotic motion
planning and decision making problems. This library is tightly integrated with
[DART] for kinematic/dynamics calculations and [OMPL] for motion planning. AIKIDO
optionally integrates with [ROS], through the suite of `aikido_ros` packages, for
execution on real robots.


<!--- TODO: Re-add Aikido to PPA
## Installation

### On Ubuntu Focal using `apt`

AIKIDO depends on [ROS]. You should [install ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) by adding the ROS repository to your `sources.list` as follows. We encourage users to install [`noetic`](http://wiki.ros.org/noetic) with at least the following packages:
```shell
$ sudo apt install ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-interactive-markers ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-tf ros-noetic-trajectory-msgs ros-noetic-visualization-msgs
```


Once ROS is installed, you can install AIKIDO from the [Personal Robotics Lab PPA](https://launchpad.net/~personalrobotics/+archive/ubuntu/ppa):
```shell
$ sudo add-apt-repository ppa:libccd-debs/ppa
$ sudo add-apt-repository ppa:fcl-debs/ppa
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update
$ sudo apt-get install libaikido-all-dev
```
-->

<!--- TODO: Test on OS X and re-add to Homebrew

### On macOS using [Homebrew]

```shell
# Install the Homebrew package manager
$ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
# Add Homebrew tap for Personal Robotics Lab software
$ brew tap personalrobotics/tap
# Install AIKIDO
$ brew install aikido
```
> Note: While ROS seems to be [available on macOS](http://wiki.ros.org/noetic/Installation/OSX/Homebrew/Source), we haven't tested it with AIKIDO. For now, `brew install aikido` installs AIKIDO without the ROS-dependent components.
-->


## Building from Source

### Dependencies

AIKIDO depends on [ROS]. You should [install ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) by adding the ROS repository to your `sources.list` as follows. We encourage users to install [`noetic`](http://wiki.ros.org/noetic) with at least the following packages:
```shell
$ sudo apt install ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-interactive-markers ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-tf ros-noetic-trajectory-msgs ros-noetic-visualization-msgs
```

AIKIDO also depends on [CMake], [Boost], [DART] \(version 6.8.5 or above), [OMPL], [yaml-cpp](https://github.com/jbeder/yaml-cpp), tinyxml2, pr-control-msgs, libmicrohttpd, and the
Python development headers (`python-dev` on Debian systems). [DART] and AIKIDO both
make heavy use of C++14 and require a modern compiler.

### On Ubuntu Focal using CMake

You should install the ROS packages as described above to build all the ROS-dependent AIKIDO components (e.g., `aikido-control-ros`).

Install the other dependencies:
```shell
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update
$ sudo apt-get install cmake build-essential libboost-filesystem-dev libdart6-optimizer-nlopt-dev libdart6-utils-dev libdart6-utils-urdf-dev libmicrohttpd-dev libompl-dev libtinyxml2-dev libyaml-cpp-dev pr-control-msgs
```

Once the dependencies are installed, you can build and install AIKIDO using [CMake]:
```shell
$ mkdir build
$ cd build
$ cmake ..
$ make  # you may want to build AIKIDO using multi-core by executing `make -j4`
$ sudo make install
```

AIKIDO includes several optional components that depend on [ROS]. While we
suggest building AIKIDO in a Catkin workspace (see below) to enable the ROS
components, it is also possible to build those components in a standalone
build. To do so, source the `setup.bash` file in your Catkin workspace before
running the above commands, e.g.:
```shell
$ . /path/to/my/workspace/setup.bash
```

### On Ubuntu Focal using Catkin

It is also possible to build AIKIDO as a [third-party package][REP-136] inside a
[Catkin workspace][Catkin Workspaces]. To do so, clone AIKIDO into your Catkin
workspace and use the `catkin build` command like normal.

If you are using the older `catkin_make` command, then you must build your workspace
with `catkin_make_isolated`. This may dramatically increase your build time, so we
*strongly recommend* that you use `catkin build`, which is provided by the
[`catkin_tools` package][Catkin Tools], if possible.

### On macOS using CMake

Please install [Homebrew] as described above, then you can easily install the dependencies as follows:
```shell
$ cd <aikido_directory>
$ brew bundle
```

Once the dependencies are installed, you can build and install AIKIDO using [CMake]:
```shell
$ cd <aikido_directory>
$ mkdir build
$ cd build
$ cmake ..
$ make  # you may want to build AIKIDO using multi-core by executing `make -j4`
$ sudo make install
```

## Code Style

Please follow the [AIKIDO style guidelines](https://github.com/personalrobotics/aikido/blob/master/STYLE.md) when making a contribution.

## License

AIKIDO is licensed under a BSD license. See [LICENSE](./LICENSE) for more
information.

## Authors

AIKIDO is developed by the
[Personal Robotics Lab](https://personalrobotics.cs.washington.edu/) in the
[Paul G. Allen School of Computer Science and Engineering](https://www.cs.washington.edu/) at
the [University of Washington](https://www.washington.edu/).
The library was started by
Michael Koval ([**@mkoval**](https://github.com/mkoval))
and Pras Velagapudi ([**@psigen**](https://github.com/psigen)).
It has received major contributions from
Shushman Choudhury ([**@Shushman**](https://github.com/Shushman)),
Ethan Gordon ([**@egordon**](https://github.com/egordon)),
Brian Hou ([**@brianhou**](https://github.com/brianhou)),
Aaron Johnson ([**@aaronjoh**](https://github.com/aaronjoh)),
Jennifer King ([**@jeking**](https://github.com/jeking04)),
Gilwoo Lee ([**@gilwoolee**](https://github.com/gilwoolee)),
Jeongseok Lee ([**@jslee02**](https://github.com/jslee02)),
and Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick)).
We also would like to thank
Michael Grey ([**@mxgrey**](https://github.com/mxgrey))
and Jeongseok Lee ([**@jslee02**](https://github.com/jslee02))
for making changes to DART to better support AIKIDO.


[DART]: http://dartsim.github.io/
[OMPL]: http://ompl.kavrakilab.org/
[ROS]: http://ros.org/
[CMake]: http://www.cmake.org/
[Boost]: http://www.boost.org/
[REP-136]: http://www.ros.org/reps/rep-0136.html
[Catkin Workspaces]: http://wiki.ros.org/catkin/workspaces
[Catkin Tools]: http://catkin-tools.readthedocs.org/en/latest/
[Homebrew]: https://brew.sh/
