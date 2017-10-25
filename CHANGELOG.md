## Aikido 0 (prerelease)

### 0.2.0 (201X-XX-XX)

* Constraint

  * Added methods for removing groups from NonColliding constraints. [#247](https://github.com/personalrobotics/aikido/pull/247)

* Planner

  * Added World class: [#243](https://github.com/personalrobotics/aikido/pull/243)

* RViz

  * Added WorldInteractiveMarkerViewer: [#242](https://github.com/personalrobotics/aikido/pull/242)

* Build & Testing & ETC

  * Changed to use size_t over std::size_t: [#230](https://github.com/personalrobotics/aikido/pull/230)
  * Included test code to formatting code list: [#239](https://github.com/personalrobotics/aikido/pull/239)

### [0.1.0 (2017-10-02)](https://github.com/personalrobotics/aikido/milestone/4?closed=1)

* Common

  * Added VanDerCorput::getLength(): [#223](https://github.com/personalrobotics/aikido/pull/223)
  * Added ExecutorThread and ExecutorMultiplexer: [#151](https://github.com/personalrobotics/aikido/pull/151)
  * Changed splitEngine() to set default numOutputs to 1: [#214](https://github.com/personalrobotics/aikido/pull/214)

* State Space

  * Added MetaSkeletonStateSpaceSaver: [#184](https://github.com/personalrobotics/aikido/pull/184), [#219](https://github.com/personalrobotics/aikido/pull/219)
  * Added WeldJoint: [#146](https://github.com/personalrobotics/aikido/pull/146)
  * Improved real vector space classes to take advantage of Eigen's fixed-size objects: [#159](https://github.com/personalrobotics/aikido/pull/159)

* Constraint

  * Added SE2BoxConstraint: [#135](https://github.com/personalrobotics/aikido/pull/135)
  * Added satisfiable tolerance parameter to TSR constructor: [#180](https://github.com/personalrobotics/aikido/pull/180)
  * Added ConstantSampler: [#146](https://github.com/personalrobotics/aikido/pull/146)
  * Renamed SO2Sampleable to SO2UniformSampler: [#224](https://github.com/personalrobotics/aikido/pull/224)

* Perception

  * Added simulation world env to ApriltagsModule: [#156](https://github.com/personalrobotics/aikido/pull/156)

* Planner

  * Added parabolic smoother: [#206](https://github.com/personalrobotics/aikido/pull/206)
  * Added wrapper for OMPL path simplifier: [#164](https://github.com/personalrobotics/aikido/pull/164)
  * Added support OMPL 1.2.0 and above: [#139](https://github.com/personalrobotics/aikido/pull/139)
  * Changed to SnapePlanner to return nullptr when planning fails: [#201](https://github.com/personalrobotics/aikido/pull/201)
  * Refactored parabolic timer: [#206](https://github.com/personalrobotics/aikido/pull/206)

* IO

  * Added KinBody parser: [#102](https://github.com/personalrobotics/aikido/pull/102)
  * Improved yaml extension: [#175](https://github.com/personalrobotics/aikido/pull/175), [#200](https://github.com/personalrobotics/aikido/pull/200)

* RViz

  * Added TSR visualizer: [#136](https://github.com/personalrobotics/aikido/pull/136)
  * Made FrameId a variable: [#183](https://github.com/personalrobotics/aikido/pull/183)
  * Fixed inconsistent function signature: [#222](https://github.com/personalrobotics/aikido/pull/222)

* Control

  * Added toSplineJointTrajectory() that can controls only specifying joints: [#217](https://github.com/personalrobotics/aikido/pull/217)
  * Added RosPositionCommandExecutor: [#173](https://github.com/personalrobotics/aikido/pull/173)
  * Added BarretthandPositionCommandExecutor: [#166](https://github.com/personalrobotics/aikido/pull/166)
  * Added RosJointStateClient: [#150](https://github.com/personalrobotics/aikido/pull/150)
  * Added trajectory conversions between Aikido's and ROS's: [#147](https://github.com/personalrobotics/aikido/pull/147), [#149](https://github.com/personalrobotics/aikido/pull/149)
  * Improved RosTrajectoryExecutor to use helper function for waiting for action server: [#192](https://github.com/personalrobotics/aikido/pull/192)
  * Improved (~)Executor::step() not to lock: [#182](https://github.com/personalrobotics/aikido/pull/182)
  * Fixed renaming sping to step: [#189](https://github.com/personalrobotics/aikido/pull/189)
  * Fixed to work on real hardware: [#216](https://github.com/personalrobotics/aikido/pull/216)
  * Removed StateSpace from RosTrajectoryExecutor: [#190](https://github.com/personalrobotics/aikido/pull/190)

* Build & Testing & ETC

  * Added Findinteractive_markers.cmake: [#198](https://github.com/personalrobotics/aikido/pull/198)
  * Improved to build without warnings: [#114](https://github.com/personalrobotics/aikido/pull/114), [#143](https://github.com/personalrobotics/aikido/pull/143)
  * Improved to enforce to follow code formatting: [#153](https://github.com/personalrobotics/aikido/pull/153), [#167](https://github.com/personalrobotics/aikido/pull/167), [#172](https://github.com/personalrobotics/aikido/pull/172), [#168](https://github.com/personalrobotics/aikido/pull/168), [#170](https://github.com/personalrobotics/aikido/pull/170), [#171](https://github.com/personalrobotics/aikido/pull/171), [#191](https://github.com/personalrobotics/aikido/pull/191), [#207](https://github.com/personalrobotics/aikido/pull/207), [#211](https://github.com/personalrobotics/aikido/pull/211)
  * Switched to codecov for online code coverage reporting: [#208](https://github.com/personalrobotics/aikido/pull/208)
  * Increased the required minimum version of DART to 6.2: [#210](https://github.com/personalrobotics/aikido/pull/210)
  * Changed to test Aikido for both of release and debug modes, and on macOS: [#195](https://github.com/personalrobotics/aikido/pull/195)
  * Splitted util namespace into common and io: [#225](https://github.com/personalrobotics/aikido/pull/225)
  * Fixed build issues on macOS: [#138](https://github.com/personalrobotics/aikido/pull/138)

### 0.0.1 (2017-03-10)

  * Initial release

