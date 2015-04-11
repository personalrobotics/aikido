r3 - Robot Representation Resources
===================================

Let's put together a list of things we would want in a robot representation system, and see if we can work out how to make it happen!  This is not necessarily an implementation of anything, just a place to put notes and pseudocode.

## Things we definitely want ##
* Explicit DOF-value representation for forward kinematics (not backing it out of transforms).
* C++ and Python support
* Revolute, Prismatic, and Affine DOFs
* Fast queries for forward-/inverse-kinematics, Jacobians

## OpenRAVE quirks

* build/packaging system
* string parsing for plugins
* unstable dynamics simulators
* complicated trajectory representation / ConfigurationSpecification
* inability to extend python objects for subclasses
* inability to store metadata in an extensible way
* private scope on certain interfaces
* complicated extension for custom loaders
* multiple joint types that don't seem to be useful

## Klampt quirks

* No plugin system
* API doesn't see to match our codebase(?)

## MoveIt quirks

* Crashes all the time
* Cannot handle multiple independent kinematics chains (robot + cabinet)

