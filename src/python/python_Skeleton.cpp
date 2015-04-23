#include <boost/python.hpp>
#include <dart/dynamics/dynamics.h>
#include "util.h"

using ::dart::dynamics::Skeleton;

static Eigen::Isometry3d Skeleton_getPose(Skeleton *skeleton)
{
    //FreeJoint::convertToTransform(
}

void python_Skeleton()
{
    using namespace ::boost::python;
    using ::dart::dynamics::BodyNode;
    using ::dart::dynamics::DegreeOfFreedom;
    using ::dart::dynamics::Joint;
    using ::dart::dynamics::Marker;
    using ::dart::dynamics::SoftBodyNode;
    using ::r3::python::util::collection_from_python;

    collection_from_python<std::vector<Skeleton *> >();

    class_<Skeleton, Skeleton *>("Skeleton")
        .add_property("name",
            make_function(&Skeleton::getName,
                          return_value_policy<copy_const_reference>()),
            &Skeleton::setName)
        .add_property("is_enabled_self_collision_check",
            &Skeleton::isEnabledSelfCollisionCheck)
        .add_property("is_enabled_adjacent_body_check",
            &Skeleton::isEnabledAdjacentBodyCheck)
        .add_property("position", &Skeleton::getPositions, &Skeleton::setPositions)
        .add_property("is_mobile", &Skeleton::isMobile, &Skeleton::setMobile)
        .add_property("timestep", &Skeleton::getTimeStep, &Skeleton::setTimeStep)
        .add_property("gravity",
            make_function(&Skeleton::getGravity,
                          return_value_policy<copy_const_reference>()),
            &Skeleton::setGravity)
        .add_property("mass", &Skeleton::getMass)
        .add_property("num_body_nodes", &Skeleton::getNumBodyNodes)
        .add_property("num_rigid_body_nodes", &Skeleton::getNumRigidBodyNodes)
        .add_property("num_soft_body_nodes", &Skeleton::getNumSoftBodyNodes)
        .add_property("root_body_node",
            make_function(
                static_cast<BodyNode *(Skeleton::*)()>(
                    &Skeleton::getRootBodyNode),
                return_value_policy<reference_existing_object>()))
        .add_property("num_dofs", &Skeleton::getNumDofs)
        .def("get_marker_by_name",
            make_function(
                static_cast<Marker *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getMarker),
                return_value_policy<reference_existing_object>()))
        .def("get_joint",
            make_function(
                static_cast<Joint *(Skeleton::*)(size_t)>(
                    &Skeleton::getJoint),
                return_value_policy<reference_existing_object>()))
        .def("get_joint_by_name",
            make_function(
                static_cast<Joint *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getJoint),
                return_value_policy<reference_existing_object>()))
        .def("get_dof",
            make_function(
                static_cast<DegreeOfFreedom *(Skeleton::*)(size_t)>(
                    &Skeleton::getDof),
                return_value_policy<reference_existing_object>()))
        .def("get_dof_by_name",
            make_function(
                static_cast<DegreeOfFreedom *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getDof),
                return_value_policy<reference_existing_object>()))
        .def("get_body_node",
            make_function(
                static_cast<BodyNode *(Skeleton::*)(size_t)>(
                    &Skeleton::getBodyNode),
                return_value_policy<reference_existing_object>()))
        .def("get_body_node_by_name",
            make_function(
                static_cast<BodyNode *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getBodyNode),
                return_value_policy<reference_existing_object>()))
        .def("get_soft_body_node",
            make_function(
                static_cast<BodyNode *(Skeleton::*)(size_t)>(
                    &Skeleton::getBodyNode),
                return_value_policy<reference_existing_object>()))
        .def("get_soft_body_node_by_name",
            make_function(
                static_cast<SoftBodyNode *(Skeleton::*)(std::string const &)>(
                    &Skeleton::getSoftBodyNode),
                return_value_policy<reference_existing_object>()))
        .def("enable_self_collision", &Skeleton::enableSelfCollision)
        .def("disable_self_collision", &Skeleton::disableSelfCollision)
        .def("compute_forward_kinematics", &Skeleton::computeForwardKinematics)
        .def("add_body_node", &Skeleton::addBodyNode)
        .def("init", &Skeleton::init)
        ;
}
