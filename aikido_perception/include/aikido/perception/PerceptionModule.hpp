#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

/// The interface for the generic perception module. The \c detectObjects method implements the particular
/// detector via a service call or ROS message reception, and updates the list of DART skeletons that
/// represents the world.
class PerceptionModule
{
public:

    virtual ~PerceptionModule() = default;

    /// The pure virtual method that is implemented by each perception module
    /// \param[in,out] skeleton_list the set of skeletons currently in context. It will either be added to or updated
    /// \param[in] timeout the duration up to which to wait for the transform. Returns false if none of the markers get correctly transformed
    /// \param[in] timestamp only detections more recent than this timestamp will be accepted. A timestamp of 0 greedily takes the first
    ///  available message, and is the default behaviour. Returns false if no marker has a more recent timestamp than the parameter
    virtual bool detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list, ros::Duration timeout, ros::Time timestamp) = 0;

};

} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H