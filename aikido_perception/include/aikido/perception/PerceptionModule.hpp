#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

/// The interface for the generic perception module.

class PerceptionModule
{
public:

    /// The virtual destructor
    virtual ~PerceptionModule() = default;

    /// The pure virtual method that is called by each perception module
    /// \param[in] skeleton_list the set of skeletons currently in context. It will either be added to or updated
    /// \param[in] timeout the duration up to which to wait for the transform.
    /// \param[in] timestamp only detections more recent than this timestamp will be accepted.
    /// \return the updated list of skeletons
    virtual void detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double timeout, ros::Time timestamp) = 0;

};

} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H