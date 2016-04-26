/** 
 * @file PerceptionModule.hpp
 * @author Shushman Choudhury
 * @date Apr 10, 2016
 * @brief The interface for the generic perception module.
 */

#ifndef AIKIDO_PERCEPTION_PERCEPTIONMODULE_H
#define AIKIDO_PERCEPTION_PERCEPTIONMODULE_H

namespace aikido {
namespace perception{

class PerceptionModule
{
public:

    //! The virtual destructor
    virtual ~PerceptionModule() = default;

    //! The pure virtual method that is called by each perception module
    /*!
        \param skeleton_list the set of skeletons currently in context. It will either be added to or updated
        \param timeout the duration up to which to wait for the transform.
        \param timestamp only detections more recent than this timestamp will be accepted.
    */  
    virtual void detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double timeout, ros::Time timestamp) = 0;

};

} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_PERCEPTIONMODULE_H