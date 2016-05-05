#ifndef AIKIDO_PERCEPTION_APRILTAGS_DATABASE_H
#define AIKIDO_PERCEPTION_APRILTAGS_DATABASE_H

#include <Eigen/Dense>
#include <dart/dart.h>

namespace aikido{
namespace perception{

/// An interface for providing configuration information for the AprilTags perception module.
/// It provides a callback that maps the AprilTag ID to the information of the object it
/// is attached to - name, resource URI, and relative transform.
class AprilTagsDatabase
{
public:

    virtual ~AprilTagsDatabase() = default;

    /// Look up the name of the detected AprilTag and return
    /// the name of the body, the resource file for the body and the relative offset
    /// between the tag position and object origin
    /// 
    /// \param[in] _tagName the ID of the tag to look up
    /// \param[out] body_name the name of the object associated with the tag
    /// \param[out] body_resource the resource that points to the URDF file of the object
    /// \param[out] body_offset the relative transform between the tag and the object it is attached to
    /// \return bool indicates success in obtaining all information, else failure
    virtual bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) = 0;

};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_APRILTAGS_DATABASE_H