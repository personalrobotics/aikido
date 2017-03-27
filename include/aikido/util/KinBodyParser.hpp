#ifndef AIKIDO_UTIL_KINBODYPARSER_HPP_
#define AIKIDO_UTIL_KINBODYPARSER_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace util {

/// Read skeleton from a string of OpenRAVE's custom XML format
///
/// The detail of the format can be found at:
/// http://openrave.programmingvision.com/wiki/index.php/Format:XML).
///
/// This function only parses a subset of the format assuming only one body node
/// in a kinbody file.
///
/// \param[in] kinBodyString The Kinbody XML string.
/// \param[in] baseUri The base URI of the mesh files in the KinBody XML string.
/// \param[in] retriever A DART retriever for the mesh files in the KinBody XML
/// string. If nullptr is passed, a local file resource retriever is used by
/// default.
/// \return The parsed DART skeleton; returns nullptr on failure.
dart::dynamics::SkeletonPtr readKinbodyString(
  const std::string& kinBodyString,
  const dart::common::Uri& baseUri = "",
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

/// Read skeleton from a file of OpenRAVE's custom XML format
///
/// The detail of the format can be found at:
/// http://openrave.programmingvision.com/wiki/index.php/Format:XML).
///
/// This function only parses a subset of the format assuming only one body node
/// in a kinbody file.
///
/// \param[in] kinBodyFileUri The file URI ("file://...") of the KinBody file.
/// \param[in] retriever A DART retriever for the KinBody file and mesh files
///  in the KinBody file. If nullptr is passed, a local file resource retriever
/// is used by default.
/// \return The parsed DART skeleton; returns nullptr on failure.
dart::dynamics::SkeletonPtr readKinbody(
  const dart::common::Uri& kinBodyFileUri,
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

} //namespace utils
} //namespace aikido

#endif // AIKIDO_UTIL_KINBODYPARSER_HPP_
