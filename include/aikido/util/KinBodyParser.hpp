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
/// \param[in] kinBodyString todo
/// \param[in] baseUri todo
/// \param[in] retriever todo
/// \return todo
dart::dynamics::SkeletonPtr readSkeletonXML(
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
/// \param[in] kinBodyFileUri todo
/// \param[in] retriever todo
/// \return todo
dart::dynamics::SkeletonPtr readSkeleton(
  const dart::common::Uri& kinBodyFileUri,
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

} //namespace utils
} //namespace aikido

#endif // AIKIDO_UTIL_KINBODYPARSER_HPP_
