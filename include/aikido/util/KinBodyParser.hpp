#ifndef AIKIDO_UTIL_KINBODY_PARSER_HPP_
#define AIKIDO_UTIL_KINBODY_PARSER_HPP_

#include <dart/dart.hpp>

// KinBodyParser is to parse the OpenRAVE's custom XML format
// (http://openrave.programmingvision.com/wiki/index.php/Format:XML). Currently,
// KinBodyParser only parses a subset of the format assuming only one body node
// in a kinbody file.

namespace aikido {
namespace util {
namespace KinBodyParser {

/// Read skeleton from a xml-formatted string
dart::dynamics::SkeletonPtr readSkeletonXML(
  const std::string& xmlString,
  const dart::common::Uri& baseUri = "",
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

/// Read skeleton from an uri
dart::dynamics::SkeletonPtr readSkeleton(
  const dart::common::Uri& fileUri,
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

} //namespace KinBodyParser
} //namespace utils
} //namespace aikido

#endif
