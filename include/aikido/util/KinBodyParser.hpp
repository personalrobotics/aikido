#ifndef AIKIDO_UTIL_KINBODY_PARSER_HPP_
#define AIKIDO_UTIL_KINBODY_PARSER_HPP_

#include <dart/dart.hpp>

//Currently assumes only one body node

namespace aikido {
namespace util {
namespace KinBodyParser {

dart::dynamics::SkeletonPtr readKinBodyXMLFile(
  const dart::common::Uri& fileUri,
  const dart::common::ResourceRetrieverPtr& retriever = nullptr);

} //namespace KinBodyParser
} //namespace utils
} //namespace aikido

#endif
