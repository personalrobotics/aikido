#ifndef AIKIDO_UTIL_KINBODY_PARSER_H
#define AIKIDO_UTIL_KINBODY_PARSER_H

#include "dart/dart.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"

//Currently assumes only one body node

namespace aikido {

namespace util {

namespace KinBodyParser {

  dart::dynamics::SkeletonPtr readKinBodyXMLFile(
    const dart::common::Uri& _fileUri,
    const dart::common::ResourceRetrieverPtr& _retriever = nullptr);

} //namespace KinBodyParser

} //namespace utils

} //namespace aikido

#endif