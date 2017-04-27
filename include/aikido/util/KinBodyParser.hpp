#ifndef AIKIDO_UTIL_KINBODYPARSER_HPP_
#define AIKIDO_UTIL_KINBODYPARSER_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace util {

/// Read skeleton from a string of OpenRAVE's custom XML format
///
/// This function only parses a subset of the format assuming only one body node
/// in a kinbody file.
///
/// The following are the available fields that this parser can read.
/// \code
/// kinbody - attributes: name
///   body - attributes: name
///     geom - attributes: name, type* (none, box, sphere, trimesh, cylinder),
///            render
///       data* (or collision) - file* scale [for trimesh] (see below for the
///                              detail)
///       extents* - 3 float [for box]
///       height* - float [for cylinder]
///       radius* - float [for cylinder and sphere]
///       render - file* scale (see below for the detail)
/// \endcode
/// Elements marked with `*` are required.
///
/// <render>, <data>, or <collision> contain the relative path to a mesh file
/// and optionally a single float (for all three axes) or three float's (for the
/// x, y, and z-axes) for the scale.
///
/// Example forms:
///   <Render>my/mesh/file.stl<Render>
///   <Render>my/mesh/file.stl 0.25<Render> <!--scale for all three axes-->
///   <Render>my/mesh/file.stl 0.25 0.5 2.0<Render>
///
/// If the scale is not provided then (1, 1, 1) is used by default.
///
/// The detail of the format can be found at:
/// http://openrave.programmingvision.com/wiki/index.php/Format:XML.
///
/// \param[in] kinBodyString The KinBody XML string.
/// \param[in] baseUri The base URI of the mesh files in the KinBody XML string.
/// If an empty URI, which is the default, is passed, the mesh URIs in the
/// KinBody XML string should be absolute URIs or paths.
/// \param[in] retriever A DART retriever for the mesh URIs in the KinBody XML
/// string. If nullptr is passed, this function uses a local file resource
/// retriever which only can parse absolute file paths or file URIs
/// (e.g., file://path/to/local/file.kinbody.xml).
/// \return The parsed DART skeleton; returns nullptr on failure.
///
/// \sa readKinbody
dart::dynamics::SkeletonPtr readKinbodyString(
    const std::string& kinBodyString,
    const dart::common::Uri& baseUri = "",
    const dart::common::ResourceRetrieverPtr& retriever = nullptr);

/// Read skeleton from a file of OpenRAVE's custom XML format
///
/// This function only parses a subset of the format assuming only one body node
/// in a kinbody file.
///
/// The detail of the format can be found at:
/// http://openrave.programmingvision.com/wiki/index.php/Format:XML.
///
/// \param[in] kinBodyUri The URI to a KinBody file. If the URI scheme is not
/// file (i.e., file://), a relevant ResourceRetriever should be passed to
/// retrieve the KinBody file.
/// \param[in] retriever A DART retriever for the URI to a KinBody. The
/// retriever is also used to read the mesh URI in the KinBody file. If nullptr
/// is passed, this function uses a local file resource retriever which only can
/// parse absolute file paths or file URIs
/// (e.g., file://path/to/local/file.kinbody.xml).
/// \return The parsed DART skeleton; returns nullptr on failure.
///
/// \sa readKinbodyString
dart::dynamics::SkeletonPtr readKinbody(
    const dart::common::Uri& kinBodyUri,
    const dart::common::ResourceRetrieverPtr& retriever = nullptr);

} // namespace utils
} // namespace aikido

#endif // AIKIDO_UTIL_KINBODYPARSER_HPP_
