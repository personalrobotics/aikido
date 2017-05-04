#include "aikido/util/KinBodyParser.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include <dart/utils/utils.hpp>

#include "aikido/util/string.hpp"

namespace aikido {
namespace util {

static const std::string DEFAULT_KINBODY_SKELETON_NAME = "kinbody skeleton";
static const std::string DEFAULT_KINBODY_ROOT_JOINT_NAME = "root joint";
static const std::string DEFAULT_KINBODY_ROOT_BODYNODE_NAME = "root bodynode";
static const Eigen::Vector3d DEFAULT_KINBODY_MESH_SCALE
    = Eigen::Vector3d::Ones();

namespace {

using BodyPropPtr = std::shared_ptr<dart::dynamics::BodyNode::Properties>;

struct BodyNodeInfo
{
  dart::dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLDocument& kinBodyDoc,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

BodyNodeInfo readBodyNodeInfo(tinyxml2::XMLElement* bodyNodeElement);

void readGeoms(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* bodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::common::ResourceRetrieverPtr getRetriever(
    const dart::common::ResourceRetrieverPtr& retriever);

void transformElementNamesToLowerCases(tinyxml2::XMLDocument& doc);

} // anonymous namespace

//==============================================================================
dart::dynamics::SkeletonPtr readKinbodyString(
    const std::string& kinBodyString,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  const auto retriever = getRetriever(nullOrRetriever);

  // Parse XML string
  tinyxml2::XMLDocument kinBodyDoc;
  if (kinBodyDoc.Parse(kinBodyString.c_str()) != tinyxml2::XML_SUCCESS)
  {
    kinBodyDoc.PrintError();
    return nullptr;
  }

  return readKinBody(kinBodyDoc, baseUri, retriever);
}

//==============================================================================
dart::dynamics::SkeletonPtr readKinbody(
    const dart::common::Uri& kinBodyUri,
    const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  const auto retriever = getRetriever(nullOrRetriever);

  // Parse XML file
  tinyxml2::XMLDocument kinBodyDoc;
  try
  {
    dart::utils::openXMLFile(kinBodyDoc, kinBodyUri, retriever);
  }
  catch (const std::exception& e)
  {
    dtwarn << "[KinBodyParser] Failed to load '" << kinBodyUri.toString()
           << "'. Reason: " << e.what() << ". Returning nullptr.\n";
    return nullptr;
  }

  return readKinBody(kinBodyDoc, kinBodyUri, retriever);
}

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLDocument& kinBodyDoc,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  assert(retriever);

  // Transform all the element names to lower cases because the KinBody spec is
  // case-insensitive. We use lower cases to read the XML elements.
  transformElementNamesToLowerCases(kinBodyDoc);

  auto kinBodyEle = kinBodyDoc.FirstChildElement("kinbody");
  if (!kinBodyEle)
  {
    dtwarn << "[KinBodyParser] KinBody document does not contain <KinBody> as "
           << "the root element. Returning nullptr\n";
    return nullptr;
  }

  return readKinBody(kinBodyEle, baseUri, retriever);
}

//==============================================================================
dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  assert(kinBodyEle != nullptr);
  // kinBodyEle shouldn't be nullptr at all, which always should be
  // gauranteed because this function is only used internally to be so. If we
  // find the nullptr case, then we should fix the caller.

  assert(retriever);

  auto skeleton = dart::dynamics::Skeleton::create();

  // Read name attribute.
  auto name = DEFAULT_KINBODY_SKELETON_NAME;
  if (dart::utils::hasAttribute(kinBodyEle, "name"))
  {
    name = dart::utils::getAttributeString(kinBodyEle, "name");
  }
  else
  {
    dtwarn << "[KinBodyParser] The <KinBody> in '" << baseUri.toString()
           << "' doesn't have name attribute or has empty name. Assigning "
           << "'" << DEFAULT_KINBODY_SKELETON_NAME << "' instead.\n";
  }
  skeleton->setName(name);

  // Get the first Body element
  auto bodyEle = kinBodyEle->FirstChildElement("body");
  if (bodyEle == nullptr)
  {
    dtwarn << "[KinBodyParser] KinBody document '" << baseUri.toString()
           << "' does not contain <Body> element "
           << "under <KinBody> element. Returning nullptr.\n";
    return nullptr;
  }

  // Warn if there is more than one <Body> elements
  auto nextBodyEle = bodyEle->NextSiblingElement("body");
  if (nextBodyEle)
  {
    dtwarn << "[KinBodyParser] KinBody document '" << baseUri.toString()
           << "' contains more than one <Body> elements "
           << "under <KinBody> element. We only parse the first one by the "
           << "design.\n";
  }

  auto bodyNodeInfo = readBodyNodeInfo(bodyEle);

  dart::dynamics::FreeJoint::Properties jointProps;
  jointProps.mName = DEFAULT_KINBODY_ROOT_JOINT_NAME;
  jointProps.mT_ParentBodyToJoint = bodyNodeInfo.initTransform;

  auto jointAndBodyNode
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
          nullptr, jointProps, bodyNodeInfo.properties);
  assert(jointAndBodyNode.first && jointAndBodyNode.second);

  readGeoms(jointAndBodyNode.second, bodyEle, baseUri, retriever);

  return skeleton;
}

//==============================================================================
BodyNodeInfo readBodyNodeInfo(tinyxml2::XMLElement* bodyNodeEle)
{
  assert(bodyNodeEle != nullptr);
  // bodyNodeElement shouldn't be nullptr at all, which always should be
  // gauranteed because this function is only used internally to be so. If we
  // find the nullptr case, then we should fix the caller.

  BodyNodeInfo bodyNodeInfo;

  dart::dynamics::BodyNode::Properties properties;

  // Name attribute
  properties.mName = DEFAULT_KINBODY_ROOT_BODYNODE_NAME;
  if (dart::utils::hasAttribute(bodyNodeEle, "name"))
    properties.mName = dart::utils::getAttributeString(bodyNodeEle, "name");

  // transformation
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();
  if (dart::utils::hasElement(bodyNodeEle, "transformation"))
  {
    initTransform
        = dart::utils::getValueIsometry3d(bodyNodeEle, "transformation");
  }
  else
  {
    initTransform.setIdentity();
  }

  bodyNodeInfo.properties = std::move(properties);
  bodyNodeInfo.initTransform = initTransform;

  return bodyNodeInfo;
}

//==============================================================================
void readGeoms(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* bodyEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  assert(bodyNode);

  dart::utils::ElementEnumerator geomIterator(bodyEle, "geom");

  if (!bodyEle->FirstChildElement("geom"))
  {
    dtwarn << "[KinBodyParser] KinBody document '" << baseUri.toString()
           << "' does not contain any <Geom> element "
           << "under <Body>. This body will have no shape.\n";
  }

  while (geomIterator.next())
  {
    auto geomEle = geomIterator.get();
    readGeom(geomEle, bodyNode, baseUri, retriever);
  }
}

//==============================================================================
dart::dynamics::ShapePtr readMeshShape(
    const std::string& fileName,
    const Eigen::Vector3d& scale,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  auto meshUri = dart::common::Uri::getRelativeUri(baseUri, fileName);
  auto model = dart::dynamics::MeshShape::loadMesh(meshUri, retriever);

  if (model)
  {
    return std::make_shared<dart::dynamics::MeshShape>(
        scale, model, meshUri, retriever);
  }
  else
  {
    dtwarn << "[KinBodyParser] Failed to load model '" << fileName << "'. "
           << "Returning nullptr.\n";
    return nullptr;
  }
}

//==============================================================================
std::string resolveShapeName(
    tinyxml2::XMLElement* geomEle, dart::dynamics::BodyNode* bodyNode)
{
  std::string shapeNodeName;

  if (dart::utils::hasAttribute(geomEle, "name"))
  {
    shapeNodeName = dart::utils::getAttributeString(geomEle, "name");
  }
  else
  {
    shapeNodeName = bodyNode->getName() + " shape ("
                    + std::to_string(bodyNode->getShapeNodes().size()) + ")";
  }

  return shapeNodeName;
}

//==============================================================================
void checkScaleValidity(Eigen::Vector3d& scale)
{
  if (scale.any() == 0.0)
  {
    dtwarn << "[KinBodyParser]: Invalid scale (" << scale[0] << ", " << scale[1]
           << ", " << scale[2] << "). All the elements shouln't be zero.\n";
  }
}

//==============================================================================
void resolveFileNameAndScale(
    std::string& fileName,
    Eigen::Vector3d& scale,
    tinyxml2::XMLElement* geomEle,
    const std::string& elementName)
{
  // <render>, <data>, or <collision> contains the relative path to a mesh file
  // and optionally a single float (for the uni-scale) or three float's (for the
  // x, y, and z-axes) for the scale.
  //
  // Example forms:
  //   <Render>my/mesh/file.stl<Render>
  //   <Render>my/mesh/file.stl 0.25<Render> <!--Unscale>
  //   <Render>my/mesh/file.stl 0.25 0.5 2.0<Render>
  //
  // If the scale is not provided then (1, 1, 1) is used by default.

  const auto fileNameAndScale
      = util::split(dart::utils::getValueString(geomEle, elementName), " ");

  if (fileNameAndScale.size() != 1u && fileNameAndScale.size() != 2u
      && fileNameAndScale.size() != 4u)
  {
    throw std::invalid_argument(
        "[KinBodyParser]: Invalid number of arguments for file name and "
        "scale. It should be 1, 2, or 4. "
        "Ex) <Render>path/to/file.stl 0.1</Render>");
  }

  fileName = fileNameAndScale[0];

  if (fileNameAndScale.size() == 2u)
  {
    scale
        = Eigen::Vector3d::Constant(dart::utils::toDouble(fileNameAndScale[1]));
  }
  else if (fileNameAndScale.size() == 4u)
  {
    scale << dart::utils::toDouble(fileNameAndScale[1]),
        dart::utils::toDouble(fileNameAndScale[2]),
        dart::utils::toDouble(fileNameAndScale[3]);
  }
  else
  {
    assert(fileNameAndScale.size() == 1u);
    scale = DEFAULT_KINBODY_MESH_SCALE;
  }

  checkScaleValidity(scale);
}

//==============================================================================
void shouldBeNonPositive(const Eigen::Vector3d& extents)
{
  if (extents.any() <= 0.0)
  {
    dtwarn << "[KinBodyParser]: Invalid extents (" << extents[0] << ", "
           << extents[1] << ", " << extents[2]
           << "). All the elements should be positive. "
           << "If you used 0 value intentionally to create a pure "
           << "visualization geometry, try to use 'none' attribute for the "
           << "geom type instead.\n";
  }
}

//==============================================================================
void shouldBeNonPositive(const std::string& name, double val)
{
  if (val <= 0.0)
  {
    dtwarn << "[KinBodyParser]: Invalid " << name << " '" << val
           << "'. The value should be positive to get reliable simulation "
           << "results. If you used 0 value intentionally to create a pure "
           << "visualization geometry, try to use 'none' attribute for the "
           << "geom type instead.\n";
  }
}

//==============================================================================
void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  // `type` attribute should be one of `none`, `box`, `sphere`, `trimesh`,
  // or `cylinder`
  auto typeAttr = dart::utils::getAttributeString(geomEle, "type");

  dart::dynamics::ShapeNode* collShapeNode = nullptr;
  std::string fileNameOfCollision;
  Eigen::Vector3d scaleOfCollision = Eigen::Vector3d::Ones();

  auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

  if (typeAttr == "none")
  {
    // Do nothing.
  }
  else if (typeAttr == "box")
  {
    // Note: OpenRAVE uses 'extents' to refer to half extents.
    auto halfExtents = dart::utils::getValueVector3d(geomEle, "extents");
    shouldBeNonPositive(halfExtents);

    auto shape = std::make_shared<dart::dynamics::BoxShape>(2.0 * halfExtents);

    collShapeNode
        = bodyNode->createShapeNodeWith<dart::dynamics::CollisionAspect>(
            shape, shapeNodeName);
  }
  else if (typeAttr == "sphere")
  {
    auto radius = dart::utils::getValueDouble(geomEle, "radius");
    shouldBeNonPositive("radius", radius);

    auto shape = std::make_shared<dart::dynamics::SphereShape>(radius);

    collShapeNode
        = bodyNode->createShapeNodeWith<dart::dynamics::CollisionAspect>(
            shape, shapeNodeName);
  }
  else if (typeAttr == "cylinder")
  {
    auto radius = dart::utils::getValueDouble(geomEle, "radius");
    auto height = dart::utils::getValueDouble(geomEle, "height");
    shouldBeNonPositive("radius", radius);
    shouldBeNonPositive("height", height);

    auto shape
        = std::make_shared<dart::dynamics::CylinderShape>(radius, height);

    collShapeNode
        = bodyNode->createShapeNodeWith<dart::dynamics::CollisionAspect>(
            shape, shapeNodeName);
  }
  else if (typeAttr == "trimesh")
  {
    std::string collisionOrData;

    auto hasCollisionEle = dart::utils::hasElement(geomEle, "collision");
    auto hasDataEle = dart::utils::hasElement(geomEle, "data");

    if (hasCollisionEle && hasDataEle)
    {
      dtwarn << "[KinBodyParser]: <Geom> element contains both of <Collision> "
             << "and <Data> as the child elements. <Data> element is "
             << "ignored.\n";
    }

    if (hasCollisionEle)
    {
      collisionOrData = "collision";
    }
    else if (hasDataEle)
    {
      collisionOrData = "data";
    }
    else
    {
      dtwarn << "[KinBodyParser] <Geom> element doesn't contain neither of "
             << "<Collision> and <Data> as the child element. Not creating "
             << "collision geometry for this <Geom>.\n";
      // We don't return here because there is a chance to create visualization
      // geometry if <Render> element is provided.
    }

    resolveFileNameAndScale(
        fileNameOfCollision, scaleOfCollision, geomEle, collisionOrData);
    auto shape = readMeshShape(
        fileNameOfCollision, scaleOfCollision, baseUri, retriever);

    collShapeNode
        = bodyNode->createShapeNodeWith<dart::dynamics::CollisionAspect>(
            shape, shapeNodeName);
  }
  else
  {
    dtwarn << "[KinBodyParser] Attempts to parse unsupported geom type '"
           << typeAttr << "'. Not creating collision geometry for this "
           << "<Geom>.\n";
    // We don't return here because there is a chance to create visualization
    // geometry if <Render> element is provided.
  }

  //
  // Logic of what is used as the visual geometry:
  // * If <Geom>'s render attribute is false, display nothing. Otherwise...
  // * If <Render> is present, render the trimesh at the specified path.
  //   Otherwise...
  // * Render the collision geometry.
  //

  // `render` is an optional attribute and true by default.
  bool renderAttr = true;
  if (dart::utils::hasAttribute(geomEle, "render"))
    renderAttr = dart::utils::getAttributeBool(geomEle, "render");

  if (!renderAttr)
    return;

  const auto hasRenderEle = dart::utils::hasElement(geomEle, "render");

  if (hasRenderEle)
  {
    std::string fileNameOfRender;
    Eigen::Vector3d scaleOfRender;
    resolveFileNameAndScale(fileNameOfRender, scaleOfRender, geomEle, "render");
    auto renderShapeNodeName = resolveShapeName(geomEle, bodyNode);

    if (typeAttr == "trimesh" && fileNameOfCollision == fileNameOfRender
        && scaleOfCollision == scaleOfRender)
    {
      collShapeNode->createVisualAspect();
    }
    else
    {
      auto shape
          = readMeshShape(fileNameOfRender, scaleOfRender, baseUri, retriever);

      // Create additional ShapeNode only for visualization
      bodyNode->createShapeNodeWith<dart::dynamics::VisualAspect>(
          shape, renderShapeNodeName);
    }
  }
  else
  {
    // `none` or invalid Geom type can reach here that hasn't created a
    // ShapeNode. We don't create the visual geometry for those types when
    // <Render> element is not provided.
    if (collShapeNode)
      collShapeNode->createVisualAspect();
  }
}

//==============================================================================
dart::common::ResourceRetrieverPtr getRetriever(
    const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  if (nullOrRetriever)
    return nullOrRetriever;
  else
    return std::make_shared<dart::common::LocalResourceRetriever>();
}

//==============================================================================
std::string toLowerCases(const std::string& in)
{
  std::string out(in);
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);

  return out;
}

//==============================================================================
void transformElementNamesToLowerCasesRecurse(tinyxml2::XMLElement* ele)
{
  std::string eleName = ele->Name();
  ele->SetName(toLowerCases(eleName).c_str());

  auto childEle = ele->FirstChildElement();
  while (childEle)
  {
    transformElementNamesToLowerCasesRecurse(childEle);
    childEle = childEle->NextSiblingElement();
  }
}

//==============================================================================
void transformElementNamesToLowerCases(tinyxml2::XMLDocument& doc)
{
  auto firstEle = doc.FirstChildElement();
  if (firstEle)
    transformElementNamesToLowerCasesRecurse(firstEle);
}

} // anonymous namespace
} // namespace utils
} // namespace dart
