#include "aikido/util/KinBodyParser.hpp"

#include <algorithm>
#include <regex>
#include <string>
#include <sstream>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include <dart/utils/utils.hpp>

namespace aikido {
namespace util {

static const std::string DEFAULT_KINBODY_ROOT_JOINTNAME = "root joint";

namespace {

using BodyPropPtr = std::shared_ptr<dart::dynamics::BodyNode::Properties>;

struct BodyNodeInfo
{
  dart::dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform;
  bool valid;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::dynamics::SkeletonPtr readKinBody(
  tinyxml2::XMLDocument& kinBodyDoc,
  const dart::common::Uri& baseUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever);

BodyNodeInfo readBodyNodeInfo(
    tinyxml2::XMLElement* bodyNodeElement,
    const dart::common::Uri& baseUri);

void readGeoms(
    const dart::dynamics::SkeletonPtr& skeleton,
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

tinyxml2::XMLElement* transformElementNamesToLowerCases(
    tinyxml2::XMLDocument& doc);

std::vector<std::string> split(
    const std::string& str, const std::string& delimiters = " ");

} //anonymous namespace

//==============================================================================
dart::dynamics::SkeletonPtr readKinbodyString(
  const std::string& kinBodyString,
  const dart::common::Uri& baseUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  // Parse XML string
  tinyxml2::XMLDocument kinBodyDoc;
  if (kinBodyDoc.Parse(kinBodyString.c_str()) != tinyxml2::XML_SUCCESS)
  {
    kinBodyDoc.PrintError();
    return nullptr;
  }

  return readKinBody(kinBodyDoc, baseUri, nullOrRetriever);
}

//==============================================================================
dart::dynamics::SkeletonPtr readKinbody(
  const dart::common::Uri& kinBodyFileUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  // Parse XML file
  tinyxml2::XMLDocument kinBodyDoc;
  try
  {
    dart::utils::openXMLFile(kinBodyDoc, kinBodyFileUri, nullOrRetriever);
  }
  catch(std::exception const& e)
  {
    dterr << "[KinBodyParser] LoadFile '" << kinBodyFileUri.toString()
          << "' Fails: " << e.what() << "\n";
    return nullptr;
  }

  return readKinBody(kinBodyDoc, kinBodyFileUri, nullOrRetriever);
}

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr readKinBody(
  tinyxml2::XMLDocument& kinBodyDoc,
  const dart::common::Uri& baseUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  const auto retriever = getRetriever(nullOrRetriever);

  // Transform all the element names to lower cases because the KinBody spec is
  // case-insensitive. We use lower cases to read the XML elements.
  transformElementNamesToLowerCases(kinBodyDoc);

  auto kinBodyEle = kinBodyDoc.FirstChildElement("kinbody");
  if (!kinBodyEle)
  {
    dterr << "[KinBodyParser] KinBody document does not contain <KinBody> as "
          << "the root element.\n";
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

  auto skeleton = dart::dynamics::Skeleton::create();

  // Read name attribute.
  //
  // If name attribute doesn't exist, getAttributeString() returns empty string.
  // Empty name for skeleton is allowed.
  auto name = dart::utils::getAttributeString(kinBodyEle, "name");
  if (name.empty())
  {
    dtwarn << "[KinBodyParser] The <KinBody> in '" << baseUri.toString()
           << "' doesn't have name attribute or has empty name.\n";
  }
  skeleton->setName(name);

  // Get Body node
  auto bodyEle = kinBodyEle->FirstChildElement("body");
  if (bodyEle == nullptr)
  {
    dterr << "[KinBodyParser] KinBody document '" << baseUri.toString()
          << "' does not contain <Body> element "
          << "under <KinBody> element.\n";
    return nullptr;
  }

  auto bodyNodeInfo = readBodyNodeInfo(bodyEle, baseUri);
  if (!bodyNodeInfo.valid)
    return nullptr;

  dart::dynamics::FreeJoint::Properties jointProps;
  jointProps.mName = DEFAULT_KINBODY_ROOT_JOINTNAME;
  jointProps.mT_ParentBodyToJoint = bodyNodeInfo.initTransform;

  auto jointAndBodyNode = skeleton->createJointAndBodyNodePair<
      dart::dynamics::FreeJoint>(nullptr, jointProps, bodyNodeInfo.properties);
  assert(jointAndBodyNode.first && jointAndBodyNode.second);

  readGeoms(skeleton, bodyEle, baseUri, retriever);
  skeleton->resetPositions();
  skeleton->resetVelocities();

  return skeleton;
}

//==============================================================================
BodyNodeInfo readBodyNodeInfo(
    tinyxml2::XMLElement* bodyNodeElement,
    const dart::common::Uri& baseUri)
{
  assert(bodyNodeElement != nullptr);
  // bodyNodeElement shouldn't be nullptr at all, which always should be
  // gauranteed because this function is only used internally to be so. If we
  // find the nullptr case, then we should fix the caller.

  BodyNodeInfo bodyNodeInfo;
  bodyNodeInfo.valid = true;

  dart::dynamics::BodyNode::Properties properties;

  // Name attribute
  properties.mName = dart::utils::getAttributeString(bodyNodeElement, "name");
  if (properties.mName.empty())
  {
    dterr << "[KinBodyParser] <Body> in '" << baseUri.toString()
          << "' doesn't have name attribute or has empty name, which is not "
          << "allowed.\n";
    bodyNodeInfo.valid = false;
  }

  // transformation
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();
  if (dart::utils::hasElement(bodyNodeElement, "transformation"))
  {
    initTransform
        = dart::utils::getValueIsometry3d(bodyNodeElement, "transformation");
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
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* bodyEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  auto bodyNodeName = dart::utils::getAttributeString(bodyEle, "name");
  auto bodyNode = skeleton->getBodyNode(bodyNodeName);
  assert(bodyNode);

  dart::utils::ElementEnumerator geomIterator(bodyEle, "geom");

  if (!bodyEle->FirstChildElement("geom"))
  {
    dtwarn << "[KinBodyParser] KinBody document '"
           << baseUri.toString() << "' does not contain any <Geom> element "
           << "under <Body name='" << bodyNodeName << "'>. This body will "
           << "have no shape.\n";
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
    double uniScale,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  Eigen::Vector3d scale = Eigen::Vector3d::Constant(uniScale);

  auto meshUri = dart::common::Uri::getRelativeUri(baseUri, fileName);
  auto model = dart::dynamics::MeshShape::loadMesh(meshUri, retriever);

  if (model)
  {
    return std::make_shared<dart::dynamics::MeshShape>(
          scale, model, meshUri, retriever);
  }
  else
  {
    dterr << "[KinBodyParser] Fail to load model '" << fileName << "'.\n";
    return nullptr;
  }
}

//==============================================================================
std::string resolveShapeName(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode)
{
  std::string shapeNodeName;

  if (dart::utils::hasAttribute(geomEle, "name"))
  {
    shapeNodeName = dart::utils::getAttributeString(geomEle, "name");
  }
  else
  {
    shapeNodeName = bodyNode->getName()
        + " shape (" + std::to_string(bodyNode->getShapeNodes().size()) + ")";
  }

  return shapeNodeName;
}

//==============================================================================
std::pair<std::string, double> resolveFileNameAndScale(
    tinyxml2::XMLElement* geomEle,
    const std::string& renderOrData)
{
  // <Render> or <Data> contains the relative mesh file path and optionally the
  // mesh scale (e.g., <Render>my/mesh/file.stl<Render> or
  // <Render>my/mesh/file.stl 0.25<Render>). If the scale is not provided then
  // this function returns 1.0 by default.

  std::pair<std::string, double> ret{"", 1.0};

  const auto fileNameAndScale
      = split(dart::utils::getValueString(geomEle, renderOrData), " ");
  assert(!fileNameAndScale.empty());

  ret.first = fileNameAndScale[0];

  if (fileNameAndScale.size() > 1u)
  {
    ret.second = std::atof(fileNameAndScale[1].c_str());
    // TODO(JS): warning if zero scale
  }

  return ret;
}

//==============================================================================
void readTriMeshGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  const auto hasRender = dart::utils::hasElement(geomEle, "render");
  const auto hasData = dart::utils::hasElement(geomEle, "data");

  if (!hasRender && !hasData)
  {
    dterr << "[KinBodyParser] <Geom> doesn't contains any of <Render> or "
          << "<Data>, which is invalid. Please add at least one of them.\n";
    return;
  }

  std::string fileNameOfRender;
  std::string fileNameOfData;

  double uniScaleOfRender;
  double uniScaleOfData;

  if (hasRender)
  {
    std::tie(fileNameOfRender, uniScaleOfRender)
        = resolveFileNameAndScale(geomEle, "render");
  }

  if (hasData)
  {
    std::tie(fileNameOfData, uniScaleOfData)
        = resolveFileNameAndScale(geomEle, "data");
  }

  // If both <Render> and <Data> exist and refer to the same mesh file, then
  // create one ShapeNode with visual/collision/dynamics aspects for the same
  // shape.
  if (hasRender && hasData && fileNameOfRender == fileNameOfData)
  {
    auto shape
        = readMeshShape(fileNameOfRender, uniScaleOfRender, baseUri, retriever);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);

    return;
  }

  if (hasRender)
  {
    auto shape
        = readMeshShape(fileNameOfRender, uniScaleOfRender, baseUri, retriever);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect>(shape, shapeNodeName);
  }

  if (hasData)
  {
    auto shape
        = readMeshShape(fileNameOfData, uniScaleOfData, baseUri, retriever);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
}

//==============================================================================
void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  auto attribType = dart::utils::getAttributeString(geomEle, "type");

  if (attribType == "box")
  {
    auto extents = dart::utils::getValueVector3d(geomEle, "extents");
    auto shape = std::make_shared<dart::dynamics::BoxShape>(extents);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
  else if (attribType == "sphere")
  {
    auto radius = dart::utils::getValueDouble(geomEle, "radius");
    auto shape = std::make_shared<dart::dynamics::SphereShape>(radius);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
  else if (attribType == "cylinder")
  {
    auto radius = dart::utils::getValueDouble(geomEle, "radius");
    auto height = dart::utils::getValueDouble(geomEle, "height");
    auto shape
        = std::make_shared<dart::dynamics::CylinderShape>(radius, height);
    auto shapeNodeName = resolveShapeName(geomEle, bodyNode);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
  else if(attribType == "trimesh")
  {
    readTriMeshGeom(geomEle, bodyNode, baseUri, retriever);
  }
  else
  {
    dterr << "[KinBodyParser] Attempts to parse unsupported geom type '"
          << attribType << "'. Ignoring this geom.\n";
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
tinyxml2::XMLElement* transformElementNamesToLowerCases(
    tinyxml2::XMLDocument& doc)
{
  auto firstEle = doc.FirstChildElement();
  if (firstEle)
    transformElementNamesToLowerCasesRecurse(firstEle);
}

//==============================================================================
std::vector<std::string> split(
    const std::string& str, const std::string& delimiters)
{
  std::vector<std::string> tokens;

  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);

  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));

    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);

    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }

  return tokens;
}

} //anonymous namespace
} //namespace utils
} //namespace dart
