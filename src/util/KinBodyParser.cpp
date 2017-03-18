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
namespace {

using BodyPropPtr = std::shared_ptr<dart::dynamics::BodyNode::Properties>;
using JointPropPtr = std::shared_ptr<dart::dynamics::Joint::Properties>;

struct SkelBodyNode
{
  BodyPropPtr properties;
  Eigen::Isometry3d initTransform;
  std::string type;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SkelJoint
{
  JointPropPtr properties;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd force;
  std::string parentName;
  std::string childName;
  std::string type;
};

// first: BodyNode name | second: BodyNode information
using BodyMap = Eigen::aligned_map<std::string, SkelBodyNode>;

// first: Child BodyNode name | second: Joint information
using JointMap = std::map<std::string, SkelJoint>;

// first: Order that Joint appears in file | second: Child BodyNode name
using IndexToJoint = std::map<size_t, std::string>;

// first: Child BodyNode name | second: Order that Joint appears in file
using JointToIndex = std::map<std::string, size_t>;

dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

SkelBodyNode readBody(
    tinyxml2::XMLElement* bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const dart::common::Uri& baseUri);

void readGeoms(
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::common::ResourceRetrieverPtr getRetriever(
  const dart::common::ResourceRetrieverPtr& retriever);

tinyxml2::XMLElement* checkFormatAndGetKinBodyElement(
    tinyxml2::XMLDocument& doc);

tinyxml2::XMLElement* transformElementNamesToLowerCases(tinyxml2::XMLDocument& doc);

std::vector<std::string> split(
    const std::string& str, const std::string& delimiters = " ");

} //anonymous namespace

//==============================================================================
dart::dynamics::SkeletonPtr KinBodyParser::readSkeletonXML(
  const std::string& xmlString,
  const dart::common::Uri& baseUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  const auto retriever = getRetriever(nullOrRetriever);

  tinyxml2::XMLDocument kinBodyDoc;
  if (kinBodyDoc.Parse(xmlString.c_str()) != tinyxml2::XML_SUCCESS)
  {
    kinBodyDoc.PrintError();
    return nullptr;
  }

  // Transform all the element names to lower cases
  transformElementNamesToLowerCases(kinBodyDoc);

  auto kinBodyElement = checkFormatAndGetKinBodyElement(kinBodyDoc);
  if (!kinBodyElement)
  {
    dterr << "[KinBodyParser] XML string could not be parsed.\n";
    return nullptr;
  }

  return readKinBody(kinBodyElement, baseUri, retriever);
}

//==============================================================================
dart::dynamics::SkeletonPtr KinBodyParser::readSkeleton(
  const dart::common::Uri& fileUri,
  const dart::common::ResourceRetrieverPtr& nullOrRetriever)
{
  const auto retriever = getRetriever(nullOrRetriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument kinBodyDoc;
  try
  {
    dart::utils::openXMLFile(kinBodyDoc, fileUri, nullOrRetriever);
  }
  catch(std::exception const& e)
  {
    dterr << "[KinBodyParser] LoadFile '" << fileUri.toString() << "' Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  // Transform all the element names to lower cases
  transformElementNamesToLowerCases(kinBodyDoc);

  auto kinBodyElement = checkFormatAndGetKinBodyElement(kinBodyDoc);
  if (!kinBodyElement)
  {
    dterr << "[KinBodyParser] XML string could not be parsed.\n";
    return nullptr;
  }

  return readKinBody(kinBodyElement, fileUri, retriever);
}

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  assert(kinBodyEle != nullptr);

  auto newSkeleton = dart::dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  // Name attribute
  auto name = dart::utils::getAttributeString(kinBodyEle, "name");
  newSkeleton->setName(name);

  // Get Body node
  auto bodyElement = kinBodyEle->FirstChildElement("body");
  if (bodyElement == nullptr)
  {
    dterr << "[KinBodyParser] KinBody file '" << baseUri.toString()
          << "' does not contain <Body> element "
          << "under <KinBody> element.\n";
    return nullptr;
  }

  auto newBodyNode = readBody(bodyElement, skeletonFrame, baseUri);

  // Add a free joint
  SkelJoint rootJoint;
  rootJoint.properties = 
      Eigen::make_aligned_shared<dart::dynamics::FreeJoint::Properties>(
            dart::dynamics::Joint::Properties("root_joint", newBodyNode.initTransform));
  rootJoint.type = "free";

  std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> jbn_pair;
  jbn_pair = newSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint, dart::dynamics::BodyNode>(nullptr,
      static_cast<const dart::dynamics::FreeJoint::Properties&>(*rootJoint.properties),
      static_cast<const dart::dynamics::BodyNode::Properties&>(*newBodyNode.properties));

  assert(jbn_pair.first != nullptr && jbn_pair.second != nullptr);

  readGeoms(newSkeleton, kinBodyEle, baseUri, retriever);
  newSkeleton->resetPositions();
  newSkeleton->resetVelocities();

  return newSkeleton;
}

//==============================================================================
SkelBodyNode readBody(
    tinyxml2::XMLElement* bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const dart::common::Uri& baseUri)
{
  assert(bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dart::dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  if (!dart::utils::hasAttribute(bodyNodeElement, "name"))
  {
    dterr << "[KinBodyParser] <Body> in '" << baseUri.toString()
          << "' doesn't have name attribute.\n";
    assert(0);
  }
  newBodyNode->mName = dart::utils::getAttributeString(bodyNodeElement, "name");

  // transformation
  if (dart::utils::hasElement(bodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        dart::utils::getValueIsometry3d(bodyNodeElement, "transformation");
    initTransform = skeletonFrame * W;
  }
  else
  {
    initTransform = skeletonFrame;
  }

  //Get SkelBodyNode from PropPtr
  SkelBodyNode skelBodyNode;
  skelBodyNode.properties = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  return skelBodyNode;
}

//==============================================================================
void readGeoms(
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* kinBodyEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::utils::ElementEnumerator bodieIterator(kinBodyEle, "body");
  while (bodieIterator.next())
  {
    auto bodyEle = bodieIterator.get();
    auto bodyNodeName = dart::utils::getAttributeString(bodyEle, "name");
    auto bodyNode = skeleton->getBodyNode(bodyNodeName);

    dart::utils::ElementEnumerator geomIterator(bodyEle, "geom");

    if (!bodyEle->FirstChildElement("geom"))
    {
      dtwarn << "[KinBodyParser] KinBody file '"
             << baseUri.toString() << "' doesn not contain any <geom> element "
             << "under <body name='" << bodyNodeName << "'>.\n";
    }

    while (geomIterator.next())
    {
      auto geomEle = geomIterator.get();
      readGeom(geomEle, bodyNode, baseUri, retriever);
    }
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
  const dart::common::ResourceRetrieverPtr& _retriever)
{
  if (_retriever)
    return _retriever;
  else
    return std::make_shared<dart::common::LocalResourceRetriever>();
}

//==============================================================================
tinyxml2::XMLElement* checkFormatAndGetKinBodyElement(
  tinyxml2::XMLDocument& doc)
{
  auto kinBodyElement = doc.FirstChildElement("kinbody");
  if (kinBodyElement == nullptr)
  {
    dterr << "[KinBodyParser] XML document does not contain <kinbody> as the "
          << "root element.\n";
    return nullptr;
  }

  return kinBodyElement;
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
