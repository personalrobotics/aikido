#include "aikido/util/KinBodyParser.hpp"

#include <algorithm>
#include <regex>
#include <string>
#include <sstream>
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

//Method defs
dart::dynamics::SkeletonPtr readKinBody(
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

SkelBodyNode readBodyNode(
    tinyxml2::XMLElement* bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readGeoms(
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* kinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::common::ResourceRetrieverPtr getRetriever(
  const dart::common::ResourceRetrieverPtr& retriever);

tinyxml2::XMLElement* checkFormatAndGetKinBodyElement(
    tinyxml2::XMLDocument& doc);

tinyxml2::XMLElement* transformToLowerCases(tinyxml2::XMLDocument& doc);

std::vector<std::string> split(
  const std::string& str,
  const std::vector<std::string>& delimiters);

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
  transformToLowerCases(kinBodyDoc);

  auto kinBodyElement = checkFormatAndGetKinBodyElement(kinBodyDoc);
  if (!kinBodyElement)
  {
    dterr << "[KinBodyParser::readSkeletonXML] XML string could not be "
          << "parsed.\n";
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
    dterr << "[KinBodyParser] LoadFile [" << fileUri.toString() << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  // Transform all the element names to lower cases
  transformToLowerCases(kinBodyDoc);

  auto kinBodyElement = checkFormatAndGetKinBodyElement(kinBodyDoc);
  if (!kinBodyElement)
  {
    dterr << "[KinBodyParser::readSkeletonXML] XML string could not be "
          << "parsed.\n";
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
    dterr << "[KinBodyParser] KinBody file [" << baseUri.toString()
          << "] does not contain <Body> element "
          << "under <KinBody> element.\n";
    return nullptr;
  }

  auto newBodyNode = readBodyNode(
    bodyElement, skeletonFrame,baseUri, retriever);

  // How to add newBodyNode to new Skeleton?
  // Also need to add some static joint I guess
  
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
SkelBodyNode readBodyNode(
  tinyxml2::XMLElement* _bodyNodeElement,
  const Eigen::Isometry3d& _skeletonFrame,
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever)
{
  assert(_bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dart::dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = dart::utils::getAttributeString(_bodyNodeElement, "name");

  // transformation
  if (dart::utils::hasElement(_bodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        dart::utils::getValueIsometry3d(_bodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
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
      dtwarn << "[KinBodyParser::readAspects] KinBody file '"
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
    tinyxml2::XMLElement* geomEle,
    const std::string& renderOrData,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  auto fileNameAndScale
      = split(dart::utils::getValueString(geomEle, renderOrData), " ");

  if (fileNameAndScale.empty())
  {
    // TODO(JS): error!
  }

  auto fileName = fileNameAndScale[0];

  auto uniScale = 1.0;
  if (fileNameAndScale.size() > 1u)
    uniScale = std::atof(fileNameAndScale[1].c_str());
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
    assert(0);
  }
}

//==============================================================================
void readGeom(
    tinyxml2::XMLElement* geomEle,
    dart::dynamics::BodyNode* bodyNode,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  // ShapeNode name
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

  auto attribType = dart::utils::getAttributeString(geomEle, "type");

  if (attribType == "box")
  {
    auto extents = dart::utils::getValueVector3d(geomEle, "extents");
    auto shape = std::make_shared<dart::dynamics::BoxShape>(extents);

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
  else if (attribType == "sphere")
  {
    auto radius = dart::utils::getValueDouble(geomEle, "radius");
    auto shape = std::make_shared<dart::dynamics::SphereShape>(radius);

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

    bodyNode->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
  }
  else if(attribType == "trimesh")
  {
    const auto hasRender = dart::utils::hasElement(geomEle, "render");
    const auto hasData = dart::utils::hasElement(geomEle, "data");

    if (!(hasRender ^ hasData))
    {
      dterr << "[KinBodyParser] <Geom> doesn't contains none of <Render> or "
            << "<Data>, or contains both of them, which is invalid. Please "
            << "set only one of them.\n";
      assert(0);
    }

    if (hasRender)
    {
      auto shape = readMeshShape(geomEle, "render", baseUri, retriever);
      bodyNode->createShapeNodeWith<dart::dynamics::VisualAspect>(
          shape, shapeNodeName);
    }
    else if (hasData)
    {
      auto shape = readMeshShape(geomEle, "data", baseUri, retriever);
      bodyNode->createShapeNodeWith<
          dart::dynamics::CollisionAspect,
          dart::dynamics::DynamicsAspect>(shape, shapeNodeName);
    }
  }
  else
  {
    dterr << "[KinBodyParser] Attempts to parse unsupported geom type '"
          << attribType << "'.\n";
    assert(0);
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
void recurseElements(tinyxml2::XMLElement* ele)
{
  std::string eleName = ele->Name();
  ele->SetName(toLowerCases(eleName).c_str());

  if (ele->NoChildren())
    return;

  auto childEle = ele->FirstChildElement();
  while (childEle)
  {
    recurseElements(childEle);
    childEle = childEle->NextSiblingElement();
  }
}

//==============================================================================
tinyxml2::XMLElement* transformToLowerCases(tinyxml2::XMLDocument& doc)
{
  auto firstEle = doc.FirstChildElement();
  if (firstEle)
    recurseElements(firstEle);
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
}

} //anonymous namespace
} //namespace utils
} //namespace dart
