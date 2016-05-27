#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "dart/config.h"
#include "dart/common/Console.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/simulation/World.h"
#include "dart/utils/XmlHelpers.h"
#include "dart/common/LocalResourceRetriever.h"
#include "dart/common/Uri.h"

#include "aikido/util/KinBodyParser.h"

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
  tinyxml2::XMLElement* _KinBodyElement,
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readBodyNode(
  tinyxml2::XMLElement* _bodyNodeElement,
  const Eigen::Isometry3d& _skeletonFrame,
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever);

//Similar to SkelParser::readShape
dart::dynamics::ShapePtr readShape(
  tinyxml2::XMLElement* vizOrColEle,
  const std::string& fieldName, //Either Render or Data for KinBody - may need to make more general
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever);

dart::dynamics::ShapeNode* readShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readVisualizationShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);


void readAddons(
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* _KinBodyElement,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever);

dart::common::ResourceRetrieverPtr getRetriever(
  const dart::common::ResourceRetrieverPtr& _retriever);

} //anonymous namespace


//==============================================================================
dart::dynamics::SkeletonPtr KinBodyParser::readKinBodyXMLFile(
  const dart::common::Uri& _fileUri,
  const dart::common::ResourceRetrieverPtr& _retriever)
{
  const dart::common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _kinBodyFile;
  try
  {
    dart::utils::openXMLFile(_kinBodyFile, _fileUri, _retriever);
  }
  catch(std::exception const& e)
  {
    dterr << "[KinBodyParser] LoadFile [" << _fileUri.toString() << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  tinyxml2::XMLElement* kinBodyElement = nullptr;
  kinBodyElement = _kinBodyFile.FirstChildElement("KinBody");
  if (kinBodyElement == nullptr)
  {
    dterr << "[KinBodyParser] KinBody  file[" << _fileUri.toString()
          << "] does not contain <KinBody> as the element.\n";
    return nullptr;
  }

  dart::dynamics::SkeletonPtr newSkeleton = readKinBody(
    kinBodyElement, _fileUri, retriever);

  return newSkeleton;
}

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr readKinBody(
  tinyxml2::XMLElement* _KinBodyElement,
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever)
{

  assert(_KinBodyElement != nullptr);

  dart::dynamics::SkeletonPtr newSkeleton = dart::dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = dart::utils::getAttributeString(_KinBodyElement, "name");
  newSkeleton->setName(name);

  //Get Body node
  tinyxml2::XMLElement* bodyElement = nullptr;
  bodyElement = _KinBodyElement->FirstChildElement("Body");
  if (bodyElement == nullptr)
  {
    dterr << "[KinBodyParser] KinBody file[" << _baseUri.toString()
          << "] does not contain <Body> element "
          <<"under <KinBody> element.\n";
    return nullptr;
  }

  SkelBodyNode newBodyNode = readBodyNode(
    bodyElement, skeletonFrame,_baseUri, _retriever);

  //How to add newBodyNode to new Skeleton?
  //Also need to add some static joint I guess
  
  //Add a free joint
  SkelJoint rootJoint;
  rootJoint.properties = 
      Eigen::make_aligned_shared<dart::dynamics::FreeJoint::Properties>(
            dart::dynamics::Joint::Properties("root_joint", newBodyNode.initTransform));
  rootJoint.type = "free";

  std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> jbn_pair;
  jbn_pair =  newSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint, dart::dynamics::BodyNode>(nullptr,
      static_cast<const dart::dynamics::FreeJoint::Properties&>(*rootJoint.properties),
      static_cast<const dart::dynamics::BodyNode::Properties&>(*newBodyNode.properties));


  assert(jbn_pair.first != nullptr && jbn_pair.second != nullptr);


  readAddons(newSkeleton, _KinBodyElement, _baseUri, _retriever);
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
void readAddons(
    const dart::dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* _KinBodyElement,
    const dart::common::Uri& _baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::utils::ElementEnumerator xmlBodies(_KinBodyElement, "Body");
  while (xmlBodies.next())
  {
    auto bodyElement = xmlBodies.get();
    auto bodyNodeName = dart::utils::getAttributeString(bodyElement, "name");
    auto bodyNode = skeleton->getBodyNode(bodyNodeName);

    //Assume either 1 or 2 Geom elements
    tinyxml2::XMLElement* geomElement = nullptr;
    geomElement = bodyElement->FirstChildElement("Geom");
    if (geomElement == nullptr)
    {
      dterr << "[KinBodyParser] KinBody file[" << _baseUri.toString()
          << "] does not contain <Geom> element "
          <<"under <Body>, under <KinBody> element.\n";
      assert(0);
    }

    int element_count = 0;
    if (dart::utils::hasElement(geomElement,"Render"))
    {
      element_count ++;
      readVisualizationShapeNode(bodyNode, geomElement, _baseUri, retriever);
    }
    
    if (dart::utils::hasElement(geomElement,"Data"))
    {
      element_count ++;
      readCollisionShapeNode(bodyNode, geomElement, _baseUri, retriever);
    }

    if(element_count == 0){
      //Neither Render nor Data
      dterr <<"[KinBodyParser] Geom Element in "<< _baseUri.toString()<<" has neither Render nor Data \n";
      assert(0);
    }
    else if(element_count == 1){
      //One done not the other
      tinyxml2::XMLElement* geomElementSibling = nullptr;
      geomElementSibling = geomElement->NextSiblingElement("Geom");
      if (geomElementSibling == nullptr)
      {
        dterr<<"[KinBodyParser] Only one Geom element in "<<_baseUri.toString()<<" which does not have both Render and Data \n";
        assert(0);
      }

      if(dart::utils::hasElement(geomElementSibling,"Render")){
        readVisualizationShapeNode(bodyNode, geomElementSibling, _baseUri, retriever);
      }
      if(dart::utils::hasElement(geomElementSibling,"Data")){
        readCollisionShapeNode(bodyNode, geomElementSibling, _baseUri, retriever);
      }

    }
  }
}

//==============================================================================
void readVisualizationShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::dynamics::ShapeNode* newShapeNode
      = readShapeNode(bodyNode, vizShapeNodeEle,
                      "Render",
                      baseUri, retriever);

  auto visualAddon = newShapeNode->getVisualAddon(true);
  visualAddon->setColor(Eigen::Vector3d(1,1,1));

  // color
  if (dart::utils::hasElement(vizShapeNodeEle, "color"))
  {
    Eigen::Vector3d color = dart::utils::getValueVector3d(vizShapeNodeEle, "color");
    visualAddon->setColor(color);
  }
}


//==============================================================================
void readCollisionShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::dynamics::ShapeNode* newShapeNode
      = readShapeNode(bodyNode, collShapeNodeEle,
                      "Data",
                      baseUri, retriever);

  auto collisionAddon = newShapeNode->getCollisionAddon(true);
  newShapeNode->createDynamicsAddon();

  // collidable
  if (dart::utils::hasElement(collShapeNodeEle, "collidable"))
  {
    const bool collidable = dart::utils::getValueDouble(collShapeNodeEle, "collidable");
    collisionAddon->setCollidable(collidable);
  }
}

//==============================================================================
dart::dynamics::ShapeNode* readShapeNode(
    dart::dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
    const dart::common::Uri& baseUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  assert(bodyNode);

  auto shape = readShape(shapeNodeEle, shapeNodeName, baseUri, retriever);
  auto shapeNode = bodyNode->createShapeNode(shape, shapeNodeName);

  // Transformation
  if (dart::utils::hasElement(shapeNodeEle, "transformation"))
  {
    Eigen::Isometry3d W = dart::utils::getValueIsometry3d(shapeNodeEle, "transformation");
    shapeNode->setRelativeTransform(W);
  }

  return shapeNode;
}

//==============================================================================
dart::dynamics::ShapePtr readShape(
  tinyxml2::XMLElement* vizOrColEle,
  const std::string& fieldName, //Either Render or Data for KinBody - may need to make more general
  const dart::common::Uri& _baseUri,
  const dart::common::ResourceRetrieverPtr& _retriever)
{

  dart::dynamics::ShapePtr newShape;

  //Either viz(Render field) or col(Data field)
  if(dart::utils::hasElement(vizOrColEle,fieldName))
  {
    std::string filename = dart::utils::getValueString(vizOrColEle, fieldName);
    const std::string meshUri = dart::common::Uri::getRelativeUri(_baseUri, filename);
    const aiScene* model = dart::dynamics::MeshShape::loadMesh(meshUri, _retriever);
    const Eigen::Vector3d scale(1.0,1.0,1.0); //Default scale as kinbody does not have info
    if (model)
    {
      newShape = std::make_shared<dart::dynamics::MeshShape>(
          scale, model, meshUri, _retriever);
    }
    else
    {
      dterr << "[KinBodyParser] Fail to load model[" << filename << "]." << std::endl;
    }
  }
  else
  {
    //No field
    dterr << "[KinBodyParser] "<<fieldName<<" not present in Geom ";
    assert(0);
    return nullptr;
  }

  return newShape;
}


dart::common::ResourceRetrieverPtr getRetriever(
  const dart::common::ResourceRetrieverPtr& _retriever)
{
  if(_retriever)
    return _retriever;
  else
    return std::make_shared<dart::common::LocalResourceRetriever>();
}

} //anonymous namespace

} //namespace utils

} //namespace dart