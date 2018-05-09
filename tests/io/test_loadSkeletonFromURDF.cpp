#include <dart/utils/utils.hpp>
#include <gtest/gtest.h>
#include <aikido/io/util.hpp>
#include "eigen_tests.hpp"

#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

using namespace aikido::io;

static constexpr double EPS = 1e-6;

static std::string TEST_RESOURCES_PATH = STR(AIKIDO_TEST_RESOURCES_PATH);

//==============================================================================
TEST(LoadURDF, LoadBox)
{
  auto uri = std::string("file://") + TEST_RESOURCES_PATH
             + std::string("/urdf/objects/block.urdf");
  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

  EXPECT_TRUE(retriever->exists(uri));

  auto skel = loadSkeletonFromURDF(retriever, uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(1u, skel->getNumBodyNodes());
  EXPECT_EQ(1u, skel->getNumJoints());

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  // Two ShapeNodes: visual and collision
  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_EQ(2u, shapeNodes.size());

  for (std::size_t i = 0; i < bodyNode->getNumShapeNodes(); ++i)
  {
    auto shape = bodyNode->getShapeNode(i)->getShape();

    EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

    auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
    EXPECT_EIGEN_EQUAL(
        boxShape->getSize(), Eigen::Vector3d(0.0254, 0.0254, 0.0254), EPS);
  }

  auto joint = dynamic_cast<dart::dynamics::FreeJoint*>(skel->getRootJoint());
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(LoadURDF, LoadBoxWithTransform)
{
  auto uri = std::string("file://") + TEST_RESOURCES_PATH
             + std::string("/urdf/objects/block.urdf");
  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

  EXPECT_TRUE(retriever->exists(uri));

  auto tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = 1.0;
  tf.translation()[1] = 1.0;

  auto skel = loadSkeletonFromURDF(retriever, uri, tf);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(1u, skel->getNumBodyNodes());
  EXPECT_EQ(1u, skel->getNumJoints());

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  // Two ShapeNodes: visual and collision
  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_EQ(2u, shapeNodes.size());

  for (std::size_t i = 0; i < bodyNode->getNumShapeNodes(); ++i)
  {
    auto shape = bodyNode->getShapeNode(i)->getShape();

    EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

    auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
    EXPECT_EIGEN_EQUAL(
        boxShape->getSize(), Eigen::Vector3d(0.0254, 0.0254, 0.0254), EPS);
  }

  auto joint = dynamic_cast<dart::dynamics::FreeJoint*>(skel->getRootJoint());
  EXPECT_TRUE(joint != nullptr);

  EXPECT_EIGEN_EQUAL(
      tf.matrix(),
      joint->getChildBodyNode()->getWorldTransform().matrix(),
      EPS);
}

//==============================================================================
TEST(LoadURDF, LoadGround)
{
  auto uri = std::string("file://") + TEST_RESOURCES_PATH
             + std::string("/urdf/objects/ground.urdf");
  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

  EXPECT_TRUE(retriever->exists(uri));

  auto skel = loadSkeletonFromURDF(retriever, uri);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(1u, skel->getNumBodyNodes());
  EXPECT_EQ(1u, skel->getNumJoints());

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  // Two ShapeNodes: visual and collision
  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_EQ(2u, shapeNodes.size());

  for (std::size_t i = 0; i < bodyNode->getNumShapeNodes(); ++i)
  {
    auto shape = bodyNode->getShapeNode(i)->getShape();

    EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

    auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
    EXPECT_EIGEN_EQUAL(
        boxShape->getSize(), Eigen::Vector3d(50.0, 50.0, 0.05), EPS);
  }

  auto joint = dynamic_cast<dart::dynamics::WeldJoint*>(skel->getRootJoint());
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(LoadURDF, LoadGroundWithTransform)
{
  auto uri = std::string("file://") + TEST_RESOURCES_PATH
             + std::string("/urdf/objects/ground.urdf");
  auto retriever = std::make_shared<dart::common::LocalResourceRetriever>();

  EXPECT_TRUE(retriever->exists(uri));

  auto tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = 1.0;
  tf.translation()[1] = 1.0;

  EXPECT_THROW(loadSkeletonFromURDF(retriever, uri, tf), std::runtime_error);
}
