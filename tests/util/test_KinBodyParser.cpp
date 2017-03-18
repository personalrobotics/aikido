#include <gtest/gtest.h>
#include <aikido/util/KinBodyParser.hpp>

using namespace aikido::util;

//==============================================================================
TEST(KinBodyParser, LoadBoxGeom)
{
  const std::string str =
      "<KinBody name=\"paper_box\">                \n"
      "  <Body type=\"static\" name=\"paper_box\"> \n"
      "    <Geom type=\"box\">                     \n"
      "      <Translation>0 0 0</Translation>      \n"
      "      <Extents>0.1 0.145 0.055</Extents>    \n"
      "      <DiffuseColor>0 0 1.0</DiffuseColor>  \n"
      "    </Geom>                                 \n"
      "  </Body>                                   \n"
      "</KinBody>                                    ";

  auto skel = KinBodyParser::readSkeletonXML(str);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_TRUE(skel->getNumBodyNodes() == 1u);
  EXPECT_TRUE(skel->getNumJoints() == 1u);

  auto bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);

  auto shapeNodes = bodyNode->getShapeNodes();
  EXPECT_TRUE(shapeNodes.size() == 1u);

  auto shapeNode = shapeNodes[0];
  EXPECT_TRUE(shapeNode != nullptr);

  auto shape = shapeNode->getShape();
  EXPECT_TRUE(shape->is<dart::dynamics::BoxShape>());

  auto boxShape = static_cast<dart::dynamics::BoxShape*>(shape.get());
  EXPECT_TRUE(boxShape->getSize().isApprox(Eigen::Vector3d(0.1, 0.145, 0.055)));

  auto joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
}

//==============================================================================
TEST(KinBodyParser, LoadTriMeshGeom)
{
  // TODO(JS)
}
