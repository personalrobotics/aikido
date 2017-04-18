#include <gtest/gtest.h>
#include <aikido/util/yaml.hpp>

//==============================================================================
TEST(YamlEigenExtension, LoadVectorMatrix3dIsometry3d)
{
  std::string yamlString
      = "closed: !Vector [2.443, 2.443, 2.443, 0.0]            \n"
        "relative_pose: !Matrix                                \n"
        "- [1.0, 0.0, 0.0]                                     \n"
        "- [0.0, 1.0, 0.0]                                     \n"
        "- [0.0, 0.0, 1.0]                                     \n"
        "absolute_pose: !Matrix                                \n"
        "- [1.0, 0.0, 0.0, 0.0]                                \n"
        "- [0.0, 1.0, 0.0, 0.0]                                \n"
        "- [0.0, 0.0, 1.0, 0.0]                                \n"
        "- [0.0, 0.0, 0.0, 1.0]                                \n";

  auto root = YAML::Load(yamlString);
  auto closed = root["closed"].as<Eigen::VectorXd>();
  auto relative_pose = root["relative_pose"].as<Eigen::Matrix3d>();
  auto absolute_pose = root["absolute_pose"].as<Eigen::Isometry3d>();

  EXPECT_TRUE(closed.isApprox(Eigen::Vector4d(2.443, 2.443, 2.443, 0.0)));
  EXPECT_TRUE(relative_pose.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(absolute_pose.matrix().isApprox(
      Eigen::Isometry3d::Identity().matrix()));
}
