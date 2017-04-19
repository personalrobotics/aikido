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

  EXPECT_TRUE(YAML::Node(closed).as<Eigen::VectorXd>() == closed);
  EXPECT_TRUE(YAML::Node(relative_pose).as<Eigen::Matrix3d>() == relative_pose);
  EXPECT_TRUE(
      YAML::Node(absolute_pose).as<Eigen::Isometry3d>().matrix().isApprox(
          absolute_pose.matrix()));
}

//==============================================================================
TEST(YamlEigenExtension, UnorderedMap)
{
  using UnorderedMap = std::unordered_map<std::string, std::string>;

  std::string yamlString
      = "key_map:        \n"
        "    key1: value1\n"
        "    key2: value2\n";

  auto root = YAML::Load(yamlString);
  auto key_map = root["key_map"];
  auto map = key_map.as<UnorderedMap>();
  EXPECT_TRUE(map.size() == 2u);
  EXPECT_TRUE(map["key1"] == "value1");
  EXPECT_TRUE(map["key2"] == "value2");

  EXPECT_TRUE(YAML::Node(map).as<UnorderedMap>() == map);
}
