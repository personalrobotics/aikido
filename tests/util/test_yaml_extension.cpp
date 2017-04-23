#include <gtest/gtest.h>
#include <aikido/util/yaml.hpp>

//==============================================================================
TEST(YamlEigenExtension, Errors)
{
  std::string notSequential = "name: PRL";
  EXPECT_THROW(YAML::Load(notSequential)["name"].as<Eigen::VectorXd>(),
      YAML::RepresentationException);

  std::string incorrectSize = "vector: [1, 2, 3]";
  EXPECT_THROW(YAML::Load(incorrectSize)["vector"].as<Eigen::Vector2d>(),
      YAML::RepresentationException);
}

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

//==============================================================================
TEST(YamlEigenExtension, TsrTransforms)
{
  using TransformMap = std::unordered_map<
      std::string,
      Eigen::Isometry3d,
      std::hash<std::string>,
      std::equal_to<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>
  >;

  using TransformsMap = std::unordered_map<std::string, TransformMap>;

  std::string yamlString
      = "---                                              \n"
        "# BH280 endeffector frame to palm offset = 0.18 m\n"
        "left:                                            \n"
        "  default: !Matrix &left_default                 \n"
        "    [[0., 0., 1., 0.],                           \n"
        "     [1., 0., 0., 0.],                           \n"
        "     [0., 1., 0., .18],                          \n"
        "     [0., 0., 0., 1.]]                           \n"
        "  cylinder: *left_default                        \n"
        "# BH282 endeffector frame to palm offset = ?     \n"
        "right:                                           \n"
        "  default: !Matrix &right_default                \n"
        "    [[0., 0., 1., 0.],                           \n"
        "     [1., 0., 0., 0.],                           \n"
        "     [0., 1., 0., .18],                          \n"
        "     [0., 0., 0., 1.]]                           \n"
        "  cylinder: *right_default                       \n";

  auto root = YAML::Load(yamlString);
  auto map = root.as<TransformsMap>();
  EXPECT_TRUE(map.size() == 2u);

  auto left = map["left"];
  EXPECT_TRUE(left.size() == 2u);

  auto left_default = left["default"];
  Eigen::Isometry3d expectedLeftDefault = Eigen::Isometry3d::Identity();
  expectedLeftDefault.linear()
      << 0.0, 0.0, 1.0,
         1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;
  expectedLeftDefault.translation() << 0.0, 0.0, 0.18;
  EXPECT_TRUE(left_default.matrix().isApprox(expectedLeftDefault.matrix()));

  auto left_cylinder = left["cylinder"];
  EXPECT_TRUE(left_cylinder.matrix().isApprox(expectedLeftDefault.matrix()));

  auto right = map["right"];
  EXPECT_TRUE(left.size() == 2u);

  auto right_default = right["default"];
  Eigen::Isometry3d expectedRightDefault = Eigen::Isometry3d::Identity();
  expectedRightDefault.linear()
      << 0.0, 0.0, 1.0,
         1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;
  expectedRightDefault.translation() << 0.0, 0.0, 0.18;
  EXPECT_TRUE(right_default.matrix().isApprox(expectedRightDefault.matrix()));

  auto right_cylinder = left["cylinder"];
  EXPECT_TRUE(right_cylinder.matrix().isApprox(expectedRightDefault.matrix()));
}
