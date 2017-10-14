#include <gtest/gtest.h>
#include <aikido/io/yaml.hpp>

//==============================================================================
TEST(YamlEigenExtension, Validity)
{
  using Vector1d = Eigen::Matrix<double, 1, 1>;
  using Matrix1d = Eigen::Matrix<double, 1, 1>;

  std::string empty = "empty:";
  EXPECT_THROW(
      YAML::Load(empty)["empty"].as<Eigen::VectorXd>(),
      YAML::RepresentationException);

  std::string map = "map: {left: ok, right: bad}";
  EXPECT_THROW(
      YAML::Load(map)["map"].as<Eigen::VectorXd>(),
      YAML::RepresentationException);

  std::string zeroSizedVector = "vector: []";
  Eigen::VectorXd zeroVec;
  EXPECT_NO_THROW(
      zeroVec = YAML::Load(zeroSizedVector)["vector"].as<Eigen::VectorXd>());
  EXPECT_EQ(zeroVec.size(), 0);

  std::string zeroSizedMatrix1 = "matrix: [[]]";
  Eigen::MatrixXd zeroMat1;
  EXPECT_NO_THROW(
      zeroMat1 = YAML::Load(zeroSizedMatrix1)["matrix"].as<Eigen::MatrixXd>());
  EXPECT_EQ(zeroMat1.rows(), 0);
  EXPECT_EQ(zeroMat1.cols(), 0);

  std::string zeroSizedMatrix2 = "matrix: [[], []]";
  Eigen::MatrixXd zeroMat2;
  EXPECT_NO_THROW(
      zeroMat2 = YAML::Load(zeroSizedMatrix2)["matrix"].as<Eigen::MatrixXd>());
  EXPECT_EQ(zeroMat2.rows(), 0);
  EXPECT_EQ(zeroMat2.cols(), 0);

  std::string vector1 = "vector: [1]";
  EXPECT_NO_THROW(YAML::Load(vector1)["vector"].as<Vector1d>());

  std::string matrix1 = "matrix: [[1]]";
  EXPECT_NO_THROW(YAML::Load(matrix1)["matrix"].as<Matrix1d>());

  std::string incorrectVectorSize = "vector: [1, 2, 3]";
  EXPECT_THROW(
      YAML::Load(incorrectVectorSize)["vector"].as<Eigen::Vector4d>(),
      YAML::RepresentationException);

  std::string incorrectMatrixRowSize = "matrix: [[1, 2, 3], [4, 5, 6]]";
  EXPECT_THROW(
      YAML::Load(incorrectMatrixRowSize)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string incorrectMatrixColSize = "matrix: [[1, 2], [3, 4], [5, 6]]";
  EXPECT_THROW(
      YAML::Load(incorrectMatrixColSize)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string incorrectMatrixSize1 = "matrix: [[1, 2, 3], [1, 2, 3], [1, 2]]";
  EXPECT_THROW(
      YAML::Load(incorrectMatrixSize1)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string incorrectMatrixSize2 = "matrix: [1, 2, 3]";
  EXPECT_THROW(
      YAML::Load(incorrectMatrixSize2)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string inconsistentMatrixRowSize1
      = "matrix: [[1, 2, 3], [4, 5, 6], [7, 8]]";
  EXPECT_THROW(
      YAML::Load(inconsistentMatrixRowSize1)["matrix"].as<Eigen::MatrixXd>(),
      YAML::RepresentationException);

  std::string inconsistentMatrixRowSize2 = "matrix: [[], [1], [2]]";
  EXPECT_THROW(
      YAML::Load(inconsistentMatrixRowSize2)["matrix"].as<Eigen::MatrixXd>(),
      YAML::RepresentationException);

  std::string notSequenceInVector = "vector: 1";
  EXPECT_THROW(
      YAML::Load(notSequenceInVector)["vector"].as<Vector1d>(),
      YAML::RepresentationException);

  std::string notSequenceInMatrix1 = "matrix: [1, 2, 3]";
  EXPECT_THROW(
      YAML::Load(notSequenceInMatrix1)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string notSequenceInMatrix2 = "matrix: [[1, 2, 3], [1, 2, 3], 1]";
  EXPECT_THROW(
      YAML::Load(notSequenceInMatrix2)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string notScalarElementInVector1 = "vector: [1, 2, [3]]";
  EXPECT_THROW(
      YAML::Load(notScalarElementInVector1)["vector"].as<Eigen::Vector3d>(),
      YAML::RepresentationException);

  std::string notScalarElementInVector2 = "vector: [1, 2, []]";
  EXPECT_THROW(
      YAML::Load(notScalarElementInVector2)["vector"].as<Eigen::Vector3d>(),
      YAML::RepresentationException);

  std::string notScalarElementInMatrix1
      = "matrix: [[1, 2, 3], [[1, 2], 2, 3], [1, 2, 3]]";
  EXPECT_THROW(
      YAML::Load(notScalarElementInMatrix1)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);

  std::string notScalarElementInMatrix2
      = "matrix: [[1, 2, 3], [[], 2, 3], [1, 2, 3]]";
  EXPECT_THROW(
      YAML::Load(notScalarElementInMatrix2)["matrix"].as<Eigen::Matrix3d>(),
      YAML::RepresentationException);
}

//==============================================================================
TEST(YamlEigenExtension, LoadVectorMatrix3dIsometry3d)
{
  std::string yamlString
      = "closed: [2.443, 2.443, 2.443, 0.0] \n"
        "relative_pose:                     \n"
        "- [1.0, 0.0, 0.0]                  \n"
        "- [0.0, 1.0, 0.0]                  \n"
        "- [0.0, 0.0, 1.0]                  \n"
        "absolute_pose:                     \n"
        "- [1.0, 0.0, 0.0, 0.0]             \n"
        "- [0.0, 1.0, 0.0, 0.0]             \n"
        "- [0.0, 0.0, 1.0, 0.0]             \n"
        "- [0.0, 0.0, 0.0, 1.0]             \n";

  auto root = YAML::Load(yamlString);
  auto closed = root["closed"].as<Eigen::VectorXd>();
  auto relative_pose = root["relative_pose"].as<Eigen::Matrix3d>();
  auto absolute_pose = root["absolute_pose"].as<Eigen::Isometry3d>();

  EXPECT_TRUE(closed.isApprox(Eigen::Vector4d(2.443, 2.443, 2.443, 0.0)));
  EXPECT_TRUE(relative_pose.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(
      absolute_pose.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()));

  EXPECT_TRUE(YAML::Node(closed).as<Eigen::VectorXd>() == closed);
  EXPECT_TRUE(YAML::Node(relative_pose).as<Eigen::Matrix3d>() == relative_pose);
  EXPECT_TRUE(
      YAML::Node(absolute_pose)
          .as<Eigen::Isometry3d>()
          .matrix()
          .isApprox(absolute_pose.matrix()));
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
  using TransformMap = std::
      unordered_map<std::string,
                    Eigen::Isometry3d,
                    std::hash<std::string>,
                    std::equal_to<std::string>,
                    Eigen::aligned_allocator<std::pair<const std::string,
                                                       Eigen::Isometry3d>>>;

  using TransformsMap = std::unordered_map<std::string, TransformMap>;

  std::string yamlString
      = "---                                              \n"
        "# BH280 endeffector frame to palm offset = 0.18 m\n"
        "left:                                            \n"
        "  default: &left_default                         \n"
        "    [[0., 0., 1., 0.],                           \n"
        "     [1., 0., 0., 0.],                           \n"
        "     [0., 1., 0., .18],                          \n"
        "     [0., 0., 0., 1.]]                           \n"
        "  cylinder: *left_default                        \n"
        "# BH282 endeffector frame to palm offset = ?     \n"
        "right:                                           \n"
        "  default: &right_default                        \n"
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
  expectedLeftDefault.linear() << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  expectedLeftDefault.translation() << 0.0, 0.0, 0.18;
  EXPECT_TRUE(left_default.matrix().isApprox(expectedLeftDefault.matrix()));

  auto left_cylinder = left["cylinder"];
  EXPECT_TRUE(left_cylinder.matrix().isApprox(expectedLeftDefault.matrix()));

  auto right = map["right"];
  EXPECT_TRUE(left.size() == 2u);

  auto right_default = right["default"];
  Eigen::Isometry3d expectedRightDefault = Eigen::Isometry3d::Identity();
  expectedRightDefault.linear() << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  expectedRightDefault.translation() << 0.0, 0.0, 0.18;
  EXPECT_TRUE(right_default.matrix().isApprox(expectedRightDefault.matrix()));

  auto right_cylinder = left["cylinder"];
  EXPECT_TRUE(right_cylinder.matrix().isApprox(expectedRightDefault.matrix()));
}

//==============================================================================
TEST(YamlEigenExtension, AprilTagInJSON)
{
  std::string jsonString
      = "{                                     \n"
        "  \"tag124\": {                       \n"
        "    \"name\": \"test_object\",        \n"
        "    \"package://pr_ordata/test.urdf\",\n"
        "    \"offset\":                       \n"
        "      [[ 1, 0, 0, 1 ],                \n"
        "       [ 0, 1, 0, 2 ],                \n"
        "       [ 0, 0, 1, 3 ],                \n"
        "       [ 0, 0, 0, 1 ]]                \n"
        "  }                                   \n"
        "}                                     \n";

  auto root = YAML::Load(jsonString);

  auto tag124 = root["tag124"]["offset"].as<Eigen::Isometry3d>();
  Eigen::Isometry3d expectedTag124 = Eigen::Isometry3d::Identity();
  expectedTag124.translation() << 1, 2, 3;
  EXPECT_TRUE(tag124.matrix().isApprox(expectedTag124.matrix()));
}
