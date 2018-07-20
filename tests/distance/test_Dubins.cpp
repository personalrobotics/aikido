#include <aikido/distance/Dubins.hpp>
#include <aikido/statespace/SE2.hpp>

#include <gtest/gtest.h>

#define PRINT_VALUE(exp) std::cout << "L" << __LINE__ << ": " << #exp << " = " << exp << std::endl;

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(Dubins, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(Dubins(nullptr), std::invalid_argument);
}

TEST(Dubins, StateSpaceEquality)
{
  auto se2 = std::make_shared<SE2>();
  Dubins dmetric(se2);

  EXPECT_EQ(se2, dmetric.getStateSpace());
}

TEST(Dubins, Distance)
{
  auto se2 = std::make_shared<SE2>();
  Dubins dmetric(se2, 1.0);

  auto state1 = se2->createState();
  auto state2 = se2->createState();
  
  class TestVector{
  public:
    TestVector(double x1, double y1, double yaw1,
               double x2, double y2, double yaw2,
               double expectedValue, double precision = 1E-12)
              : mExpectedValue(expectedValue)
              , mPrecision(precision)
    {
        mS1Isometry = Eigen::Isometry2d::Identity();     
        mS1Isometry.rotate(Eigen::Rotation2Dd(yaw1));
        mS1Isometry.pretranslate(Eigen::Vector2d(x1, y1));

        mS2Isometry = Eigen::Isometry2d::Identity();
        mS2Isometry.rotate(Eigen::Rotation2Dd(yaw2));
        mS2Isometry.pretranslate(Eigen::Vector2d(x2, y2));
    }
    double mExpectedValue;
    double mPrecision;
    Eigen::Isometry2d mS1Isometry;
    Eigen::Isometry2d mS2Isometry;
  };

  Eigen::Isometry2d s1Isometry;
  Eigen::Isometry2d s2Isometry;

  std::vector<TestVector> testVectors;
  testVectors.reserve(20);

  testVectors.push_back(TestVector(0., 0., M_PI_2, 2., 0., -M_PI_2, M_PI));
  testVectors.push_back(TestVector(0., 0., M_PI_2, 3., 0., -M_PI_2, M_PI + 1.0));
  testVectors.push_back(TestVector(0., 0., M_PI_2, 3., 2., M_PI_2, M_PI + 1.0));
  testVectors.push_back(TestVector(0., 0., M_PI_2, -2., 0., -M_PI_2, M_PI));
  testVectors.push_back(TestVector(0., 0., M_PI_2, -3., 0., -M_PI_2, M_PI + 1.0));
  testVectors.push_back(TestVector(0., 0., M_PI_2, -3., 2., M_PI_2, M_PI + 1.0));  
  testVectors.push_back(TestVector(4.0579, 2.6552, 5.5396, 3.6489, 2.7603, 0.5892, 5.9110 , 1E-4));
  testVectors.push_back(TestVector(3.6489, 2.7603, 0.5892, 3.9223, 1.1900, 2.6511, 4.4585, 1E-4));
  testVectors.push_back(TestVector(3.9223, 1.1900, 2.6511, 3.1338, 1.8687, 2.7337, 1.0529, 1E-4));
  testVectors.push_back(TestVector(3.1338, 1.8687, 2.7337, 1.1574, 1.8527, 2.8227, 2.0023, 1E-4));
  testVectors.push_back(TestVector(1.1574, 1.8527, 2.8227, 0.2346, 1.9660, 3.47468, 0.9582, 1E-4));
  testVectors.push_back(TestVector(0.2346, 1.9660, 3.47468, 0.3445, 3.5166, 0.5435, 5.5117, 1E-4));
  testVectors.push_back(TestVector(0.3445, 3.5166, 0.5435, 0.9754, 4.4559, 0.8874, 1.1556, 1E-4));
  testVectors.push_back(TestVector(0.9754, 4.4559, 0.8874, 2.5774, 4.4722, 5.5635, 1.8043, 1E-4));
  testVectors.push_back(TestVector(2.5774, 4.4722, 5.5635, 3.5949, 3.5127, 5.3026, 1.4007, 1E-4));

  
  for(auto& testVector : testVectors)
  {
    state1.setIsometry(testVector.mS1Isometry);
    state2.setIsometry(testVector.mS2Isometry);

    EXPECT_NEAR(testVector.mExpectedValue, dmetric.distance(state1, state2), testVector.mPrecision);
  }

}
