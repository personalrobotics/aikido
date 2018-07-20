#include <aikido/statespace/DubinsInterpolator.hpp>
#include <aikido/statespace/SE2.hpp>

#include <gtest/gtest.h>

#define PRINT_VALUE(exp) std::cout << "L" << __LINE__ << ": " << #exp << " = " << exp << std::endl;

using namespace aikido::statespace;

TEST(Dubins, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(DubinsInterpolator(nullptr), std::invalid_argument);
}

TEST(Dubins, StateSpaceEquality)
{
  auto se2 = std::make_shared<SE2>();
  DubinsInterpolator interpolator(se2);

  EXPECT_EQ(se2, interpolator.getStateSpace());
}

TEST(Dubins, Interpolation)
{
  auto se2 = std::make_shared<SE2>();
  DubinsInterpolator interpolator(se2, 1.0);

  auto state1 = se2->createState();
  auto state2 = se2->createState();
  auto stateOut = se2->createState();
  
  class TestVector{
  public:
    TestVector(double x1, double y1, double yaw1,
               double x2, double y2, double yaw2,
               double alpha,
               double expectedValueX, double expectedValueY, double expectedValueYaw,
               double precision = 1E-12)
              : mAlpha(alpha)
              , mExpectedValueX(expectedValueX)
              , mExpectedValueY(expectedValueY)
              , mExpectedValueYaw(expectedValueYaw)
              , mPrecision(precision)
    {
        mS1Isometry = Eigen::Isometry2d::Identity();     
        mS1Isometry.rotate(Eigen::Rotation2Dd(yaw1));
        mS1Isometry.pretranslate(Eigen::Vector2d(x1, y1));

        mS2Isometry = Eigen::Isometry2d::Identity();
        mS2Isometry.rotate(Eigen::Rotation2Dd(yaw2));
        mS2Isometry.pretranslate(Eigen::Vector2d(x2, y2));
    }
    Eigen::Isometry2d mS1Isometry;
    Eigen::Isometry2d mS2Isometry;
    double mAlpha;
    double mExpectedValueX;
    double mExpectedValueY;
    double mExpectedValueYaw;
    double mPrecision;
  };

  Eigen::Isometry2d s1Isometry;
  Eigen::Isometry2d s2Isometry;
  Eigen::Isometry2d sOutIsometry;

  std::vector<TestVector> testVectors;
  testVectors.reserve(20);

  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                   2., 0., -M_PI_2, 1.0,
                                   2., 0., -M_PI_2));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                   2., 0., -M_PI_2, 0.5,
                                   1., 1., 0.));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                   2., 0., -M_PI_2, 0.25,
                                   1. - 1./sqrt(2), 1./sqrt(2), M_PI_4));                           
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                   3., 0., -M_PI_2, 1.,
                                   3., 0., -M_PI_2));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                   3., 2., M_PI_2, 1.0,
                                   3., 2., M_PI_2));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                  -2., 0., -M_PI_2, 1.0,
                                  -2., 0., -M_PI_2));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                  -3., 0., -M_PI_2, 1.0,
                                  -3., 0., -M_PI_2));
  testVectors.push_back(TestVector(0., 0., M_PI_2,
                                  -3., 2., M_PI_2, 1.0,
                                  -3., 2., M_PI_2));
  
  for(auto& testVector : testVectors)
  {
    Eigen::Rotation2Dd rotationExpected = Eigen::Rotation2Dd::Identity();

    state1.setIsometry(testVector.mS1Isometry);
    state2.setIsometry(testVector.mS2Isometry);
    
    interpolator.interpolate(state1, state2, testVector.mAlpha, stateOut);

    sOutIsometry = stateOut.getIsometry();
    rotationExpected.fromRotationMatrix(sOutIsometry.rotation());

    EXPECT_NEAR(testVector.mExpectedValueX, sOutIsometry.translation()[0], testVector.mPrecision);
    EXPECT_NEAR(testVector.mExpectedValueY, sOutIsometry.translation()[1], testVector.mPrecision);
    EXPECT_NEAR(testVector.mExpectedValueYaw, rotationExpected.angle(), testVector.mPrecision);
  }
}
