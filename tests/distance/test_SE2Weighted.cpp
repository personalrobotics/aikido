#include <aikido/distance/SE2Weighted.hpp>
#include <aikido/statespace/SE2.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(SE2WeightedDistance, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(SE2Weighted(nullptr), std::invalid_argument);

  Eigen::Vector2d empty;
  EXPECT_THROW(SE2Weighted(nullptr, empty), std::invalid_argument);
}

TEST(SE2WeightedDistance, ThrowsOnNegativeWeights)
{
  auto se2 = std::make_shared<SE2>();
  Eigen::Vector2d weights(-1.0, 2.0);

  EXPECT_THROW(SE2Weighted(se2, weights), std::invalid_argument);
}

TEST(SE2WeightedDistance, StateSpaceEquality)
{
  auto se2 = std::make_shared<SE2>();
  Eigen::Vector2d weights(1.0, 2.0);  

  SE2Weighted dmetric(se2);
  SE2Weighted dmetric2(se2, weights);

  EXPECT_EQ(se2, dmetric.getStateSpace());
  EXPECT_EQ(se2, dmetric2.getStateSpace());
}

TEST(SE2WeightedDistance, DistanceUnitWeights)
{
  auto se2 = std::make_shared<SE2>();
  SE2Weighted dmetric(se2);

  auto state1 = se2->createState();
  auto state2 = se2->createState();
  
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d pose2 = Eigen::Isometry2d::Identity();

  Eigen::Rotation2Dd rotation1(0);
  Eigen::Vector2d translation1(1, 2);
  pose1.translate(translation1);
  pose1.rotate(rotation1);


  Eigen::Rotation2Dd rotation2(M_PI_4);
  Eigen::Vector2d translation2(4, 3);
  pose2.translate(translation2);
  pose2.rotate(rotation2);

  state1.setIsometry(pose1);
  state2.setIsometry(pose2);

  Eigen::Vector2d vdiff = translation1 - translation2;
  
  EXPECT_DOUBLE_EQ(M_PI_4 + vdiff.norm(), dmetric.distance(state1, state2));
}

TEST(SE2WeightedDistance, DistanceCustomWeights)
{
  auto se2 = std::make_shared<SE2>();
  Eigen::Vector2d weights(1.0, 2.0);
  SE2Weighted dmetric(se2, weights);

  auto state1 = se2->createState();
  auto state2 = se2->createState();
  
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d pose2 = Eigen::Isometry2d::Identity();

  Eigen::Rotation2Dd rotation1(0);
  Eigen::Vector2d translation1(1, 2);
  pose1.translate(translation1);
  pose1.rotate(rotation1);


  Eigen::Rotation2Dd rotation2(M_PI_4);
  Eigen::Vector2d translation2(4, 3);
  pose2.translate(translation2);
  pose2.rotate(rotation2);

  state1.setIsometry(pose1);
  state2.setIsometry(pose2);

  Eigen::Vector2d vdiff = translation1 - translation2;
  
  EXPECT_DOUBLE_EQ(1*M_PI_4 + 2*(vdiff.norm()), dmetric.distance(state1, state2));
}
