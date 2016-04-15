#include <aikido/distance/WeightedDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(WeightedDistance, StateSpaceEquality)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  WeightedDistanceMetric dmetric(
      space, {std::make_shared<AngularDistanceMetric>(so2),
              std::make_shared<EuclideanDistanceMetric>(rv3),
              std::make_shared<GeodesicDistanceMetric>(so3)});
  EXPECT_EQ(space, dmetric.getStateSpace());
}

TEST(WeightedDistance, Distance)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  WeightedDistanceMetric dmetric(
      space, {std::make_shared<AngularDistanceMetric>(so2),
              std::make_shared<EuclideanDistanceMetric>(rv3),
              std::make_shared<GeodesicDistanceMetric>(so3)});

  auto state1 = space->createState();
  auto state2 = space->createState();

  double angle1 = M_PI;
  double angle2 = 0.5 + M_PI;
  auto rv1 = Eigen::Vector3d(3, 4, 5);
  auto rv2 = Eigen::Vector3d(1, 2, 3);
  auto quat1 =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  auto quat2 = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI - 0.5, Eigen::Vector3d::UnitZ()));

  state1.getSubStateHandle<SO2StateSpace>(0).setAngle(angle1);
  state1.getSubStateHandle<RealVectorStateSpace>(1).setValue(rv1);
  state1.getSubStateHandle<SO3StateSpace>(2).setQuaternion(quat1);

  state2.getSubStateHandle<SO2StateSpace>(0).setAngle(angle2);
  state2.getSubStateHandle<RealVectorStateSpace>(1).setValue(rv2);
  state2.getSubStateHandle<SO3StateSpace>(2).setQuaternion(quat2);

  auto vdiff = Eigen::Vector3d(2, 2, 2);
  EXPECT_DOUBLE_EQ(0.5 + 0.5 + vdiff.norm(), dmetric.distance(state1, state2));
}
