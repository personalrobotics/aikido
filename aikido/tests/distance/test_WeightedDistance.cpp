#include <aikido/distance/WeightedDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(WeightedDistance, Distance)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();

  auto space = std::make_shared<CompoundStateSpace>({so2, rv3, so3});

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

TEST(CompoundStateSpace, Interpolate)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();

  auto space = std::make_shared<CompoundStateSpace>({so2, rv3, so3});

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

  auto state1_handle0 = state1.getSubStateHandle<SO2StateSpace>(0);
  auto state1_handle1 = state1.getSubStateHandle<RealVectorStateSpace>(1);
  auto state1_handle2 = state1.getSubStateHandle<SO3StateSpace>(2);

  auto state2_handle0 = state2.getSubStateHandle<SO2StateSpace>(0);
  auto state2_handle1 = state2.getSubStateHandle<RealVectorStateSpace>(1);
  auto state2_handle2 = state2.getSubStateHandle<SO3StateSpace>(2);

  state1_handle0.setAngle(angle1);
  state1_handle1.setValue(rv1);
  state1_handle2.setQuaternion(quat1);
  state2_handle0.setAngle(angle2);
  state2_handle1.setValue(rv2);
  state2_handle2.setQuaternion(quat2);

  auto istate = space->createState();
  auto istate_handle0 = istate.getSubStateHandle<SO2StateSpace>(0);
  auto istate_handle1 = istate.getSubStateHandle<RealVectorStateSpace>(1);
  auto istate_handle2 = istate.getSubStateHandle<SO3StateSpace>(2);

  dmetric.interpolate(state1, state2, 0, istate);
  EXPECT_DOUBLE_EQ(state1_handle0.getAngle(), istate_handle0.getAngle());
  EXPECT_TRUE(state1_handle1.getValue().isApprox(istate_handle1.getValue()));
  EXPECT_TRUE(
      state1_handle2.getQuaternion().isApprox(istate_handle2.getQuaternion()));
  EXPECT_TRUE(space.equalStates(state1, istate));

  dmetric.interpolate(state1, state2, 1, istate);
  EXPECT_DOUBLE_EQ(state2_handle0.getAngle(), istate_handle0.getAngle());
  EXPECT_TRUE(state2_handle1.getValue().isApprox(istate_handle1.getValue()));
  EXPECT_TRUE(
      state2_handle2.getQuaternion().isApprox(istate_handle2.getQuaternion()));
  EXPECT_TRUE(space.equalStates(state2, istate));

  dmetric.interpolate(state1, state2, 0.5, istate);
  EXPECT_DOUBLE_EQ(0.25 + M_PI, istate_handle0.getAngle());
  auto rv3 = Eigen::Vector3d(2, 3, 4);
  EXPECT_TRUE(rv3.isApprox(istate_handle1.getValue()));

  auto quat3 = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI - 0.25, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(quat3.isApprox(istate_handle2.getQuaternion()));
}
