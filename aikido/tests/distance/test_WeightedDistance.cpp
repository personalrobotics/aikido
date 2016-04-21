#include <aikido/distance/WeightedDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(WeightedDistance, ThrowsOnNullStateSpace)
{
  auto space = nullptr;
  std::vector<DistanceMetricPtr> empty;
  EXPECT_THROW(WeightedDistanceMetric(space, empty), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr, double>> empty2;
  EXPECT_THROW(WeightedDistanceMetric(space, empty2), std::invalid_argument);
}

TEST(WeightedDistance, ThrowsOnNullMetric){
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  std::vector<DistanceMetricPtr> dmetrics = {
      std::make_shared<AngularDistanceMetric>(so2),
      std::make_shared<EuclideanDistanceMetric>(rv3), nullptr};

  EXPECT_THROW(WeightedDistanceMetric(space, dmetrics), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr, double>> dmetrics2 = {
      std::make_pair(std::make_shared<AngularDistanceMetric>(so2), 1),
      std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 1),
      std::make_pair(nullptr, 1)};
  EXPECT_THROW(WeightedDistanceMetric(space, dmetrics2),
                std::invalid_argument);
}

TEST(WeightedDistance, ThrowsOnMissingMetric)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  std::vector<DistanceMetricPtr> dmetrics = {
      std::make_shared<AngularDistanceMetric>(so2),
              std::make_shared<EuclideanDistanceMetric>(rv3)
  };

  EXPECT_THROW(WeightedDistanceMetric(space, dmetrics), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr,double>> dmetrics2 = {
      std::make_pair(std::make_shared<AngularDistanceMetric>(so2), 1),
      std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 1)
  };
  EXPECT_THROW(WeightedDistanceMetric(space, dmetrics2), std::invalid_argument);
}

TEST(WeightedDistance, ThrowsOnMismatchMetricStatespace)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  EXPECT_THROW(WeightedDistanceMetric(
                    space, {std::make_shared<EuclideanDistanceMetric>(rv3),
                            std::make_shared<AngularDistanceMetric>(so2),
                            std::make_shared<GeodesicDistanceMetric>(so3)}),
                std::invalid_argument);

  EXPECT_THROW(
      WeightedDistanceMetric(
          space,
          {std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 2),
           std::make_pair(std::make_shared<AngularDistanceMetric>(so2), 1),
           std::make_pair(std::make_shared<GeodesicDistanceMetric>(so3), 3)}),
      std::invalid_argument);
}

TEST(WeightedDistance, ThrowsOnNegativeWeights)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);
  EXPECT_THROW(
      WeightedDistanceMetric(
          space,
          {std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 2),
           std::make_pair(std::make_shared<AngularDistanceMetric>(so2), -1),
           std::make_pair(std::make_shared<GeodesicDistanceMetric>(so3), 3)}),
      std::invalid_argument);
}

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

  WeightedDistanceMetric dmetric2(
      space, {std::make_pair(std::make_shared<AngularDistanceMetric>(so2), 1),
              std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 2),
              std::make_pair(std::make_shared<GeodesicDistanceMetric>(so3), 3)});
  EXPECT_EQ(space, dmetric2.getStateSpace());
}

TEST(WeightedDistance, DistanceUnitWeights)
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

TEST(WeightedDistance, DistanceCustomWeights)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto so3 = std::make_shared<SO3StateSpace>();
  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};

  auto space = std::make_shared<CompoundStateSpace>(spaces);

  WeightedDistanceMetric dmetric(
      space, {std::make_pair(std::make_shared<AngularDistanceMetric>(so2), 2),
              std::make_pair(std::make_shared<EuclideanDistanceMetric>(rv3), 3),
              std::make_pair(std::make_shared<GeodesicDistanceMetric>(so3), 4)});

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
  EXPECT_DOUBLE_EQ(2*0.5 + 4*0.5 + 3*vdiff.norm(), dmetric.distance(state1, state2));
}
