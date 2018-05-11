#include <aikido/distance/CartesianProductWeighted.hpp>
#include <aikido/distance/RnEuclidean.hpp>
#include <aikido/distance/SO2Angular.hpp>
#include <aikido/distance/SO3Angular.hpp>
#include <aikido/statespace/CartesianProduct.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(CartesianProductWeightedDistance, ThrowsOnNullStateSpace)
{
  auto space = nullptr;
  std::vector<DistanceMetricPtr> empty;
  EXPECT_THROW(CartesianProductWeighted(space, empty), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr, double>> empty2;
  EXPECT_THROW(CartesianProductWeighted(space, empty2), std::invalid_argument);
}

TEST(CartesianProductWeightedDistance, ThrowsOnNullMetric)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  std::vector<DistanceMetricPtr> dmetrics = {std::make_shared<SO2Angular>(so2),
                                             std::make_shared<R3Euclidean>(rv3),
                                             nullptr};

  EXPECT_THROW(
      CartesianProductWeighted(space, dmetrics), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr, double>> dmetrics2
      = {std::make_pair(std::make_shared<SO2Angular>(so2), 1),
         std::make_pair(std::make_shared<R3Euclidean>(rv3), 1),
         std::make_pair(nullptr, 1)};
  EXPECT_THROW(
      CartesianProductWeighted(space, dmetrics2), std::invalid_argument);
}

TEST(CartesianProductWeightedDistance, ThrowsOnMissingMetric)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  std::vector<DistanceMetricPtr> dmetrics
      = {std::make_shared<SO2Angular>(so2), std::make_shared<R3Euclidean>(rv3)};

  EXPECT_THROW(
      CartesianProductWeighted(space, dmetrics), std::invalid_argument);

  std::vector<std::pair<DistanceMetricPtr, double>> dmetrics2
      = {std::make_pair(std::make_shared<SO2Angular>(so2), 1),
         std::make_pair(std::make_shared<R3Euclidean>(rv3), 1)};
  EXPECT_THROW(
      CartesianProductWeighted(space, dmetrics2), std::invalid_argument);
}

TEST(CartesianProductWeightedDistance, ThrowsOnMismatchMetricStatespace)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  EXPECT_THROW(
      CartesianProductWeighted(
          space,
          {std::make_shared<R3Euclidean>(rv3),
           std::make_shared<SO2Angular>(so2),
           std::make_shared<SO3Angular>(so3)}),
      std::invalid_argument);

  EXPECT_THROW(
      CartesianProductWeighted(
          space,
          {std::make_pair(std::make_shared<R3Euclidean>(rv3), 2),
           std::make_pair(std::make_shared<SO2Angular>(so2), 1),
           std::make_pair(std::make_shared<SO3Angular>(so3), 3)}),
      std::invalid_argument);
}

TEST(CartesianProductWeightedDistance, ThrowsOnNegativeWeights)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);
  EXPECT_THROW(
      CartesianProductWeighted(
          space,
          {std::make_pair(std::make_shared<SO2Angular>(so2), -1),
           std::make_pair(std::make_shared<R3Euclidean>(rv3), 2),
           std::make_pair(std::make_shared<SO3Angular>(so3), 3)}),
      std::invalid_argument);
}

TEST(CartesianProductWeightedDistance, StateSpaceEquality)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  CartesianProductWeighted dmetric(
      space,
      {std::make_shared<SO2Angular>(so2),
       std::make_shared<R3Euclidean>(rv3),
       std::make_shared<SO3Angular>(so3)});
  EXPECT_EQ(space, dmetric.getStateSpace());

  CartesianProductWeighted dmetric2(
      space,
      {std::make_pair(std::make_shared<SO2Angular>(so2), 1),
       std::make_pair(std::make_shared<R3Euclidean>(rv3), 2),
       std::make_pair(std::make_shared<SO3Angular>(so3), 3)});
  EXPECT_EQ(space, dmetric2.getStateSpace());
}

TEST(CartesianProductWeightedDistance, DistanceUnitWeights)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  CartesianProductWeighted dmetric(
      space,
      {std::make_shared<SO2Angular>(so2),
       std::make_shared<R3Euclidean>(rv3),
       std::make_shared<SO3Angular>(so3)});

  auto state1 = space->createState();
  auto state2 = space->createState();

  double angle1 = M_PI;
  double angle2 = 0.5 + M_PI;
  auto rv1 = Eigen::Vector3d(3, 4, 5);
  auto rv2 = Eigen::Vector3d(1, 2, 3);
  auto quat1
      = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  auto quat2 = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI - 0.5, Eigen::Vector3d::UnitZ()));

  state1.getSubStateHandle<SO2>(0).fromAngle(angle1);
  state1.getSubStateHandle<R3>(1).setValue(rv1);
  state1.getSubStateHandle<SO3>(2).setQuaternion(quat1);

  state2.getSubStateHandle<SO2>(0).fromAngle(angle2);
  state2.getSubStateHandle<R3>(1).setValue(rv2);
  state2.getSubStateHandle<SO3>(2).setQuaternion(quat2);

  auto vdiff = Eigen::Vector3d(2, 2, 2);
  EXPECT_DOUBLE_EQ(0.5 + 0.5 + vdiff.norm(), dmetric.distance(state1, state2));
}

TEST(CartesianProductWeightedDistance, DistanceCustomWeights)
{
  auto so2 = std::make_shared<SO2>();
  auto rv3 = std::make_shared<R3>();
  auto so3 = std::make_shared<SO3>();
  std::vector<std::shared_ptr<StateSpace>> spaces = {so2, rv3, so3};

  auto space = std::make_shared<CartesianProduct>(spaces);

  CartesianProductWeighted dmetric(
      space,
      {std::make_pair(std::make_shared<SO2Angular>(so2), 2),
       std::make_pair(std::make_shared<R3Euclidean>(rv3), 3),
       std::make_pair(std::make_shared<SO3Angular>(so3), 4)});

  auto state1 = space->createState();
  auto state2 = space->createState();

  double angle1 = M_PI;
  double angle2 = 0.5 + M_PI;
  auto rv1 = Eigen::Vector3d(3, 4, 5);
  auto rv2 = Eigen::Vector3d(1, 2, 3);
  auto quat1
      = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  auto quat2 = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI - 0.5, Eigen::Vector3d::UnitZ()));

  state1.getSubStateHandle<SO2>(0).fromAngle(angle1);
  state1.getSubStateHandle<R3>(1).setValue(rv1);
  state1.getSubStateHandle<SO3>(2).setQuaternion(quat1);

  state2.getSubStateHandle<SO2>(0).fromAngle(angle2);
  state2.getSubStateHandle<R3>(1).setValue(rv2);
  state2.getSubStateHandle<SO3>(2).setQuaternion(quat2);

  auto vdiff = Eigen::Vector3d(2, 2, 2);
  EXPECT_DOUBLE_EQ(
      2 * 0.5 + 4 * 0.5 + 3 * vdiff.norm(), dmetric.distance(state1, state2));
}
