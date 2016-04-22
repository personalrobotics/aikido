#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/distance/Weighted.hpp>
#include <aikido/distance/SO3Angular.hpp>
#include <aikido/distance/RnEuclidean.hpp>
#include <aikido/distance/SO2Angular.hpp>
#include <aikido/statespace/CartesianProduct.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(DistanceMetricDefaults, CreateDistanceMetricForThrowsOnNull)
{
  EXPECT_THROW(createDistanceMetricFor<SO2>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<Rn>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<SO3>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<CartesianProduct>(nullptr),
               std::invalid_argument);
}

TEST(DistanceMetricDefaults, CreateDistanceMetricThrowsOnNull)
{
  EXPECT_THROW(createDistanceMetric(nullptr), std::invalid_argument);
}

TEST(DistanceMetricDefaults, CreateDistanceMetricFor)
{
  auto so2 = std::make_shared<SO2>();
  auto so2metric = createDistanceMetricFor<SO2>(so2);
  auto so2metric_c = dynamic_cast<SO2Angular*>(so2metric.get());
  EXPECT_TRUE(so2metric_c != nullptr);

  auto rv3 = std::make_shared<Rn>(3);
  auto rv3metric = createDistanceMetricFor<Rn>(rv3);
  auto rv3metric_c = dynamic_cast<RnEuclidean*>(rv3metric.get());
  EXPECT_TRUE(rv3metric_c != nullptr);

  auto so3 = std::make_shared<SO3>();
  auto so3metric = createDistanceMetricFor<SO3>(so3);
  auto so3metric_c = dynamic_cast<SO3Angular*>(so3metric.get());
  EXPECT_TRUE(so3metric_c != nullptr);

  std::vector<StateSpacePtr> spaces({so2, rv3, so3});
  auto space = std::make_shared<CartesianProduct>(spaces);
  auto dmetric = createDistanceMetricFor<CartesianProduct>(space);
  auto cmetric = dynamic_cast<Weighted*>(dmetric.get());
  EXPECT_TRUE(cmetric != nullptr);
}

TEST(DistanceMetricDefaults, CreateDistanceMetric)
{
  auto so2 = std::make_shared<SO2>();
  auto so2metric = createDistanceMetric(so2);
  auto so2metric_c = dynamic_cast<SO2Angular*>(so2metric.get());
  EXPECT_TRUE(so2metric_c != nullptr);

  auto rv3 = std::make_shared<Rn>(3);
  auto rv3metric = createDistanceMetric(rv3);
  auto rv3metric_c = dynamic_cast<RnEuclidean*>(rv3metric.get());
  EXPECT_TRUE(rv3metric_c != nullptr);

  auto so3 = std::make_shared<SO3>();
  auto so3metric = createDistanceMetric(so3);
  auto so3metric_c = dynamic_cast<SO3Angular*>(so3metric.get());
  EXPECT_TRUE(so3metric_c != nullptr);

  std::vector<StateSpacePtr> spaces({so2, rv3, so3});
  auto space = std::make_shared<CartesianProduct>(spaces);
  auto dmetric = createDistanceMetric(space);
  auto cmetric = dynamic_cast<Weighted*>(dmetric.get());
  EXPECT_TRUE(cmetric != nullptr);
}
