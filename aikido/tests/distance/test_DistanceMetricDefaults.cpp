#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/distance/WeightedDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(DistanceMetricDefaults, CreateDistanceMetricForThrowsOnNull)
{
  EXPECT_THROW(createDistanceMetricFor<SO2StateSpace>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<RealVectorStateSpace>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<SO3StateSpace>(nullptr),
               std::invalid_argument);
  EXPECT_THROW(createDistanceMetricFor<CompoundStateSpace>(nullptr),
               std::invalid_argument);
}

TEST(DistanceMetricDefaults, CreateDistanceMetricThrowsOnNull)
{
  EXPECT_THROW(createDistanceMetric(nullptr), std::invalid_argument);
}

TEST(DistanceMetricDefaults, CreateDistanceMetricFor)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto so2metric = createDistanceMetricFor<SO2StateSpace>(so2);
  auto so2metric_c = dynamic_cast<AngularDistanceMetric*>(so2metric.get());
  EXPECT_TRUE(so2metric_c != nullptr);

  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto rv3metric = createDistanceMetricFor<RealVectorStateSpace>(rv3);
  auto rv3metric_c = dynamic_cast<EuclideanDistanceMetric*>(rv3metric.get());
  EXPECT_TRUE(rv3metric_c != nullptr);

  auto so3 = std::make_shared<SO3StateSpace>();
  auto so3metric = createDistanceMetricFor<SO3StateSpace>(so3);
  auto so3metric_c = dynamic_cast<GeodesicDistanceMetric*>(so3metric.get());
  EXPECT_TRUE(so3metric_c != nullptr);

  std::vector<StateSpacePtr> spaces({so2, rv3, so3});
  auto space = std::make_shared<CompoundStateSpace>(spaces);
  auto dmetric = createDistanceMetricFor<CompoundStateSpace>(space);
  auto cmetric = dynamic_cast<WeightedDistanceMetric*>(dmetric.get());
  EXPECT_TRUE(cmetric != nullptr);
}

TEST(DistanceMetricDefaults, CreateDistanceMetric)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto so2metric = createDistanceMetric(so2);
  auto so2metric_c = dynamic_cast<AngularDistanceMetric*>(so2metric.get());
  EXPECT_TRUE(so2metric_c != nullptr);

  auto rv3 = std::make_shared<RealVectorStateSpace>(3);
  auto rv3metric = createDistanceMetric(rv3);
  auto rv3metric_c = dynamic_cast<EuclideanDistanceMetric*>(rv3metric.get());
  EXPECT_TRUE(rv3metric_c != nullptr);

  auto so3 = std::make_shared<SO3StateSpace>();
  auto so3metric = createDistanceMetric(so3);
  auto so3metric_c = dynamic_cast<GeodesicDistanceMetric*>(so3metric.get());
  EXPECT_TRUE(so3metric_c != nullptr);

  std::vector<StateSpacePtr> spaces({so2, rv3, so3});
  auto space = std::make_shared<CompoundStateSpace>(spaces);
  auto dmetric = createDistanceMetric(space);
  auto cmetric = dynamic_cast<WeightedDistanceMetric*>(dmetric.get());
  EXPECT_TRUE(cmetric != nullptr);
}
