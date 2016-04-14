#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/distance/WeightedDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(DistanceMetricDefaults, Composition)
{

  auto so2 = std::make_shared<SO2StateSpace>();

  auto so2metric = createDistanceMetricFor(so2);
  EXPECT_TRUE(so2metric != nullptr);
  auto so2metric_c = dynamic_cast<AngularDistanceMetric*>(so2metric.get());
  EXPECT_TRUE(so2metric_c != nullptr);

  auto rv3 = std::make_shared<RealVectorStateSpace>(3);

  auto rv3metric = createDistanceMetricFor(rv3);
  EXPECT_TRUE(rv3metric != nullptr);
  auto rv3metric_c = dynamic_cast<EuclideanDistanceMetric*>(rv3metric.get());
  EXPECT_TRUE(rv3metric_c != nullptr);

  auto so3 = std::make_shared<SO3StateSpace>();

  auto so3metric = createDistanceMetricFor(so3);
  EXPECT_TRUE(so3metric != nullptr);
  auto so3metric_c = dynamic_cast<GeodesicDistanceMetric*>(so3metric.get());
  EXPECT_TRUE(so3metric_c != nullptr);


  std::vector<std::shared_ptr<StateSpace> > spaces = {so2, rv3, so3};
  auto space = std::make_shared<CompoundStateSpace>(spaces);

  auto dmetric = createDistanceMetricFor(space);
  EXPECT_TRUE(dmetric != nullptr);

  auto cmetric = dynamic_cast<WeightedDistanceMetric*>(dmetric.get());
  EXPECT_TRUE(cmetric != nullptr);
  
}
