#include <gtest/gtest.h>
#include <aikido/constraint/TestableConstraint.hpp>
#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

TEST(GetMaximumExtent, SO2StateSpace)
{
    auto so2 = std::make_shared<aikido::statespace::SO2StateSpace>();
    auto dmetric = std::make_shared<aikido::distance::AngularDistanceMetric>(so2);

    double v = aikido::ompl::getMaximumExtentFor(so2, dmetric);

    ASSERT_DOUBLE_EQ(M_PI, v);
}

TEST(GetMaximumExtent, SO3StateSpace)
{
    auto so3 = std::make_shared<aikido::statespace::SO3StateSpace>();
    auto dmetric = std::make_shared<aikido::distance::GeodesicDistanceMetric>(so3);

    double v = aikido::ompl::getMaximumExtentFor(so3, dmetric);

    ASSERT_DOUBLE_EQ(0.5*M_PI, v);
}
