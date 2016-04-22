#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/uniform/RealVectorBoxConstraint.hpp>
#include <aikido/constraint/ProjectableSubSpace.hpp>
#include <aikido/constraint/Projectable.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <dart/common/Console.h>

using aikido::constraint::ProjectableSubSpace;
using aikido::constraint::ProjectablePtr;
using aikido::statespace::CompoundStateSpace;
using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::RealVectorBoxConstraint;

TEST(Project, ProjectInPlaceDefault)
{
  dtwarn << "This is tested on RealVectorBoxConstraint, which uses "
            "default in-place project method. If this class later "
            "overrides this method, we should write a different test.\n";

  auto rvss = std::make_shared<RealVectorStateSpace>(3);
  auto rvBox = std::make_shared<RealVectorBoxConstraint>(
    rvss, nullptr, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 1, 1));

  auto state = rvss->createState();
  auto out = rvss->createState();

  state.setValue(Eigen::Vector3d(-1, 0, 1));

  EXPECT_TRUE(rvBox->project(state, out));
  EXPECT_TRUE(rvBox->project(state));

  EXPECT_TRUE(state.getValue().isApprox(out.getValue()));
  EXPECT_TRUE(state.getValue().isApprox(Eigen::Vector3d(1, 1, 1)));
}

