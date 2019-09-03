#include <dart/common/Console.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/CartesianProductProjectable.hpp>
#include <aikido/constraint/Projectable.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/statespace/Rn.hpp>
#include "../eigen_tests.hpp"

using aikido::constraint::CartesianProductProjectable;
using aikido::constraint::ProjectablePtr;
using aikido::constraint::uniform::R3BoxConstraint;
using aikido::statespace::CartesianProduct;
using aikido::statespace::R3;

TEST(Project, ProjectInPlaceDefault)
{
  dtwarn << "This is tested on RnBoxConstraint, which uses "
            "default in-place project method. If this class later "
            "overrides this method, we should write a different test.\n";

  auto rvss = std::make_shared<R3>();
  auto rvBox = std::make_shared<R3BoxConstraint>(
      rvss, nullptr, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 1, 1));

  auto state = rvss->createState();
  auto out = rvss->createState();

  state.setValue(Eigen::Vector3d(-1, 0, 1));

  EXPECT_TRUE(rvBox->project(state, out));
  EXPECT_TRUE(rvBox->project(state));

  EXPECT_TRUE(state.getValue().isApprox(out.getValue()));
  EXPECT_TRUE(state.getValue().isApprox(Eigen::Vector3d(1, 1, 1)));
}
