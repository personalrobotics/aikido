#include <dart/common/Memory.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/CartesianProductProjectable.hpp>
#include <aikido/constraint/Projectable.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/statespace/Rn.hpp>
#include "../eigen_tests.hpp"

using aikido::constraint::CartesianProductProjectable;
using aikido::constraint::ProjectablePtr;
using aikido::constraint::uniform::R2BoxConstraint;
using aikido::constraint::uniform::R3BoxConstraint;
using aikido::statespace::CartesianProduct;
using aikido::statespace::R2;
using aikido::statespace::R3;

class CartesianProductProjectableTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss1 = std::make_shared<R3>();
    rvss2 = std::make_shared<R2>();

    // Constraints
    rvBox1 = dart::common::make_aligned_shared<R3BoxConstraint>(
        rvss1, nullptr, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 1, 1));
    rvBox2 = dart::common::make_aligned_shared<R2BoxConstraint>(
        rvss2, nullptr, Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 2));

    projectables.push_back(rvBox1);
    projectables.push_back(rvBox2);

    cs = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::ConstStateSpacePtr>({rvss1, rvss2}));
  }

  std::shared_ptr<CartesianProduct> cs;
  std::vector<ProjectablePtr> projectables;
  std::shared_ptr<R3> rvss1;
  std::shared_ptr<R2> rvss2;
  std::shared_ptr<R3BoxConstraint> rvBox1;
  std::shared_ptr<R2BoxConstraint> rvBox2;
};

TEST_F(CartesianProductProjectableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      CartesianProductProjectable(nullptr, projectables),
      std::invalid_argument);
}

TEST_F(CartesianProductProjectableTest, ConstructorThrowsOnNullConstraints)
{
  std::vector<ProjectablePtr> projectables;
  projectables.push_back(nullptr);
  projectables.push_back(nullptr);

  EXPECT_THROW(
      CartesianProductProjectable(cs, projectables), std::invalid_argument);
}

TEST_F(
    CartesianProductProjectableTest,
    ConstructorThrowsOnUnmatchingStateSpaceConstraintPair)
{
  std::vector<ProjectablePtr> projectables;
  projectables.push_back(rvBox2);
  projectables.push_back(rvBox1);

  EXPECT_THROW(
      CartesianProductProjectable(cs, projectables), std::invalid_argument);
}

TEST_F(
    CartesianProductProjectableTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto ps = std::make_shared<CartesianProductProjectable>(cs, projectables);
  auto space = ps->getStateSpace();
  EXPECT_EQ(space, cs);
}

TEST_F(CartesianProductProjectableTest, ProjectsToCorrectValues)
{
  auto ps = std::make_shared<CartesianProductProjectable>(cs, projectables);

  auto state = cs->createState();
  auto subState = cs->getSubStateHandle<R3>(state, 0);
  subState.setValue(Eigen::Vector3d(-1, -1, -1));

  auto out = cs->createState();

  EXPECT_TRUE(ps->project(state, out));

  auto outSubState1 = cs->getSubStateHandle<R3>(out, 0);
  EXPECT_TRUE(outSubState1.getValue().isApprox(Eigen::Vector3d(1, 1, 1)));

  auto outSubState2 = cs->getSubStateHandle<R2>(out, 1);
  EXPECT_TRUE(outSubState2.getValue().isApprox(Eigen::Vector2d(1, 1)));
}
