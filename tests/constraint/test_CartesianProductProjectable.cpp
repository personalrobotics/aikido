#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/constraint/CartesianProductProjectable.hpp>
#include <aikido/constraint/Projectable.hpp>
#include <aikido/statespace/Rn.hpp>

using aikido::constraint::CartesianProductProjectable;
using aikido::constraint::ProjectablePtr;
using aikido::statespace::CartesianProduct;
using aikido::statespace::Rn;
using aikido::constraint::RnBoxConstraint;

class CartesianProductProjectableTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss1 = std::make_shared<Rn>(3);
    rvss2 = std::make_shared<Rn>(2);

    // Constraints
    rvBox1 = std::make_shared<RnBoxConstraint>(
      rvss1, nullptr, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 1, 1));
    rvBox2 = std::make_shared<RnBoxConstraint>(
      rvss2, nullptr, Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 2));

    projectables.push_back(rvBox1);
    projectables.push_back(rvBox2);

    cs = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss1, rvss2}));
  }

  std::shared_ptr<CartesianProduct> cs;
  std::vector<ProjectablePtr> projectables;
  std::shared_ptr<Rn> rvss1, rvss2;
  std::shared_ptr<RnBoxConstraint> rvBox1, rvBox2;

};


TEST_F(CartesianProductProjectableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(CartesianProductProjectable(nullptr, projectables),
               std::invalid_argument);
}


TEST_F(CartesianProductProjectableTest, ConstructorThrowsOnNullConstraints)
{
  std::vector<ProjectablePtr> projectables; 
  projectables.push_back(nullptr);
  projectables.push_back(nullptr);

  EXPECT_THROW(CartesianProductProjectable(cs, projectables),
               std::invalid_argument);
}


TEST_F(CartesianProductProjectableTest, ConstructorThrowsOnUnmatchingStateSpaceConstraintPair)
{
  std::vector<ProjectablePtr> projectables; 
  projectables.push_back(rvBox2);
  projectables.push_back(rvBox1);

  EXPECT_THROW(CartesianProductProjectable(cs, projectables),
               std::invalid_argument);
}


TEST_F(CartesianProductProjectableTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto ps = std::make_shared<CartesianProductProjectable>(cs, projectables);
  auto space = ps->getStateSpace();
  EXPECT_EQ(space, cs);
}


TEST_F(CartesianProductProjectableTest, ProjectsToCorrectValues)
{
  auto ps = std::make_shared<CartesianProductProjectable>(cs, projectables);

  auto state = cs->createState();
  auto subState = cs->getSubStateHandle<Rn>(state, 0);
  subState.setValue(Eigen::Vector3d(-1, -1, -1));

  auto out = cs->createState();

  EXPECT_TRUE(ps->project(state, out));

  auto outSubState1 = cs->getSubStateHandle<Rn>(out, 0);
  EXPECT_TRUE(outSubState1.getValue().isApprox(Eigen::Vector3d(1, 1, 1)));

  auto outSubState2 = cs->getSubStateHandle<Rn>(out, 1);
  EXPECT_TRUE(outSubState2.getValue().isApprox(Eigen::Vector2d(1, 1)));
}

