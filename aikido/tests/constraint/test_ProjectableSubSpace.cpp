#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/uniform/RealVectorBoxConstraint.hpp>
#include <aikido/constraint/ProjectableSubSpace.hpp>
#include <aikido/constraint/Projectable.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

using aikido::constraint::ProjectableSubSpace;
using aikido::constraint::ProjectablePtr;
using aikido::statespace::CompoundStateSpace;
using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::RealVectorBoxConstraint;

class ProjectableSubSpaceTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss1 = std::make_shared<RealVectorStateSpace>(3);
    rvss2 = std::make_shared<RealVectorStateSpace>(2);

    // Constraints
    rvBox1 = std::make_shared<RealVectorBoxConstraint>(
      rvss1, nullptr, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 1, 1));
    rvBox2 = std::make_shared<RealVectorBoxConstraint>(
      rvss2, nullptr, Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 2));

    projectables.push_back(rvBox1);
    projectables.push_back(rvBox2);

    cs = std::make_shared<CompoundStateSpace>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss1, rvss2}));
  }

  std::shared_ptr<CompoundStateSpace> cs;
  std::vector<ProjectablePtr> projectables;
  std::shared_ptr<RealVectorStateSpace> rvss1, rvss2;
  std::shared_ptr<RealVectorBoxConstraint> rvBox1, rvBox2;

};


TEST_F(ProjectableSubSpaceTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(ProjectableSubSpace(nullptr, projectables),
               std::invalid_argument);
}


TEST_F(ProjectableSubSpaceTest, ConstructorThrowsOnNullConstraints)
{
  std::vector<ProjectablePtr> projectables; 
  projectables.push_back(nullptr);
  projectables.push_back(nullptr);

  EXPECT_THROW(ProjectableSubSpace(cs, projectables),
               std::invalid_argument);
}


TEST_F(ProjectableSubSpaceTest, ConstructorThrowsOnUnmatchingStateSpaceConstraintPair)
{
  std::vector<ProjectablePtr> projectables; 
  projectables.push_back(rvBox2);
  projectables.push_back(rvBox1);

  EXPECT_THROW(ProjectableSubSpace(cs, projectables),
               std::invalid_argument);
}


TEST_F(ProjectableSubSpaceTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto ps = std::make_shared<ProjectableSubSpace>(cs, projectables);
  auto space = ps->getStateSpace();
  EXPECT_EQ(space, cs);
}


TEST_F(ProjectableSubSpaceTest, ProjectsToCorrectValues)
{
  auto ps = std::make_shared<ProjectableSubSpace>(cs, projectables);

  auto state = cs->createState();
  auto subState = cs->getSubStateHandle<RealVectorStateSpace>(state, 0);
  subState.setValue(Eigen::Vector3d(-1, -1, -1));

  auto out = cs->createState();

  EXPECT_TRUE(ps->project(state, out));

  auto outSubState1 = cs->getSubStateHandle<RealVectorStateSpace>(out, 0);
  EXPECT_TRUE(outSubState1.getValue().isApprox(Eigen::Vector3d(1, 1, 1)));

  auto outSubState2 = cs->getSubStateHandle<RealVectorStateSpace>(out, 1);
  EXPECT_TRUE(outSubState2.getValue().isApprox(Eigen::Vector2d(1, 1)));
}

