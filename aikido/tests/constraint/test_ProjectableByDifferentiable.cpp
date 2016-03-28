#include <aikido/constraint/PolynomialConstraint.hpp>
#include <aikido/constraint/ProjectableByDifferentiable.hpp>

#include <aikido/state/State.hpp>
#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::PolynomialConstraint;
using aikido::constraint::ProjectableByDifferentiable;

using namespace aikido::state;

TEST(ProjectableByDifferentiable, Constructor)
{

  ProjectableByDifferentiable p(std::make_shared<PolynomialConstraint>(
  							  	Eigen::Vector3d(1,2,3)));

  /// test null 
}


TEST(ProjectableByDifferentiable, Contains)
{

  ProjectableByDifferentiable p(std::make_shared<PolynomialConstraint>(
  							  	Eigen::Vector2d(1, 2)));

  EXPECT_TRUE(p.contains(std::make_shared<RealVectorState>(-0.5)));

  EXPECT_FALSE(p.contains(std::make_shared<RealVectorState>(0)));

  /// test invalid values 
}


TEST(ProjectableByDifferentiable, ProjectPolynomial)
{

  ProjectableByDifferentiable p(std::make_shared<PolynomialConstraint>(
  							  	Eigen::Vector2d(1,2)));

  boost::optional<StatePtr> s = p.project(
  								std::make_shared<RealVectorState>(0));

  EXPECT_TRUE(p.contains(s.get()));


}

TEST(ProjectableByDifferentiable, ProjectSO2)
{
	
}

TEST(ProjectableByDifferentiable, ProjectCompoundConstraint)
{
	 /// test with multiple constraints-constriant


}