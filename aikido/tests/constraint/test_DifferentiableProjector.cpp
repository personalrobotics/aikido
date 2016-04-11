#include <aikido/constraint/PolynomialConstraint.hpp>
#include <aikido/constraint/DifferentiableProjector.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::PolynomialConstraint;
using aikido::constraint::DifferentiableProjector;

using aikido::statespace::RealVectorStateSpace;

TEST(DifferentiableProjector, Constructor)
{

  DifferentiableProjector p(
    std::make_shared<PolynomialConstraint>(Eigen::Vector3d(1, 2 ,3)));

  /// TODO: test null 
}

TEST(DifferentiableProjector, ProjectPolynomial)
{

  DifferentiableProjector projector(
    std::make_shared<PolynomialConstraint>(Eigen::Vector2d(1, 2)));

  Eigen::VectorXd v(1);
  v(0) = -2;

  RealVectorStateSpace rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  auto out = rvss.createState();
  bool success = projector.project(s1, out);
  Eigen::VectorXd projected = rvss.getValue(out);

  Eigen::VectorXd expected(1);
  expected(0) = -0.5;

  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(projected));

}

// TEST(DifferentiableProjector, ProjectSO2)
// {
  
// }

// TEST(DifferentiableProjector, ProjectCompoundConstraint)
// {
//    /// test with multiple constraints-constriant


// }