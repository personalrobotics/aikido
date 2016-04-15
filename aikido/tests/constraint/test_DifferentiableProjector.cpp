#include <aikido/constraint/PolynomialConstraint.hpp>
#include <aikido/constraint/DifferentiableProjector.hpp>
#include <aikido/constraint/TSR.hpp>

#include <aikido/statespace/RealVectorStateSpace.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::PolynomialConstraint;
using aikido::constraint::DifferentiableProjector;
using aikido::constraint::TSR;
using aikido::statespace::RealVectorStateSpace;

TEST(DifferentiableProjector, Constructor)
{
  DifferentiableProjector p(
    std::make_shared<PolynomialConstraint>(Eigen::Vector3d(1, 2 ,3))); 
}

TEST(DifferentiableProjector, ProjectPolynomialFirstOrder)
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


TEST(DifferentiableProjector, ProjectPolynomialSecondOrder)
{ 
  // Constraint: x^2 - 1 = 0.
  DifferentiableProjector projector(
    std::make_shared<PolynomialConstraint>(Eigen::Vector3d(-1, 0, 1)));

  // Project x = -2. Should get -1 as projected solution.
  Eigen::VectorXd v(1);
  v(0) = -2;

  RealVectorStateSpace rvss(1);
  auto seedState = rvss.createState();
  seedState.setValue(v);

  auto out = rvss.createState();
  EXPECT_TRUE(projector.project(seedState, out));
  Eigen::VectorXd projected = rvss.getValue(out);

  Eigen::VectorXd expected(1);
  expected(0) = -1;

  EXPECT_TRUE(expected.isApprox(projected, 1e-5));


  // Project x = 1.5. Should get 1 as projected solution.
  v(0) = 1.5;
  seedState.setValue(v);

  EXPECT_TRUE(projector.project(seedState, out));
  projected = rvss.getValue(out);

  expected(0) = 1;

  EXPECT_TRUE(expected.isApprox(projected, 1e-5));


  // Project x = 1. Should get 1 as projected solution. 
  v(0) = 1;
  seedState.setValue(v);

  EXPECT_TRUE(projector.project(seedState, out));
  projected = rvss.getValue(out);

  expected(0) = 1;

  EXPECT_TRUE(expected.isApprox(projected, 1e-5));

}

TEST(DifferentiableProjector, ProjectTSRTranslation)
{
  std::shared_ptr<TSR> tsr = std::make_shared<TSR>();

  // non-trivial translation bounds 
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = 1;
  Bw(0,1) = 2;

  tsr->mBw = Bw;

  auto space = tsr->getSE3StateSpace();

  auto seedState = space->createState();  

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(-1, 0, 1);
  seedState.setIsometry(isometry);

  DifferentiableProjector projector(tsr, 1000, 1e-4, 1e-5);

  auto out = space->createState();
  EXPECT_TRUE(projector.project(seedState, out));
  auto projected = space->getIsometry(out);

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() = Eigen::Vector3d(1, 0, 0);

  EXPECT_TRUE(expected.isApprox(projected, 5e-4));

}


TEST(DifferentiableProjector, ProjectTSRRotation)
{
  std::shared_ptr<TSR> tsr = std::make_shared<TSR>();

  // non-trivial rotation bounds 
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(3,0) = M_PI_4;
  Bw(3,1) = M_PI_2;

  tsr->mBw = Bw;

  auto space = tsr->getSE3StateSpace();
  auto seedState = space->createState();

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();

  DifferentiableProjector projector(tsr, 1000, 1e-4, 1e-5);

  auto out = space->createState();
  EXPECT_TRUE(projector.project(seedState, out));
  auto projected = space->getIsometry(out);

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  expected.linear() = rotation;

  EXPECT_TRUE(expected.isApprox(projected, 5e-4));
}
