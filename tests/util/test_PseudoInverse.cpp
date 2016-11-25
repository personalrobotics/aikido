#include <aikido/util/PseudoInverse.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using namespace aikido::util;

TEST(PseudoInverse, Invertible)
{
  Eigen::Matrix2d mat;
  mat.setIdentity();
  Eigen::Matrix2d inverse = pseudoinverse(mat);

  EXPECT_TRUE((inverse*mat).isApprox(Eigen::Matrix2d::Identity()));

}


TEST(PseudoInverse, Vector)
{
  Eigen::Vector2d vec(1, 1);

  Eigen::MatrixXd inverse = pseudoinverse(vec);

  EXPECT_DOUBLE_EQ((inverse*vec)(0,0), 1);

}



TEST(PseudoInverse, Matrix)
{
  Eigen::MatrixXd mat(Eigen::MatrixXd::Random(3,4));
  Eigen::MatrixXd inverse = pseudoinverse(mat);

  EXPECT_TRUE((mat*inverse).isApprox(Eigen::Matrix3d::Identity()));


}
