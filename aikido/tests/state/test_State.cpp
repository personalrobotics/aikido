#include <aikido/state/State.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <typeinfo>


using namespace aikido::state;

TEST(RealVectorState, Constructor)
{
  Eigen::Vector3d v(0,0,0);
  RealVectorState s(v);

  EXPECT_EQ(v, s.mQ);
}

TEST(RealVectorState, Update)
{
  Eigen::Vector3d v(1,2,3);
  RealVectorState s(v);

  s.update(std::make_shared<RealVectorState>(Eigen::Vector3d(1,1,1)));

  EXPECT_EQ(Eigen::Vector3d(2,3,4), s.mQ);
}

TEST(SO2State, Constructor)
{
  Eigen::Rotation2D<double> rotation(0);
  SO2State s(Eigen::Rotation2D<double>(0));

  EXPECT_TRUE(rotation.isApprox(s.mRotation));
}

TEST(SO2State, Update)
{
  Eigen::Rotation2D<double> rotation(M_PI/2);
  SO2State s(Eigen::Rotation2D<double>(0));

  s.update(std::make_shared<SO2State>(rotation));

  EXPECT_TRUE(rotation.isApprox(s.mRotation));
}

TEST(SE2State, Constructor)
{
  Eigen::Isometry2d transform;
  transform.setIdentity();
  SE2State s(transform);

  EXPECT_TRUE(transform.isApprox(s.mTransform));
}

TEST(SE2State, Update)
{
  Eigen::Isometry2d identity;
  identity.setIdentity();
  SE2State s(identity);

  Eigen::Isometry2d transform;
  transform.setIdentity();
  transform.translation() = Eigen::Vector2d(1,2);
  transform.linear() = Eigen::Rotation2D<double>(M_PI/2).matrix();

  s.update(std::make_shared<SE2State>(transform));
  EXPECT_TRUE(transform.isApprox(s.mTransform));
}


TEST(SO3State, Constructor)
{
  Eigen::Quaternion<double> quaternion;
  quaternion.setIdentity();
  SO3State s(quaternion);

  EXPECT_TRUE(quaternion.isApprox(s.mRotation));
}

TEST(SO3State, Update)
{
  Eigen::Quaternion<double> quaternion;
  quaternion.setIdentity();
  SO3State s(quaternion);

  /// pi/2 in z-direction
  Eigen::Quaternion<double> quaternion2(0,0,0,1); 
  s.update(std::make_shared<SO3State>(quaternion2));

  EXPECT_TRUE(quaternion2.isApprox(s.mRotation));
}

TEST(SE3State, Constructor)
{
  Eigen::Isometry3d transform;
  transform.setIdentity();
  SE3State s(transform);

  EXPECT_TRUE(transform.isApprox(s.mTransform));
}

TEST(SE3State, Update)
{
  Eigen::Isometry3d identity;
  identity.setIdentity();
  SE3State s(identity);

  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation() = Eigen::Vector3d(1,2,3);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  transform.linear() = rotation;

  s.update(std::make_shared<SE3State>(transform));
  EXPECT_TRUE(transform.isApprox(s.mTransform));
}


TEST(CompoundState, Constructor)
{
  /// SO2State + RealVectorState
  std::vector<std::shared_ptr<State>> states;
  states.push_back(std::make_shared<SO2State>(Eigen::Rotation2D<double>(0)));
  states.push_back(std::make_shared<RealVectorState>(Eigen::Vector3d(1,2,3)));
  CompoundState compoundState(states);

  std::shared_ptr<SO2State> s0 = std::dynamic_pointer_cast<SO2State>(
    compoundState.mComponents.at(0));
  std::shared_ptr<RealVectorState> s1 = std::dynamic_pointer_cast<RealVectorState>(
    compoundState.mComponents.at(1));

  EXPECT_TRUE(Eigen::Rotation2D<double>(0).isApprox(s0->mRotation));
  EXPECT_EQ(Eigen::Vector3d(1,2,3), s1->mQ);

}



TEST(CompoundState, ConstructorNestedComponent)
{
  /// SO2State + RealVectorState
  std::vector<std::shared_ptr<State>> states;
  states.push_back(std::make_shared<SO2State>(Eigen::Rotation2D<double>(0)));
  states.push_back(std::make_shared<RealVectorState>(Eigen::Vector3d(1,2,3)));
  CompoundState compoundState(states);

  CompoundState compoundState2(compoundState);

  std::shared_ptr<SO2State> s0 = std::dynamic_pointer_cast<SO2State>(
    compoundState2.mComponents.at(0));
  std::shared_ptr<RealVectorState> s1 = std::dynamic_pointer_cast<
                                          RealVectorState>(
                                          compoundState2.mComponents.at(1));

  EXPECT_TRUE(Eigen::Rotation2D<double>(0).isApprox(s0->mRotation));
  EXPECT_EQ(Eigen::Vector3d(1,2,3), s1->mQ);

  std::shared_ptr<SO2State> s0_inner = std::dynamic_pointer_cast<SO2State>(
    compoundState.mComponents.at(0));
  std::shared_ptr<RealVectorState> s1_inner = std::dynamic_pointer_cast<
                                              RealVectorState>(
                                              compoundState.mComponents.at(1));

  EXPECT_TRUE(Eigen::Rotation2D<double>(0).isApprox(s0_inner->mRotation));
  EXPECT_EQ(Eigen::Vector3d(1,2,3), s1_inner->mQ);

}