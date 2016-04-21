#include <gtest/gtest.h>
#include <aikido/statespace/CompoundStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>

using aikido::statespace::CompoundStateSpace;
using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::SO3StateSpace;

TEST(CompoundStateSpace, Compose)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState s2 = space.createState();
  s2.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s2.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(5., 10.));

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, s2, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(8., 14.)));
}

TEST(CompoundStateSpace, Identity)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, ident, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(3., 4.)));
}

TEST(CompoundStateSpace, Inverse)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CompoundStateSpace::ScopedState inv = space.createState();
  space.getInverse(s1, inv);

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, inv, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  const double iout1 = ident.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(iout1, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  const Vector2d iout2 =
      ident.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(iout2));
}

TEST(CompoundStateSpace, ExpMap)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState out = space.createState();

  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(1, 2)));
}

TEST(CompoundStateSpace, LogMap)
{
  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState state = space.createState();

  Eigen::VectorXd out;
  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), state);
  space.logMap(state, out);

  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, 1, 2)));
}

TEST(CompoundStateSpace, CopyState)
{
  CompoundStateSpace space({
      std::make_shared<SO2StateSpace>(),
      std::make_shared<RealVectorStateSpace>(3),
      std::make_shared<SO3StateSpace>(),
  });

  auto source = space.createState();
  auto dest = space.createState();

  double angle = M_PI;
  auto rv = Eigen::Vector3d(3, 4, 5);
  auto quat =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  source.getSubStateHandle<SO2StateSpace>(0).setAngle(angle);
  source.getSubStateHandle<RealVectorStateSpace>(1).setValue(rv);
  source.getSubStateHandle<SO3StateSpace>(2).setQuaternion(quat);

  space.copyState(source, dest);

  const double out1 = dest.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(angle, out1);

  auto out2 = dest.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(rv));

  auto out3 = dest.getSubStateHandle<SO3StateSpace>(2).getQuaternion();
  EXPECT_TRUE(out3.isApprox(quat));
}
