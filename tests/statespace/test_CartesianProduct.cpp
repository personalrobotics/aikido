#include <dart/math/Helpers.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE2.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>

using aikido::statespace::CartesianProduct;
using aikido::statespace::R2;
using aikido::statespace::R3;
using aikido::statespace::SO2;
using aikido::statespace::SO3;
using aikido::statespace::SE2;

TEST(CartesianProduct, Clone)
{
  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  for (auto i = 0u; i < 5u; ++i)
  {
    auto s1 = space.createState();
    const auto angle = dart::math::random(-M_PI, M_PI);
    s1.getSubStateHandle<SO2>(0).fromAngle(angle);
    s1.getSubStateHandle<R2>(1).setValue(Eigen::Vector2d::Random());

    auto s2 = s1.clone();

    EXPECT_DOUBLE_EQ(
        s1.getSubStateHandle<SO2>(0).toAngle(),
        s2.getSubStateHandle<SO2>(0).toAngle());
    EXPECT_TRUE(
        s1.getSubStateHandle<R2>(1).getValue().isApprox(
            s2.getSubStateHandle<R2>(1).getValue()));
  }
}

TEST(CartesianProduct, Compose)
{
  using Eigen::Vector2d;

  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  CartesianProduct::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2>(0).fromAngle(M_PI_2);
  s1.getSubStateHandle<R2>(1).setValue(Vector2d(3., 4.));

  CartesianProduct::ScopedState s2 = space.createState();
  s2.getSubStateHandle<SO2>(0).fromAngle(M_PI_2);
  s2.getSubStateHandle<R2>(1).setValue(Vector2d(5., 10.));

  CartesianProduct::ScopedState out = space.createState();
  space.compose(s1, s2, out);

  const double out1 = out.getSubStateHandle<SO2>(0).toAngle();
  EXPECT_DOUBLE_EQ(M_PI, out1);

  const Vector2d out2 = out.getSubStateHandle<R2>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(8., 14.)));
}

TEST(CartesianProduct, Identity)
{
  using Eigen::Vector2d;

  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  CartesianProduct::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2>(0).fromAngle(M_PI_2);
  s1.getSubStateHandle<R2>(1).setValue(Vector2d(3., 4.));

  CartesianProduct::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CartesianProduct::ScopedState out = space.createState();
  space.compose(s1, ident, out);

  const double out1 = out.getSubStateHandle<SO2>(0).toAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 = out.getSubStateHandle<R2>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(3., 4.)));
}

TEST(CartesianProduct, Inverse)
{
  using Eigen::Vector2d;

  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  CartesianProduct::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2>(0).fromAngle(M_PI_2);
  s1.getSubStateHandle<R2>(1).setValue(Vector2d(3., 4.));

  CartesianProduct::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CartesianProduct::ScopedState inv = space.createState();
  space.getInverse(s1, inv);

  CartesianProduct::ScopedState out = space.createState();
  space.compose(s1, inv, out);

  const double out1 = out.getSubStateHandle<SO2>(0).toAngle();
  const double iout1 = ident.getSubStateHandle<SO2>(0).toAngle();
  EXPECT_DOUBLE_EQ(iout1, out1);

  const Vector2d out2 = out.getSubStateHandle<R2>(1).getValue();
  const Vector2d iout2 = ident.getSubStateHandle<R2>(1).getValue();
  EXPECT_TRUE(out2.isApprox(iout2));
}

TEST(CartesianProduct, ExpMap)
{
  using Eigen::Vector2d;

  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  CartesianProduct::ScopedState out = space.createState();

  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), out);

  const double out1 = out.getSubStateHandle<SO2>(0).toAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 = out.getSubStateHandle<R2>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(1, 2)));
}

TEST(CartesianProduct, LogMap)
{
  CartesianProduct space({std::make_shared<SO2>(), std::make_shared<R2>()});

  CartesianProduct::ScopedState state = space.createState();

  Eigen::VectorXd out;
  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), state);
  space.logMap(state, out);

  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, 1, 2)));
}

TEST(CartesianProduct, CopyState)
{
  CartesianProduct space(
      {
          std::make_shared<SO2>(),
          std::make_shared<R3>(),
          std::make_shared<SO3>(),
      });

  auto source = space.createState();
  auto dest = space.createState();

  double angle = M_PI;
  auto rv = Eigen::Vector3d(3, 4, 5);
  auto quat
      = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  source.getSubStateHandle<SO2>(0).fromAngle(angle);
  source.getSubStateHandle<R3>(1).setValue(rv);
  source.getSubStateHandle<SO3>(2).setQuaternion(quat);

  space.copyState(source, dest);

  const double out1 = dest.getSubStateHandle<SO2>(0).toAngle();
  EXPECT_DOUBLE_EQ(angle, out1);

  auto out2 = dest.getSubStateHandle<R3>(1).getValue();
  EXPECT_TRUE(out2.isApprox(rv));

  auto out3 = dest.getSubStateHandle<SO3>(2).getQuaternion();
  EXPECT_TRUE(out3.isApprox(quat));
}

TEST(CartesianProduct, PrintState)
{
  CartesianProduct space(
      {
          std::make_shared<SO2>(),
          std::make_shared<R3>(),
          std::make_shared<SO3>(),
          std::make_shared<SE2>(),
          //      std::make_shared<SE3>(),
      });

  auto source = space.createState();

  double angle = M_PI;
  auto rv = Eigen::Vector3d(3, 4, 5);
  auto quat
      = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  auto pose1 = Eigen::Isometry2d::Identity();
  pose1.rotate(Eigen::Rotation2Dd(M_PI_2));
  pose1.translation() << 2, 3;

  source.getSubStateHandle<SO2>(0).fromAngle(angle);
  source.getSubStateHandle<R3>(1).setValue(rv);
  source.getSubStateHandle<SO3>(2).setQuaternion(quat);
  source.getSubStateHandle<SE2>(3).setIsometry(pose1);

  std::cout.precision(3);
  space.print(source, std::cout);
}
