#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>

using aikido::statespace::Rn;

TEST(Rn, Compose)
{
  Rn rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto s2 = rvss.createState();
  s2.setValue(Eigen::Vector3d(2, 3, 4));

  auto out = rvss.createState();
  rvss.compose(s1, s2, out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(3, 5, 7)));
}

TEST(Rn, Identity)
{
  Rn rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto ident = rvss.createState();
  rvss.getIdentity(ident);

  auto out = rvss.createState();
  rvss.compose(s1, ident, out);

  EXPECT_TRUE(out.getValue().isApprox(s1.getValue()));
}

TEST(Rn, Inverse)
{
  Rn rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));
  auto ident = rvss.createState();
  rvss.getIdentity(ident);

  auto inv = rvss.createState();
  rvss.getInverse(s1, inv);

  auto out = rvss.createState();
  rvss.compose(s1, inv, out);

  EXPECT_TRUE(out.getValue().isApprox(ident.getValue()));
}

TEST(Rn, ExpMap)
{
  Rn rvss(3);

  auto out = rvss.createState();
  rvss.expMap(Eigen::Vector3d(1, 2, 3), out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST(Rn, LogMap)
{
  Rn rvss(3);

  auto state = rvss.createState();
  rvss.setValue(state, Eigen::Vector3d(1, 2, 3));

  Eigen::VectorXd out;
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(1, 2, 3)));

  rvss.expMap(Eigen::Vector3d(4, 5, 7), state);
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(4, 5, 7)));
}

TEST(Rn, CopyState)
{
  Rn rvss(4);
  auto source = rvss.createState();
  auto dest = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.copyState(source, dest);
  EXPECT_TRUE(dest.getValue().isApprox(source.getValue()));
}

TEST(Rn, PrintState)
{
  Rn rvss(4);
  auto source = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.print(source, std::cout);
}
