#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>

using aikido::statespace::R3;
using aikido::statespace::Rn;
using R4 = aikido::statespace::R<4>;

//==============================================================================
TEST(Rn, CloneR3)
{
  R3 rvss;

  for (auto i = 0u; i < 5u; ++i)
  {
    const Eigen::Vector3d pos = Eigen::Vector3d::Random();

    auto s1 = rvss.createState();
    s1.setValue(pos);

    auto s2 = s1.clone();

    EXPECT_TRUE(s1.getValue().isApprox(s2.getValue()));
  }
}

//==============================================================================
TEST(Rn, CloneRx)
{
  Rn rvss(3);

  for (auto i = 0u; i < 5u; ++i)
  {
    const Eigen::Vector3d pos = Eigen::Vector3d::Random();

    auto s1 = rvss.createState();
    s1.setValue(pos);

    auto s2 = s1.clone();

    EXPECT_TRUE(s1.getValue().isApprox(s2.getValue()));
  }
}

//==============================================================================
TEST(Rn, ComposeR3)
{
  R3 rvss;

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto s2 = rvss.createState();
  s2.setValue(Eigen::Vector3d(2, 3, 4));

  auto out1 = rvss.createState();
  rvss.compose(s1, s2, out1);
  EXPECT_TRUE(out1.getValue().isApprox(Eigen::Vector3d(3, 5, 7)));

  auto out2 = rvss.createState();
  rvss.copyState(s1, out2);
  rvss.compose(out2, s2);
  EXPECT_TRUE(out2.getValue().isApprox(Eigen::Vector3d(3, 5, 7)));
}

//==============================================================================
TEST(Rn, ComposeRx)
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

//==============================================================================
TEST(Rn, IdentityR3)
{
  R3 rvss;

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto ident = rvss.createState();
  rvss.getIdentity(ident);

  auto out = rvss.createState();
  rvss.compose(s1, ident, out);

  EXPECT_TRUE(out.getValue().isApprox(s1.getValue()));
}

//==============================================================================
TEST(Rn, IdentityRx)
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

//==============================================================================
TEST(Rn, InverseR3)
{
  R3 rvss;

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

//==============================================================================
TEST(Rn, InverseRx)
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

//==============================================================================
TEST(Rn, ExpMapR3)
{
  R3 rvss;

  auto out = rvss.createState();
  rvss.expMap(Eigen::Vector3d(1, 2, 3), out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(1, 2, 3)));
}

//==============================================================================
TEST(Rn, ExpMapRx)
{
  Rn rvss(3);

  auto out = rvss.createState();
  rvss.expMap(Eigen::Vector3d(1, 2, 3), out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(1, 2, 3)));
}

//==============================================================================
TEST(Rn, LogMapR3)
{
  R3 rvss;

  auto state = rvss.createState();
  rvss.setValue(state, Eigen::Vector3d(1, 2, 3));

  Eigen::VectorXd out;
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(1, 2, 3)));

  rvss.expMap(Eigen::Vector3d(4, 5, 7), state);
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(4, 5, 7)));
}

//==============================================================================
TEST(Rn, LogMapRx)
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

//==============================================================================
TEST(Rn, CopyStateR4)
{
  R4 rvss;
  auto source = rvss.createState();
  auto dest = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.copyState(source, dest);
  EXPECT_TRUE(dest.getValue().isApprox(source.getValue()));
}

//==============================================================================
TEST(Rn, CopyStateRx)
{
  Rn rvss(4);
  auto source = rvss.createState();
  auto dest = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.copyState(source, dest);
  EXPECT_TRUE(dest.getValue().isApprox(source.getValue()));
}

//==============================================================================
TEST(Rn, PrintStateR4)
{
  R4 rvss;
  auto source = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.print(source, std::cout);
}

//==============================================================================
TEST(Rn, PrintStateRx)
{
  Rn rvss(4);
  auto source = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.print(source, std::cout);
}
