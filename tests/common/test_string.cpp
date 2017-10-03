#include <gtest/gtest.h>
#include <aikido/common/string.hpp>

using namespace aikido;

//==============================================================================
TEST(String, Split)
{
  std::string text = "HERB, the robot.";

  auto res1 = common::split(text); // {"HERB,", "the", "robot."}
  EXPECT_EQ(res1.size(), 3u);
  EXPECT_EQ(res1[0], "HERB,");
  EXPECT_EQ(res1[1], "the");
  EXPECT_EQ(res1[2], "robot.");

  auto res2 = common::split(text, ","); // {"HERB", " the robot."}
  EXPECT_EQ(res2.size(), 2u);
  EXPECT_EQ(res2[0], "HERB");
  EXPECT_EQ(res2[1], " the robot.");

  auto res3 = common::split(text, "to"); // {"HERB, ", "he r", "b", "."}
  EXPECT_EQ(res3.size(), 4u);
  EXPECT_EQ(res3[0], "HERB, ");
  EXPECT_EQ(res3[1], "he r");
  EXPECT_EQ(res3[2], "b");
  EXPECT_EQ(res3[3], ".");
}
