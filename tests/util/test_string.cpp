#include <gtest/gtest.h>
#include <aikido/util/string.hpp>

using namespace aikido;

//==============================================================================
TEST(String, Split)
{
  std::string text = "HERB, the robot.";

  auto res1 = util::split(text); // {"HERB,", "the", "robot."}
  EXPECT_EQ(res1.size(), 3u);
  EXPECT_EQ(res1[0], "HERB,");
  EXPECT_EQ(res1[1], "the");
  EXPECT_EQ(res1[2], "robot.");

  auto res2 = util::split(text, ","); // {"HERB", " the robot."}
  EXPECT_EQ(res2.size(), 2u);
  EXPECT_EQ(res2[0], "HERB");
  EXPECT_EQ(res2[1], " the robot.");
}
