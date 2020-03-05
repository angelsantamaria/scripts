#include "utils_gtest.h"

TEST(TestTest, DummyTestExample)
{
  EXPECT_FALSE(false);

  ASSERT_TRUE(true);

  int my_int = 5;

  ASSERT_EQ(my_int, 5);

  std::cout << "All good at TestTest::DummyTestExample !\n" << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
