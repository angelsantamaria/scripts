#ifndef VU_UTILS_GTEST_H
#define VU_UTILS_GTEST_H

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

/* 

Macros for testing equalities and inequalities.

  * {ASSERT|EXPECT}_EQ(expected, actual): Tests that expected == actual
  * {ASSERT|EXPECT}_NE(v1, v2):           Tests that v1 != v2
  * {ASSERT|EXPECT}_LT(v1, v2):           Tests that v1 < v2
  * {ASSERT|EXPECT}_LE(v1, v2):           Tests that v1 <= v2
  * {ASSERT|EXPECT}_GT(v1, v2):           Tests that v1 > v2
  * {ASSERT|EXPECT}_GE(v1, v2):           Tests that v1 >= v2

C String Comparisons.  All tests treat NULL and any non-NULL string as different. Two NULLs are equal.

  * {ASSERT|EXPECT}_STREQ(s1, s2):     Tests that s1 == s2
  * {ASSERT|EXPECT}_STRNE(s1, s2):     Tests that s1 != s2
  * {ASSERT|EXPECT}_STRCASEEQ(s1, s2): Tests that s1 == s2, ignoring case
  * {ASSERT|EXPECT}_STRCASENE(s1, s2): Tests that s1 != s2, ignoring case
 
Macros for comparing floating-point numbers.
 
  * {ASSERT|EXPECT}_FLOAT_EQ(expected, actual):
    Tests that two float values are almost equal.
  * {ASSERT|EXPECT}_DOUBLE_EQ(expected, actual):
    Tests that two double values are almost equal.
  * {ASSERT|EXPECT}_NEAR(v1, v2, abs_error):
    Tests that v1 and v2 are within the given distance to each other.
 
These predicate format functions work on floating-point values, and can be used in {ASSERT|EXPECT}_PRED_FORMAT2*(), e.g.
 
  EXPECT_PRED_FORMAT2(testing::DoubleLE, Foo(), 5.0);
 
Macros that execute statement and check that it doesn't generate new fatal failures in the current thread.
 
  * {ASSERT|EXPECT}_NO_FATAL_FAILURE(statement);

Usage :

  TEST(Test, Foo)
  {
    // the following works but prints default stream
    EXPECT_TRUE(false) << "Testing Stream.";
  
    // or you can play with AINSI color code
    EXPECT_TRUE(false) << "\033[1;31m" << "Testing Stream.";
  
    // or use the above defined macros
  
    PRINTF("Hello world");
  
    // or
  
    TEST_COUT << "Hello world";
  }

http://stackoverflow.com/a/29155677
*/

namespace testing
{
namespace internal
{
enum GTestColor
{
  COLOR_DEFAULT,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_YELLOW
};

extern void ColoredPrintf(GTestColor color, const char* fmt, ...);

#define PRINTF(...) \
  do { ColoredPrintf(testing::internal::COLOR_GREEN,\
  "[          ] "); \
  ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } \
  while(0)

// C++ stream interface
class TestCout : public std::stringstream
{
public:
  ~TestCout()
  {
    PRINTF("%s\n", str().c_str());
  }
};


#define TEST_COUT testing::internal::TestCout()

} // namespace internal

/* Macros related to testing Eigen classes:
 */
/*
define EXPECT_MATRIX_APPROX(C_expect, C_actual, precision) EXPECT_PRED2([](const Eigen::MatrixXs lhs, const Eigen::MatrixXs rhs) { \
                  return (lhs - rhs).isMuchSmallerThan(1, precision); \
               }, \
               C_expect, C_actual);

define ASSERT_MATRIX_APPROX(C_expect, C_actual, precision) ASSERT_PRED2([](const Eigen::MatrixXs lhs, const Eigen::MatrixXs rhs) { \
                  return (lhs - rhs).isMuchSmallerThan(1, precision); \
               }, \
               C_expect, C_actual);

define EXPECT_QUATERNION_APPROX(C_expect, C_actual, precision) EXPECT_MATRIX_APPROX((C_expect).coeffs(), (C_actual).coeffs(), precision)

define ASSERT_QUATERNION_APPROX(C_expect, C_actual, precision) ASSERT_MATRIX_APPROX((C_expect).coeffs(), (C_actual).coeffs(), precision)

define EXPECT_POSE2D_APPROX(C_expect, C_actual, precision) EXPECT_PRED2([](const Eigen::MatrixXs lhs, const Eigen::MatrixXs rhs) { \
                   MatrixXs er = lhs - rhs; \
                   er(2) = pi2pi((Scalar)er(2)); \
                   return er.isMuchSmallerThan(1, precision); \
               }, \
               C_expect, C_actual);

define ASSERT_POSE2D_APPROX(C_expect, C_actual, precision) EXPECT_PRED2([](const Eigen::MatrixXs lhs, const Eigen::MatrixXs rhs) { \
                   MatrixXs er = lhs - rhs; \
                   er(2) = pi2pi((Scalar)er(2)); \
                   return er.isMuchSmallerThan(1, precision); \
               }, \
               C_expect, C_actual);
*/

} // namespace testing


#endif /* VU_UTILS_GTEST_H */
