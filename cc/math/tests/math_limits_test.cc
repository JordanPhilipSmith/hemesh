#include "cc/math/math_limits.h"

#include "gtest/gtest.h"

namespace math {
namespace {

// Two's complement.
//  1 = 0x0001
//      0x1110
//      +    1
// -1 = 0x1111

TEST(MathLimitsTest, Int32) {
  EXPECT_EQ(MathLimits<std::int32_t>::kMin, 0x80000000);
  EXPECT_EQ(MathLimits<std::int32_t>::kMax, 0x7fffffff);
  EXPECT_EQ(MathLimits<std::int32_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::int32_t>::kPosMax, MathLimits<std::int32_t>::kMax);
  EXPECT_EQ(MathLimits<std::int32_t>::kPosInfinity, MathLimits<std::int32_t>::kMax);
  EXPECT_EQ(MathLimits<std::int32_t>::kNegInfinity, MathLimits<std::int32_t>::kMin);

  EXPECT_LT(MathLimits<std::int32_t>::kMin, 0);
  EXPECT_GT(MathLimits<std::int32_t>::kPosMin, 0);
  EXPECT_LE(MathLimits<std::int32_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::int32_t>::kPosInfinity, MathLimits<std::int32_t>::kMax);
  EXPECT_EQ(MathLimits<std::int32_t>::kNegInfinity, MathLimits<std::int32_t>::kMin);
}

TEST(MathLimitsTest, Uint32) {
  EXPECT_EQ(MathLimits<std::uint32_t>::kMin, 0);
  EXPECT_EQ(MathLimits<std::uint32_t>::kMax, 0xffffffff);
  EXPECT_EQ(MathLimits<std::uint32_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::uint32_t>::kPosMax, MathLimits<std::uint32_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint32_t>::kPosInfinity, MathLimits<std::uint32_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint32_t>::kNegInfinity, MathLimits<std::uint32_t>::kMin);

  EXPECT_GT(MathLimits<std::uint32_t>::kPosMin, 0);
  EXPECT_LE(MathLimits<std::uint32_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::uint32_t>::kPosInfinity, MathLimits<std::uint32_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint32_t>::kNegInfinity, MathLimits<std::uint32_t>::kMin);
}

TEST(MathLimitsTest, Int64) {
  EXPECT_EQ(MathLimits<std::int64_t>::kMin, 0x8000000000000000);
  EXPECT_EQ(MathLimits<std::int64_t>::kMax, 0x7fffffffffffffff);
  EXPECT_EQ(MathLimits<std::int64_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::int64_t>::kPosMax, MathLimits<std::int64_t>::kMax);
  EXPECT_EQ(MathLimits<std::int64_t>::kPosInfinity, MathLimits<std::int64_t>::kMax);
  EXPECT_EQ(MathLimits<std::int64_t>::kNegInfinity, MathLimits<std::int64_t>::kMin);

  EXPECT_LT(MathLimits<std::int64_t>::kMin, 0);
  EXPECT_GT(MathLimits<std::int64_t>::kPosMin, 0);
  EXPECT_LE(MathLimits<std::int64_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::int64_t>::kPosInfinity, MathLimits<std::int64_t>::kMax);
  EXPECT_EQ(MathLimits<std::int64_t>::kNegInfinity, MathLimits<std::int64_t>::kMin);
}

TEST(MathLimitsTest, Uint64) {
  EXPECT_EQ(MathLimits<std::uint64_t>::kMin, 0);
  EXPECT_EQ(MathLimits<std::uint64_t>::kMax, 0xffffffffffffffff);
  EXPECT_EQ(MathLimits<std::uint64_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::uint64_t>::kPosMax, MathLimits<std::uint64_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint64_t>::kPosInfinity, MathLimits<std::uint64_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint64_t>::kNegInfinity, MathLimits<std::uint64_t>::kMin);

  EXPECT_GT(MathLimits<std::uint64_t>::kPosMin, 0);
  EXPECT_LE(MathLimits<std::uint64_t>::kPosMin, 1);
  EXPECT_EQ(MathLimits<std::uint64_t>::kPosInfinity, MathLimits<std::uint64_t>::kMax);
  EXPECT_EQ(MathLimits<std::uint64_t>::kNegInfinity, MathLimits<std::uint64_t>::kMin);
}

TEST(MathLimitsTest, Double) {
  EXPECT_EQ(MathLimits<double>::kMin, std::numeric_limits<double>::lowest());
  EXPECT_EQ(MathLimits<double>::kMax, std::numeric_limits<double>::max());
  EXPECT_EQ(MathLimits<double>::kPosMin, std::numeric_limits<double>::min());
  EXPECT_EQ(MathLimits<double>::kPosMax, MathLimits<double>::kMax);
  EXPECT_EQ(MathLimits<double>::kPosInfinity, std::numeric_limits<double>::infinity());
  EXPECT_EQ(MathLimits<double>::kNegInfinity, -std::numeric_limits<double>::infinity());

  EXPECT_LT(MathLimits<double>::kMin, 0);
  EXPECT_GT(MathLimits<double>::kPosMin, 0);
  EXPECT_LT(MathLimits<double>::kPosMin, 1);
  EXPECT_GT(MathLimits<double>::kPosInfinity, MathLimits<double>::kMax);
  EXPECT_LT(MathLimits<double>::kNegInfinity, MathLimits<double>::kMin);
}
  

}  // namespace
}  // namespace math
