#include "cc/math/vector2.h"

#include <string>

#include "gtest/gtest.h"

namespace math {
namespace {

TEST(Vector2Test, Normal) {
  VectorOwned<double, 2> v0({1.0, 2.0});
  EXPECT_EQ(v0[0], 1.0);
  EXPECT_EQ(v0[1], 2.0);

  VectorOwned<double, 2> v1({3.0, 4.0});
  EXPECT_EQ(v1[0], 3.0);
  EXPECT_EQ(v1[1], 4.0);

  VectorOwned<double, 2> v2;
  v2 = v0;
  EXPECT_EQ(v2[0], 1.0);
  EXPECT_EQ(v2[1], 2.0);

  v2 += v1;
  EXPECT_EQ(v2[0], 4.0);
  EXPECT_EQ(v2[1], 6.0);
  
  v2 -= v0;
  EXPECT_EQ(v2[0], 3.0);
  EXPECT_EQ(v2[1], 4.0);

  v2 = v0;
  v2 *= 5.0;
  EXPECT_EQ(v2[0], 5.0);
  EXPECT_EQ(v2[1], 10.0);

  v2 = v0;
  v2 /= 2.0;
  EXPECT_EQ(v2[0], 0.5);
  EXPECT_EQ(v2[1], 1.0);

  EXPECT_EQ(Magnitude2(v0), 5.0);
  EXPECT_EQ(Magnitude(v0), std::sqrt<double>(5.0));
  EXPECT_EQ(v0.dot(v1), 11.0);
  EXPECT_EQ(CrossSignedZ(v0, v1), -2.0);

  v2 = ZCross(v0);
  EXPECT_EQ(v2[0], -2.0);
  EXPECT_EQ(v2[1], 1.0);

  EXPECT_EQ(VectorToString(v0), "{1, 2}");
}

}  // namespace
}  // namespace math
