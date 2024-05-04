#include "cc/math/bounding_box.h"

#include "cc/math/vector.h"
#include "gtest/gtest.h"

namespace math {
namespace {

TEST(BoundingBox2Test, Normal) {
  BoundingBox<double, 2> box1;
  BoundingBox<double, 2> box2;

  EXPECT_TRUE(box1.IsEmpty());
  EXPECT_TRUE(box2.IsEmpty());
  EXPECT_TRUE(box1 == box2);
  EXPECT_FALSE(box1 != box2);

  const VectorOwned<double, 2> v1({1.0, 10.0});
  box1.InsertPoint(v1);
  EXPECT_FALSE(box1.IsEmpty());
  EXPECT_FALSE(box1 == box2);
  EXPECT_TRUE(box1 != box2);
  EXPECT_EQ(box1.Min(), v1);
  EXPECT_EQ(box1.Max(), v1);

  const VectorOwned<double, 2> v3({3.0, 30.0});
  box1.InsertPoint(v3);
  EXPECT_EQ(box1.Min(), v1);
  EXPECT_EQ(box1.Max(), v3);

  const VectorOwned<double, 2> v2({2.0, 20.0});
  box1.InsertPoint(v2);
  EXPECT_EQ(box1.Min(), v1);
  EXPECT_EQ(box1.Max(), v3);

  box2.InsertPoint(v2);
  EXPECT_FALSE(box1 == box2);
  EXPECT_TRUE(box1 != box2);
  EXPECT_EQ(box2.Min(), v2);
  EXPECT_EQ(box2.Max(), v2);

  box2.InsertBoundingBox(box1);
  EXPECT_TRUE(box1 == box2);
  EXPECT_FALSE(box1 != box2);
  EXPECT_EQ(box2.Min(), v1);
  EXPECT_EQ(box2.Max(), v3);
}

}  // namespace
}  // namespace math
