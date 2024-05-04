#include "cc/geom/delaunay_predicates.h"

#include <memory>

#include "cc/math/vector.h"
#include "gtest/gtest.h"

namespace geom {
namespace {

TEST(DelaunayPredicatesTest, CCWInscribedCircle_VVV) {
  std::unique_ptr<DelaunayPredicates<double>> preds = DelaunayPredicates<double>::NewLocalOrigin();

  const math::VectorOwned<double, 2> origin({1.0, 2.0});
  const math::VectorOwned<double, 2> x({4.0, 3.0});
  const math::VectorOwned<double, 2> y({-3.0, 4.0});
  const math::VectorOwned<double, 2> v0 = origin;
  const math::VectorOwned<double, 2> v1 = origin + x;
  const math::VectorOwned<double, 2> v2 = origin + y;
  const math::VectorOwned<double, 2> v3 = origin - x;

  // CW.
  EXPECT_LT(preds->CCWInscribedCircle_VVV(v0, v2, v1), 0.0);

  // Collinear.
  EXPECT_EQ(preds->CCWInscribedCircle_VVV(v0, v0, v0), 0.0);
  EXPECT_EQ(preds->CCWInscribedCircle_VVV(v0, v0, v1), 0.0);
  EXPECT_EQ(preds->CCWInscribedCircle_VVV(v0, v1, v3), 0.0);

  // CCW.
  EXPECT_GT(preds->CCWInscribedCircle_VVV(v0, v1, v2), 0.0);
}

TEST(DelaunayPredicatesTest, InInscribedCircle_VVV_V) {
  std::unique_ptr<DelaunayPredicates<double>> preds = DelaunayPredicates<double>::NewLocalOrigin();

  const math::VectorOwned<double, 2> origin({1.0, 2.0});
  const math::VectorOwned<double, 2> x({4.0, 3.0});
  const math::VectorOwned<double, 2> y({-3.0, 4.0});
  const math::VectorOwned<double, 2> v0 = origin - y;
  const math::VectorOwned<double, 2> v1 = origin + x;
  const math::VectorOwned<double, 2> v2 = origin + y;
  const math::VectorOwned<double, 2> v3_in = origin;
  const math::VectorOwned<double, 2> v3_on = origin - x;
  const math::VectorOwned<double, 2> v3_out = origin - 2.0 * x;

  // Collinear points are always considered inside.
  EXPECT_LT(preds->InInscribedCircle_VVV_V(v0, origin, v2, v3_out), 0.0);

  // Inside.
  EXPECT_LT(preds->InInscribedCircle_VVV_V(v0, v1, v2, v3_in), 0.0);

  // On.
  EXPECT_EQ(preds->InInscribedCircle_VVV_V(v0, v1, v2, v3_on), 0.0);

  // Outside.
  EXPECT_GT(preds->InInscribedCircle_VVV_V(v0, v1, v2, v3_out), 0.0);
}

}  // namespace
}  // namespace geom
