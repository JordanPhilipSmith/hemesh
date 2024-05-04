#pragma once

// Predicates used to calculate the Delaunay triangulation.

#include <memory>

#include "cc/math/vector.h"

namespace geom {

// Predicates used to calculate the Delaunay triangulation.
template <class ScalarT>
class DelaunayPredicates {
 public:
  virtual ~DelaunayPredicates() = default;

  // Factory method using a local origin for calculations with normal precision scalars.
  static std::unique_ptr<DelaunayPredicates> NewLocalOrigin();
  
  // The counterclockwise (CCW) inscribed circle methods determine whether a circle can be inscribed
  // tangent to three sites.
  // Return values:
  //   < 0 - A circle cannot be inscribed.
  //   = 0 - The sites are collinear.
  //   > 0 - A circle can be inscribed.

  // Determines whether a circle can be inscribed tangent to three vertex sites.
  // Parameters:
  //   v0 - V0.
  //   v1 - V1.
  //   v2 - V2.
  virtual ScalarT CCWInscribedCircle_VVV(
      const math::ConstVector<ScalarT, 2>& v0,
      const math::ConstVector<ScalarT, 2>& v1,
      const math::ConstVector<ScalarT, 2>& v2) const = 0;

  // The in inscribed circle methods determine whether a fourth site is inside, on, or outside a
  // circle inscribed tangent to three sites.
  // Return values:
  //   < 0 - The fourth site penetrates inside the inscribed circle.
  //   = 0 - The fourth site touches the boundary of the inscribed circle.
  //   > 0 - The fourth site is disjoint from the inscribed circle.

  // Determines whether a fourth vertex site is inside the circle inscribed tangent to three vertex
  // sites.
  // Parameters:
  //   v0 - V0.
  //   v1 - V1.
  //   v2 - V2.
  //   v3 - V3 to test for inside.
  virtual ScalarT InInscribedCircle_VVV_V(
      const math::ConstVector<ScalarT, 2>& v0,
      const math::ConstVector<ScalarT, 2>& v1,
      const math::ConstVector<ScalarT, 2>& v2,
      const math::ConstVector<ScalarT, 2>& v3) const = 0;
};  // class DelaunayPredicates

namespace internal {

// DelaunayPredicates local origin implementation.
template <class ScalarT>
class DelaunayPredicatesLocalOrigin : public virtual DelaunayPredicates<ScalarT> {
 public:
  DelaunayPredicatesLocalOrigin() = default;
  virtual ~DelaunayPredicatesLocalOrigin() = default;

  ScalarT CCWInscribedCircle_VVV(
      const math::ConstVector<ScalarT, 2>& v0,
      const math::ConstVector<ScalarT, 2>& v1,
      const math::ConstVector<ScalarT, 2>& v2) const override {
    const math::ConstVector<ScalarT, 2>& origin = v0;
    math::VectorOwned<ScalarT, 2> dv1(v1);
    dv1 -= origin;
    math::VectorOwned<ScalarT, 2> dv2(v2);
    dv2 -= origin;
    return math::CrossSignedZ(dv1, dv2);
  }  

  ScalarT InInscribedCircle_VVV_V(
      const math::ConstVector<ScalarT, 2>& v0,
      const math::ConstVector<ScalarT, 2>& v1,
      const math::ConstVector<ScalarT, 2>& v2,
      const math::ConstVector<ScalarT, 2>& v3) const override {
    static const ScalarT kZero(0);
    static const ScalarT kOne(1);

    const math::ConstVector<ScalarT, 2>& origin = v0;
    math::VectorOwned<ScalarT, 2> dv1 = v1;
    dv1 -= origin;
    math::VectorOwned<ScalarT, 2> dv2 = v2;
    dv2 -= origin;

    // 0 ?> (|v1|^2 |v2 x v3| + |v2|^2 |v3 x v1| + |v3|^2 |v1 x v2|) / |v1 x v2|

    const ScalarT v1_x_v2 = math::CrossSignedZ(dv1, dv2);
    if (v1_x_v2 == kZero) {
      // Degenerate collinear V0, V1, V2, the radius is infinite, so return inside.
      return -kOne;
    }

    math::VectorOwned<ScalarT, 2> dv3 = v3;
    dv3 -= origin;

    const ScalarT v1_mag2 = math::Magnitude2(dv1);
    const ScalarT v2_mag2 = math::Magnitude2(dv2);
    const ScalarT v3_mag2 = math::Magnitude2(dv3);
    const ScalarT v2_x_v3 = math::CrossSignedZ(dv2, dv3);
    const ScalarT v3_x_v1 = math::CrossSignedZ(dv3, dv1);

    return (v1_mag2 * v2_x_v3 + v2_mag2 * v3_x_v1 + v3_mag2 * v1_x_v2) / v1_x_v2;
  }
};  // class DelaunayPredicatesLocalOrigin

}  // namespace internal

template <class ScalarT>
std::unique_ptr<DelaunayPredicates<ScalarT>> DelaunayPredicates<ScalarT>::NewLocalOrigin() {
  return std::make_unique<internal::DelaunayPredicatesLocalOrigin<ScalarT>>();
}

}  // namespace geom
