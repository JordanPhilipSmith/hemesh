#pragma once

#include "cc/math/vector_impl.h"

namespace math {

template <class ScalarT>
ScalarT CrossSignedZ(const ConstVector<ScalarT, 2>& v0,
				     const ConstVector<ScalarT, 2>& v1) {
  return v0[0] * v1[1] - v0[1] * v1[0];
}

template <class ScalarT>
VectorOwned<ScalarT, 2> ZCross(const ConstVector<ScalarT, 2>& v) {
  return VectorOwned<ScalarT, 2>({-v[1], v[0]});
}

}  // namespace math
