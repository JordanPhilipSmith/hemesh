#pragma once

#include <Eigen/Dense>
#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace math {

template <class ScalarT, int dim>
using Vector = Eigen::Matrix<ScalarT, dim, 1>;  

template <class ScalarT, int dim>
using VectorOwned = Eigen::Matrix<ScalarT, dim, 1>;  

template <class ScalarT, int dim>
using ConstVector = Eigen::Matrix<ScalarT, dim, 1>;  

// Read only subvector view.
// Template parameters:
//   subdim - Subvector dimensions.
//   ScalarT - Scalar value type.
//   dim - Full vector dimensions.
// Parameters:
//   v - Full vector.
//   begin_row - First row to appear in the subvector.
// Returns a read only view of a subvector of rows in [begin_row, begin_row + subdim).
template<int subdim, class ScalarT, int dim>
const auto Subvector(
    const ConstVector<ScalarT, dim>& v, int begin_row) {
  return v.template block<subdim, 1>(begin_row, 0);
}

// Mutable subvector view.
// Template parameters:
//   subdim - Subvector dimensions.
//   ScalarT - Scalar value type.
//   dim - Full vector dimensions.
// Parameters:
//   v - Full vector.
//   begin_row - First row to appear in the subvector.
// Returns a mutable view of a subvector of rows in [begin_row, begin_row + subdim).
template<int subdim, class ScalarT, int dim>
auto Subvector(Vector<ScalarT, dim>& v, int begin_row) {
  return v.template block<subdim, 1>(begin_row, 0);
}

template <class ScalarT, int dim>
ScalarT Magnitude2(const ConstVector<ScalarT, dim>& v) {
  return v.dot(v);
}

template <class ScalarT, int dim>
ScalarT Magnitude(const ConstVector<ScalarT, dim>& v) {
  return std::sqrt<ScalarT>(Magnitude2(v));
}

template <class ScalarT, int dim>
std::string VectorToString(const ConstVector<ScalarT, dim>& v) {
  std::string buffer;
  absl::StrAppend(&buffer, "{");
  for (int i = 0; i < dim; ++i) {
    if (0 < i) {
      absl::StrAppend(&buffer, ", ");
    }
    absl::StrAppendFormat(&buffer, "%.18g", v[i]);
  }
  absl::StrAppend(&buffer, "}");
  return buffer;
}

}  // namespace math
