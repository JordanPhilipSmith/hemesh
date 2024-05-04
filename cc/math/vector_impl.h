#pragma once

#include <Eigen/Dense>
#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace math {

template <class ScalarT, int dim>
using VectorOwned = Eigen::Matrix<ScalarT, dim, 1>;  

template <class ScalarT, int dim>
using ConstVector = Eigen::Matrix<ScalarT, dim, 1>;  

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
