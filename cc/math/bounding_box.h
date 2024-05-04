#pragma once

// Axis aligned bounding box.

#include "cc/math/math_limits.h"
#include "cc/math/vector.h"

namespace math {

template <class ScalarT, int dim>
class BoundingBox {
 public:
  // Constructor for empty bounding box.
  BoundingBox() {
    for (int i = 0; i < dim; ++i) {
      this->min_[i] = MathLimits<ScalarT>::kPosInfinity;
      this->max_[i] = MathLimits<ScalarT>::kNegInfinity;
    }
  }

  bool IsEmpty() const {
    return this->max_[0] < this->min_[0];
  }

  const ConstVector<ScalarT, dim>& Min() const {
    return this->min_;
  }
  
  const ConstVector<ScalarT, dim>& Max() const {
    return this->max_;
  }
  
  bool operator==(const BoundingBox& box) const {
    return (this->IsEmpty() && box.IsEmpty()) || (this->min_ == box.min_ && this->max_ == box.max_);
  }

  bool operator!=(const BoundingBox& box) const {
    return !(*this == box);
  }
  
  void InsertPoint(const ConstVector<ScalarT, dim>& v) {
    for (int i = 0; i < dim; ++i) {
      this->min_[i] = std::min<ScalarT>(this->min_[i], v[i]);
      this->max_[i] = std::max<ScalarT>(this->max_[i], v[i]);
    }
  }

  void InsertBoundingBox(const BoundingBox<ScalarT, dim>& box) {
    for (int i = 0; i < dim; ++i) {
      this->min_[i] = std::min<ScalarT>(this->min_[i], box.min_[i]);
      this->max_[i] = std::max<ScalarT>(this->max_[i], box.max_[i]);
    }
  }

 private:
  VectorOwned<ScalarT, dim> min_;
  VectorOwned<ScalarT, dim> max_;
};  // class BoundingBox

}  // namespace math
