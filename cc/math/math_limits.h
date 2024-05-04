#pragma once

// Mathematical limit values.

#include <limits>

namespace math {

template<class ScalarT>
struct MathLimits {
  // Smallest finite representable value.
  static const ScalarT kMin;
  // Largest finite representable value.
  static const ScalarT kMax;
  // Smallest finite representable value that is greater than zero.
  static const ScalarT kPosMin;
  // Largest finite representable value.
  static const ScalarT kPosMax;
  // Positive infinity (or largest finite representable value if infinity is not possible).
  static const ScalarT kPosInfinity;
  // Negative infinity (or smallest finite representable value if infinity is not possible).
  static const ScalarT kNegInfinity;
};  // struct MathLimits

template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kMin = std::numeric_limits<ScalarT>::lowest();
template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kMax = std::numeric_limits<ScalarT>::max();
template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kPosMin = std::numeric_limits<ScalarT>::min();
template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kPosMax = kMax;
template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kPosInfinity = std::numeric_limits<ScalarT>::infinity();
template<class ScalarT>
const ScalarT MathLimits<ScalarT>::kNegInfinity = -kPosInfinity;

// Template specialization for std::int32_t.
template<>
struct MathLimits<std::int32_t> {
  static const std::int32_t kMin;
  static const std::int32_t kMax;
  static const std::int32_t kPosMin;
  static const std::int32_t kPosMax;
  static const std::int32_t kPosInfinity;
  static const std::int32_t kNegInfinity;
};  // struct MathLimits

const std::int32_t MathLimits<std::int32_t>::kMin = 0x80000000;
const std::int32_t MathLimits<std::int32_t>::kMax = 0x7fffffff;
const std::int32_t MathLimits<std::int32_t>::kPosMin = 1;
const std::int32_t MathLimits<std::int32_t>::kPosMax = kMax;
const std::int32_t MathLimits<std::int32_t>::kPosInfinity = kMax;
const std::int32_t MathLimits<std::int32_t>::kNegInfinity = kMin;
 
// Template specialization for std::uint32_t.
template<>
struct MathLimits<std::uint32_t> {
  static const std::uint32_t kMin;
  static const std::uint32_t kMax;
  static const std::uint32_t kPosMin;
  static const std::uint32_t kPosMax;
  static const std::uint32_t kPosInfinity;
  static const std::uint32_t kNegInfinity;
};  // struct MathLimits

const std::uint32_t MathLimits<std::uint32_t>::kMin = 0;
const std::uint32_t MathLimits<std::uint32_t>::kMax = 0xffffffff;
const std::uint32_t MathLimits<std::uint32_t>::kPosMin = 1;
const std::uint32_t MathLimits<std::uint32_t>::kPosMax = kMax;
const std::uint32_t MathLimits<std::uint32_t>::kPosInfinity = kMax;
const std::uint32_t MathLimits<std::uint32_t>::kNegInfinity = kMin;
 
// Template specialization for std::int64_t.
template<>
struct MathLimits<std::int64_t> {
  static const std::int64_t kMin;
  static const std::int64_t kMax;
  static const std::int64_t kPosMin;
  static const std::int64_t kPosMax;
  static const std::int64_t kPosInfinity;
  static const std::int64_t kNegInfinity;
};  // struct MathLimits

const std::int64_t MathLimits<std::int64_t>::kMin = 0x8000000000000000;
const std::int64_t MathLimits<std::int64_t>::kMax = 0x7fffffffffffffff;
const std::int64_t MathLimits<std::int64_t>::kPosMin = 1;
const std::int64_t MathLimits<std::int64_t>::kPosMax = kMax;
const std::int64_t MathLimits<std::int64_t>::kPosInfinity = kMax;
const std::int64_t MathLimits<std::int64_t>::kNegInfinity = kMin;
 
// Template specialization for std::uint64_t.
template<>
struct MathLimits<std::uint64_t> {
  static const std::uint64_t kMin;
  static const std::uint64_t kMax;
  static const std::uint64_t kPosMin;
  static const std::uint64_t kPosMax;
  static const std::uint64_t kPosInfinity;
  static const std::uint64_t kNegInfinity;
};  // struct MathLimits

const std::uint64_t MathLimits<std::uint64_t>::kMin = 0;
const std::uint64_t MathLimits<std::uint64_t>::kMax = 0xffffffffffffffff;
const std::uint64_t MathLimits<std::uint64_t>::kPosMin = 1;
const std::uint64_t MathLimits<std::uint64_t>::kPosMax = kMax;
const std::uint64_t MathLimits<std::uint64_t>::kPosInfinity = kMax;
const std::uint64_t MathLimits<std::uint64_t>::kNegInfinity = kMin;
 
}  // namespace math
