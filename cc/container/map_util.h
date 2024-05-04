#pragma once

// Map container utilities.

#include <map>
#include <unordered_map>

namespace container {

// Creates a sorted map from an unordered map for debugging.
// Parameters:
//   unsorted - Unordered map of <key, value> pairs.
// Returns a sorted map.
template <class KeyT, class ValueT>
std::map<KeyT, ValueT> MapFromUnorderedMap(const std::unordered_map<KeyT, ValueT>& unsorted);

// Implementation.

template <class KeyT, class ValueT>
std::map<KeyT, ValueT> MapFromUnorderedMap(const std::unordered_map<KeyT, ValueT>& unsorted) {
  std::map<KeyT, ValueT> sorted;
  for (const auto& [key, value] : unsorted) {
    sorted[key] = value;
  }
  return sorted;
}

}  // namespace container
