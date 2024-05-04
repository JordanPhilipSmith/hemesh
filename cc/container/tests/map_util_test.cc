#include "cc/container/map_util.h"

#include <map>
#include <unordered_map>

#include "gtest/gtest.h"

namespace container {
namespace {

TEST(MapFromUnorderedMapTest, Empty) {
  const std::map<int, char> expected_char_for_int = {
    {0, 'a'},
    {1, 'b'},
    {2, 'c'}
  };

  std::unordered_map<int, char> unsorted_char_for_int;
  for (const auto& [key, value] : expected_char_for_int) {
    unsorted_char_for_int[key] = value;
  }
  const std::map<int, char> char_for_int = MapFromUnorderedMap<int, char>(unsorted_char_for_int);

  EXPECT_EQ(char_for_int, expected_char_for_int);
}
  
}  // namespace
}  // namespace container
