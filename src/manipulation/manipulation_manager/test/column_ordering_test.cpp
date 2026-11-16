#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/column_ordering.hpp"

namespace manipulation_manager
{

namespace
{
ColumnEnds col(double y, double z0, double z1)
{
  ColumnEnds c;
  c.start = {0.0, y, z0};
  c.end = {0.0, y, z1};
  return c;
}
}  // namespace

TEST(ColumnOrderingTest, AlreadyOrderedStaysInInputOrder)
{
  // up, down, up: the end of one is at the start of the next
  const std::vector<ColumnEnds> cols{col(0.0, 0.0, 1.0), col(0.1, 1.0, 0.0), col(0.2, 0.0, 1.0)};
  const auto order = orderColumnsByTime(cols, cols[0].start);
  EXPECT_EQ(order, (std::vector<size_t>{0, 1, 2}));
}

TEST(ColumnOrderingTest, ShuffledPlanGetsTheNearestNext)
{
  const std::vector<ColumnEnds> cols{col(0.0, 0.0, 1.0), col(0.4, 1.0, 0.0), col(0.2, 1.0, 0.0), col(0.6, 0.0, 1.0)};
  const auto order = orderColumnsByTime(cols, cols[0].start, false);
  EXPECT_EQ(order, (std::vector<size_t>{0, 2, 3, 1}));
  EXPECT_LT(
    orderTravelTime(cols, order, cols[0].start), orderTravelTime(cols, {0, 1, 2, 3}, cols[0].start));
}

TEST(ColumnOrderingTest, ResultIsAPermutation)
{
  std::vector<ColumnEnds> cols;
  for (int i = 0; i < 7; ++i) {
    cols.push_back(col(0.05 * ((i * 5) % 7), (i % 2) ? 1.0 : 0.0, (i % 2) ? 0.0 : 1.0));
  }
  for (const bool two_opt : {false, true}) {
    auto order = orderColumnsByTime(cols, cols[0].start, two_opt);
    ASSERT_EQ(order.size(), cols.size());
    std::sort(order.begin(), order.end());
    for (size_t i = 0; i < order.size(); ++i) {
      EXPECT_EQ(order[i], i);
    }
  }
}

TEST(ColumnOrderingTest, TwoOptNeverMakesItSlower)
{
  std::vector<ColumnEnds> cols;
  for (int i = 0; i < 8; ++i) {
    cols.push_back(col(0.07 * ((i * 3) % 8), (i % 3) * 0.3, ((i + 1) % 3) * 0.3));
  }
  const auto nn = orderColumnsByTime(cols, cols[0].start, false);
  const auto opt = orderColumnsByTime(cols, cols[0].start, true);
  EXPECT_LE(
    orderTravelTime(cols, opt, cols[0].start), orderTravelTime(cols, nn, cols[0].start) + 1e-9);
}

TEST(ColumnOrderingTest, EmptyAndSingle)
{
  EXPECT_TRUE(orderColumnsByTime({}, {0, 0, 0}).empty());
  EXPECT_EQ(orderColumnsByTime({col(0, 0, 1)}, {0, 0, 0}), (std::vector<size_t>{0}));
}

TEST(ColumnOrderingTest, PolicyNames)
{
  EXPECT_EQ(normalizeColumnOrder("nearest"), "nearest");
  EXPECT_EQ(normalizeColumnOrder("nearest_2opt"), "nearest_2opt");
  EXPECT_EQ(normalizeColumnOrder(""), "input");
  EXPECT_EQ(normalizeColumnOrder("bogus"), "input");
}

}  // namespace manipulation_manager
