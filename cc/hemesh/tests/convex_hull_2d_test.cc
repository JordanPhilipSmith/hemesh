#include "cc/hemesh/convex_hull_2d.h"

#include <map>
#include <unordered_map>
#include <vector>

#include "cc/container/map_util.h"
#include "cc/geom/delaunay_predicates.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/math/vector2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace hemesh {
namespace {

// | +   |
// | |\  |
// | +-+ |
TEST(ConvexHull2DTest, Triangle) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 3 - 3 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 3
    BEGIN: vx1 : valence = 2
      he4
      vx3 vx2
      fa1 fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx3
      fa1 fa0
      point = {0, 1}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he6
      vx2 vx1
      fa1 fa0
      point = {1, 0}
    END  : vx3
  END  : Vertices : size = 3
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 3
      he3
      vx2 vx1 vx3
      fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 6
    BEGIN: he2
      vx1 : he4
      fa0 : he7
    END  : he2
    BEGIN: he3
      vx2 : he7
      fa1 : he4
    END  : he3
    BEGIN: he4
      vx1 : he2
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he2
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he5
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 3
    he2 he5 he7
  END  : Boundary half edges : size = 3
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 2}, {VXIndex(3), 1}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+ |
// | |  /  |
// | | /   |
// | |/    |
// | +     |
TEST(ConvexHull2DTest, TriangleTopCollinear) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, 4.0}),
    math::VectorOwned<double, 2>({4.0, 4.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 3 - 3 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 3
    BEGIN: vx1 : valence = 2
      he4
      vx3 vx2
      fa1 fa0
      point = {-4, -4}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx3
      fa1 fa0
      point = {-4, 4}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he6
      vx2 vx1
      fa1 fa0
      point = {8, 4}
    END  : vx3
  END  : Vertices : size = 3
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 3
      he3
      vx2 vx1 vx3
      fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 6
    BEGIN: he2
      vx1 : he4
      fa0 : he7
    END  : he2
    BEGIN: he3
      vx2 : he7
      fa1 : he4
    END  : he3
    BEGIN: he4
      vx1 : he2
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he2
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he5
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 3
    he2 he5 he7
  END  : Boundary half edges : size = 3
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 1}, {VXIndex(2), 0}, {VXIndex(3), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +     |
// | |\    |
// | | \   |
// | |  \  |
// | +-+-+ |
TEST(ConvexHull2DTest, TriangleBottomCollinear) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, -4.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 3 - 3 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 3
    BEGIN: vx1 : valence = 2
      he4
      vx3 vx2
      fa1 fa0
      point = {-4, -4}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx3
      fa1 fa0
      point = {-4, 4}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he6
      vx2 vx1
      fa1 fa0
      point = {8, -4}
    END  : vx3
  END  : Vertices : size = 3
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 3
      he3
      vx2 vx1 vx3
      fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 6
    BEGIN: he2
      vx1 : he4
      fa0 : he7
    END  : he2
    BEGIN: he3
      vx2 : he7
      fa1 : he4
    END  : he3
    BEGIN: he4
      vx1 : he2
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he2
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he5
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 3
    he2 he5 he7
  END  : Boundary half edges : size = 3
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 1}, {VXIndex(2), 0}, {VXIndex(3), 3}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-----+ |
// | |  +  | |
// | | + + | |
// | |  +  | |
// | +-----+ |
TEST(ConvexHull2DTest, SquareWithInternalDiamond) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({4.0, -4.0}),
    math::VectorOwned<double, 2>({4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, -1.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 4 - 4 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 4
    BEGIN: vx1 : valence = 2
      he4
      vx3 vx2
      fa1 fa0
      point = {-4, -4}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx4
      fa1 fa0
      point = {-4, 4}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he9
      vx4 vx1
      fa1 fa0
      point = {4, -4}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he6
      vx2 vx3
      fa1 fa0
      point = {4, 4}
    END  : vx4
  END  : Vertices : size = 4
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 4
      he3
      vx2 vx1 vx3 vx4
      fa0 fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 8
    BEGIN: he2
      vx1 : he4
      fa0 : he7
    END  : he2
    BEGIN: he3
      vx2 : he7
      fa1 : he4
    END  : he3
    BEGIN: he4
      vx1 : he2
      fa1 : he9
    END  : he4
    BEGIN: he5
      vx3 : he9
      fa0 : he2
    END  : he5
    BEGIN: he6
      vx4 : he8
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he8
    END  : he7
    BEGIN: he8
      vx4 : he6
      fa0 : he5
    END  : he8
    BEGIN: he9
      vx3 : he5
      fa1 : he6
    END  : he9
  END  : Half edges : size = 8
  BEGIN: Boundary half edges : size = 4
    he2 he5 he7 he8
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 3}, {VXIndex(3), 1}, {VXIndex(4), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

}  // namespace
}  // namespace hemesh
