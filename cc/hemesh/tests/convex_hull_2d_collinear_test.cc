#include "cc/hemesh/convex_hull_2d_collinear.h"

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

// | + |
TEST(ConvexHull2DCollinearTest, OnePoint) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 1 - 0 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 1
    BEGIN: vx1 : valence = 0
      he0
     
     
      point = {0, 0}
    END  : vx1
  END  : Vertices : size = 1
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 0
  END  : Half edges : size = 0
  BEGIN: Boundary half edges : size = 0
  END  : Boundary half edges : size = 0
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {{VXIndex(1), 0}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+ |
TEST(ConvexHull2DCollinearTest, TwoPoints) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 2 - 1 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 2
    BEGIN: vx1 : valence = 1
      he2
      vx2
      fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 1
      he3
      vx1
      fa0
      point = {1, 0}
    END  : vx2
  END  : Vertices : size = 2
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 2
    BEGIN: he2
      vx1 : he2
      fa0 : he3
    END  : he2
    BEGIN: he3
      vx2 : he3
      fa0 : he2
    END  : he3
  END  : Half edges : size = 2
  BEGIN: Boundary half edges : size = 2
    he2 he3
  END  : Boundary half edges : size = 2
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {{VXIndex(1), 0}, {VXIndex(2), 3}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+ |
TEST(ConvexHull2DCollinearTest, ThreeCollinearPoints) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 3 - 2 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 3
    BEGIN: vx1 : valence = 1
      he2
      vx2
      fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he4
      vx3 vx1
      fa0 fa0
      point = {1, 0}
    END  : vx2
    BEGIN: vx3 : valence = 1
      he5
      vx2
      fa0
      point = {2, 0}
    END  : vx3
  END  : Vertices : size = 3
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 4
    BEGIN: he2
      vx1 : he2
      fa0 : he4
    END  : he2
    BEGIN: he3
      vx2 : he4
      fa0 : he2
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa0 : he5
    END  : he4
    BEGIN: he5
      vx3 : he5
      fa0 : he3
    END  : he5
  END  : Half edges : size = 4
  BEGIN: Boundary half edges : size = 4
    he2 he3 he4 he5
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 4}, {VXIndex(3), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+-+ |
TEST(ConvexHull2DCollinearTest, FourCollinearPoints) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 4 - 3 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 4
    BEGIN: vx1 : valence = 1
      he2
      vx2
      fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he4
      vx3 vx1
      fa0 fa0
      point = {1, 0}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he6
      vx4 vx2
      fa0 fa0
      point = {2, 0}
    END  : vx3
    BEGIN: vx4 : valence = 1
      he7
      vx3
      fa0
      point = {3, 0}
    END  : vx4
  END  : Vertices : size = 4
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 6
    BEGIN: he2
      vx1 : he2
      fa0 : he4
    END  : he2
    BEGIN: he3
      vx2 : he4
      fa0 : he2
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa0 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he3
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa0 : he7
    END  : he6
    BEGIN: he7
      vx4 : he7
      fa0 : he5
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 6
    he2 he3 he4 he5 he6 he7
  END  : Boundary half edges : size = 6
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 5}, {VXIndex(3), 2}, {VXIndex(4), 3}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +   |
// | |\  |
// | +-+ |
TEST(ConvexHull2DCollinearTest, Triangle) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 1.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

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
    {VXIndex(1), 0}, {VXIndex(2), 5}, {VXIndex(3), 1}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+ |
// | |  /  |
// | | /   |
// | |/    |
// | +     |
TEST(ConvexHull2DCollinearTest, TriangleTopCollinear) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, 4.0}),
    math::VectorOwned<double, 2>({4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, 4.0}),
    math::VectorOwned<double, 2>({4.0, 4.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 4 - 4 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 4
    BEGIN: vx1 : valence = 2
      he9
      vx4 vx2
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
      vx2 vx4
      fa1 fa0
      point = {4, 4}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he4
      vx3 vx1
      fa1 fa0
      point = {8, 4}
    END  : vx4
  END  : Vertices : size = 4
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 4
      he3
      vx2 vx1 vx4 vx3
      fa0 fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 8
    BEGIN: he2
      vx1 : he9
      fa0 : he7
    END  : he2
    BEGIN: he3
      vx2 : he7
      fa1 : he9
    END  : he3
    BEGIN: he4
      vx4 : he8
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he8
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he5
    END  : he7
    BEGIN: he8
      vx4 : he4
      fa0 : he2
    END  : he8
    BEGIN: he9
      vx1 : he2
      fa1 : he4
    END  : he9
  END  : Half edges : size = 8
  BEGIN: Boundary half edges : size = 4
    he2 he5 he7 he8
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 1}, {VXIndex(2), 4}, {VXIndex(3), 3}, {VXIndex(4), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +     |
// | |\    |
// | | \   |
// | |  \  |
// | +-+-+ |
TEST(ConvexHull2DCollinearTest, TriangleBottomCollinear) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, -4.0}),
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({4.0, -4.0}),
    math::VectorOwned<double, 2>({8.0, -4.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

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
      point = {8, -4}
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
    {VXIndex(1), 1}, {VXIndex(2), 4}, {VXIndex(3), 2}, {VXIndex(4), 3}};
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
    math::VectorOwned<double, 2>({0.0, 1.0}),
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
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

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
    {VXIndex(1), 0}, {VXIndex(2), 11}, {VXIndex(3), 1}, {VXIndex(4), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// |  +        |
// |  | \      |
// |  |  \     |
// |  |   \    |
// |  | +--+   |
// |  | | +    |
// |  | | |+   |
// |  | | +    |
// |  | +--+   |
// |  |   /    |
// |  |  /     |
// |  | /      |
// |  +        |
TEST(ConvexHull2DCollinearTest, TriangleOnCShape) {
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({-6.0, -10.0}),
    math::VectorOwned<double, 2>({0.0, -4.0}),
    math::VectorOwned<double, 2>({-4.0, -4.0}),
    math::VectorOwned<double, 2>({-4.0, 4.0}),
    math::VectorOwned<double, 2>({0.0, 4.0}),
    math::VectorOwned<double, 2>({-6.0, 10.0}),
    math::VectorOwned<double, 2>({-2.0, -2.0}),
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({-2.0, 2.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d->CreateConvexHull(points, /* keep_collinear_points = */ true, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 5 - 5 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 5
    BEGIN: vx1 : valence = 2
      he4
      vx3 vx2
      fa1 fa0
      point = {-6, -10}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx5
      fa1 fa0
      point = {-6, 10}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he9
      vx4 vx1
      fa1 fa0
      point = {0, -4}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he11
      vx5 vx3
      fa1 fa0
      point = {0, 0}
    END  : vx4
    BEGIN: vx5 : valence = 2
      he6
      vx2 vx4
      fa1 fa0
      point = {0, 4}
    END  : vx5
  END  : Vertices : size = 5
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 5
      he3
      vx2 vx1 vx3 vx4 vx5
      fa0 fa0 fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 10
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
      vx5 : he10
      fa1 : he3
    END  : he6
    BEGIN: he7
      vx2 : he3
      fa0 : he10
    END  : he7
    BEGIN: he8
      vx4 : he11
      fa0 : he5
    END  : he8
    BEGIN: he9
      vx3 : he5
      fa1 : he11
    END  : he9
    BEGIN: he10
      vx5 : he6
      fa0 : he8
    END  : he10
    BEGIN: he11
      vx4 : he8
      fa1 : he6
    END  : he11
  END  : Half edges : size = 10
  BEGIN: Boundary half edges : size = 5
    he2 he5 he7 he8 he10
  END  : Boundary half edges : size = 5
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 5}, {VXIndex(3), 1}, {VXIndex(4), 7}, {VXIndex(5), 4}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}
  
}  // namespace
}  // namespace hemesh
