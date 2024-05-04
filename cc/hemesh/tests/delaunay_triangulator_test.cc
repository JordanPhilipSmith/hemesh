#include "cc/hemesh/delaunay_triangulator.h"

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

TEST(DelaunayTriangulatorTest, Empty) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points;

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 0 - 0 + 0 = 0
  END  : Euler characteristic
  BEGIN: Vertices : size = 0
  END  : Vertices : size = 0
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 0
  END  : Half edges : size = 0
  BEGIN: Boundary half edges : size = 0
  END  : Boundary half edges : size = 0
END  : Mesh
)debug");

  EXPECT_TRUE(index_for_vx.empty());
}

// | + |
TEST(DelaunayTriangulatorTest, OnePoint) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

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
TEST(DelaunayTriangulatorTest, TwoPoints) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

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

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 1}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+ |
TEST(DelaunayTriangulatorTest, ThreeCollinearPoints) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

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
    {VXIndex(1), 0}, {VXIndex(2), 1}, {VXIndex(3), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +   |
// | |\  |
// | +-+ |
TEST(DelaunayTriangulatorTest, Triangle) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({0.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 3 - 3 + 1 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 3
    BEGIN: vx1 : valence = 2
      he2
      vx2 vx3
      fa1 fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he4
      vx3 vx1
      fa1 fa0
      point = {1, 0}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he6
      vx1 vx2
      fa1 fa0
      point = {0, 1}
    END  : vx3
  END  : Vertices : size = 3
  BEGIN: Facets : size = 1
    BEGIN: fa1 : valence = 3
      he2
      vx1 vx2 vx3
      fa0 fa0 fa0
    END  : fa1
  END  : Facets : size = 1
  BEGIN: Half edges : size = 6
    BEGIN: he2
      vx1 : he7
      fa1 : he4
    END  : he2
    BEGIN: he3
      vx2 : he4
      fa0 : he7
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he6
      fa0 : he3
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he2
    END  : he6
    BEGIN: he7
      vx1 : he2
      fa0 : he5
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 3
    he3 he5 he7
  END  : Boundary half edges : size = 3
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 1}, {VXIndex(3), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+-+ |
TEST(DelaunayTriangulatorTest, FourCollinearPoints) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

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
      he3
      vx1 vx3
      fa0 fa0
      point = {1, 0}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he4
      vx4 vx2
      fa0 fa0
      point = {2, 0}
    END  : vx3
    BEGIN: vx4 : valence = 1
      he5
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
      fa0 : he6
    END  : he2
    BEGIN: he3
      vx2 : he6
      fa0 : he2
    END  : he3
    BEGIN: he4
      vx3 : he7
      fa0 : he5
    END  : he4
    BEGIN: he5
      vx4 : he5
      fa0 : he7
    END  : he5
    BEGIN: he6
      vx2 : he3
      fa0 : he4
    END  : he6
    BEGIN: he7
      vx3 : he4
      fa0 : he3
    END  : he7
  END  : Half edges : size = 6
  BEGIN: Boundary half edges : size = 6
    he2 he3 he4 he5 he6 he7
  END  : Boundary half edges : size = 6
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 1}, {VXIndex(3), 2}, {VXIndex(4), 3}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +   |
// | |\  |
// | | + |
// | |/| |
// | +-+ |
TEST(DelaunayTriangulatorTest, Trapezoid) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 1.0}),
    math::VectorOwned<double, 2>({0.0, 2.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 4 - 5 + 2 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 4
    BEGIN: vx1 : valence = 3
      he2
      vx2 vx3 vx4
      fa0 fa1 fa2
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 2
      he3
      vx1 vx4
      fa2 fa0
      point = {0, 2}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he4
      vx4 vx1
      fa1 fa0
      point = {1, 0}
    END  : vx3
    BEGIN: vx4 : valence = 3
      he5
      vx3 vx2 vx1
      fa0 fa2 fa1
      point = {1, 1}
    END  : vx4
  END  : Vertices : size = 4
  BEGIN: Facets : size = 2
    BEGIN: fa1 : valence = 3
      he6
      vx1 vx3 vx4
      fa0 fa0 fa2
    END  : fa1
    BEGIN: fa2 : valence = 3
      he9
      vx1 vx4 vx2
      fa1 fa0 fa0
    END  : fa2
  END  : Facets : size = 2
  BEGIN: Half edges : size = 10
    BEGIN: he2
      vx1 : he6
      fa0 : he11
    END  : he2
    BEGIN: he3
      vx2 : he11
      fa2 : he9
    END  : he3
    BEGIN: he4
      vx3 : he7
      fa1 : he8
    END  : he4
    BEGIN: he5
      vx4 : he10
      fa0 : he7
    END  : he5
    BEGIN: he6
      vx1 : he9
      fa1 : he4
    END  : he6
    BEGIN: he7
      vx3 : he4
      fa0 : he2
    END  : he7
    BEGIN: he8
      vx4 : he5
      fa1 : he6
    END  : he8
    BEGIN: he9
      vx1 : he2
      fa2 : he10
    END  : he9
    BEGIN: he10
      vx4 : he8
      fa2 : he3
    END  : he10
    BEGIN: he11
      vx2 : he3
      fa0 : he5
    END  : he11
  END  : Half edges : size = 10
  BEGIN: Boundary half edges : size = 4
    he2 he5 he7 he11
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 3}, {VXIndex(3), 1}, {VXIndex(4), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+-+-+ |
TEST(DelaunayTriangulatorTest, FiveCollinearPoints) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0}),
    math::VectorOwned<double, 2>({4.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 5 - 4 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 5
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
      he5
      vx2 vx4
      fa0 fa0
      point = {2, 0}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he6
      vx5 vx3
      fa0 fa0
      point = {3, 0}
    END  : vx4
    BEGIN: vx5 : valence = 1
      he7
      vx4
      fa0
      point = {4, 0}
    END  : vx5
  END  : Vertices : size = 5
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 8
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
      fa0 : he8
    END  : he4
    BEGIN: he5
      vx3 : he8
      fa0 : he3
    END  : he5
    BEGIN: he6
      vx4 : he9
      fa0 : he7
    END  : he6
    BEGIN: he7
      vx5 : he7
      fa0 : he9
    END  : he7
    BEGIN: he8
      vx3 : he5
      fa0 : he6
    END  : he8
    BEGIN: he9
      vx4 : he6
      fa0 : he5
    END  : he9
  END  : Half edges : size = 8
  BEGIN: Boundary half edges : size = 8
    he2 he3 he4 he5 he6 he7 he8 he9
  END  : Boundary half edges : size = 8
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 1}, {VXIndex(3), 2}, {VXIndex(4), 3}, {VXIndex(5), 4}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +-+-+-+-+-+ |
TEST(DelaunayTriangulatorTest, SixCollinearPoints) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0}),
    math::VectorOwned<double, 2>({4.0, 0.0}),
    math::VectorOwned<double, 2>({5.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 6 - 5 + 0 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 6
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
      he5
      vx2 vx4
      fa0 fa0
      point = {2, 0}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he6
      vx5 vx3
      fa0 fa0
      point = {3, 0}
    END  : vx4
    BEGIN: vx5 : valence = 2
      he8
      vx6 vx4
      fa0 fa0
      point = {4, 0}
    END  : vx5
    BEGIN: vx6 : valence = 1
      he9
      vx5
      fa0
      point = {5, 0}
    END  : vx6
  END  : Vertices : size = 6
  BEGIN: Facets : size = 0
  END  : Facets : size = 0
  BEGIN: Half edges : size = 10
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
      fa0 : he10
    END  : he4
    BEGIN: he5
      vx3 : he10
      fa0 : he3
    END  : he5
    BEGIN: he6
      vx4 : he11
      fa0 : he8
    END  : he6
    BEGIN: he7
      vx5 : he8
      fa0 : he11
    END  : he7
    BEGIN: he8
      vx5 : he7
      fa0 : he9
    END  : he8
    BEGIN: he9
      vx6 : he9
      fa0 : he7
    END  : he9
    BEGIN: he10
      vx3 : he5
      fa0 : he6
    END  : he10
    BEGIN: he11
      vx4 : he6
      fa0 : he5
    END  : he11
  END  : Half edges : size = 10
  BEGIN: Boundary half edges : size = 10
    he2 he3 he4 he5 he6 he7 he8 he9 he10 he11
  END  : Boundary half edges : size = 10
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 1}, {VXIndex(3), 2}, {VXIndex(4), 3}, {VXIndex(5), 4},
    {VXIndex(6), 5}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +---+ |
// | |\ /| |
// | | + | |
// | || || |
// | |/ \| |
// | +---+ |
TEST(DelaunayTriangulatorTest, SquareWithOffCenterPoint) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 2.0}),
    math::VectorOwned<double, 2>({0.0, 2.0}),
    math::VectorOwned<double, 2>({1.0, 1.5})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 5 - 8 + 4 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 5
    BEGIN: vx1 : valence = 3
      he2
      vx2 vx3 vx4
      fa1 fa0 fa2
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 4
      he4
      vx3 vx1 vx4 vx5
      fa1 fa2 fa3 fa4
      point = {1, 1.5}
    END  : vx2
    BEGIN: vx3 : valence = 3
      he6
      vx1 vx2 vx5
      fa1 fa4 fa0
      point = {0, 2}
    END  : vx3
    BEGIN: vx4 : valence = 3
      he8
      vx5 vx2 vx1
      fa3 fa2 fa0
      point = {2, 0}
    END  : vx4
    BEGIN: vx5 : valence = 3
      he9
      vx4 vx3 vx2
      fa0 fa4 fa3
      point = {2, 2}
    END  : vx5
  END  : Vertices : size = 5
  BEGIN: Facets : size = 4
    BEGIN: fa1 : valence = 3
      he2
      vx1 vx2 vx3
      fa2 fa4 fa0
    END  : fa1
    BEGIN: fa2 : valence = 3
      he10
      vx1 vx4 vx2
      fa0 fa3 fa1
    END  : fa2
    BEGIN: fa3 : valence = 3
      he13
      vx2 vx4 vx5
      fa2 fa0 fa4
    END  : fa3
    BEGIN: fa4 : valence = 3
      he15
      vx2 vx5 vx3
      fa3 fa0 fa1
    END  : fa4
  END  : Facets : size = 4
  BEGIN: Half edges : size = 16
    BEGIN: he2
      vx1 : he7
      fa1 : he4
    END  : he2
    BEGIN: he3
      vx2 : he13
      fa2 : he10
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he17
      fa4 : he15
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he2
    END  : he6
    BEGIN: he7
      vx1 : he10
      fa0 : he17
    END  : he7
    BEGIN: he8
      vx4 : he12
      fa3 : he14
    END  : he8
    BEGIN: he9
      vx5 : he16
      fa0 : he11
    END  : he9
    BEGIN: he10
      vx1 : he2
      fa2 : he12
    END  : he10
    BEGIN: he11
      vx4 : he8
      fa0 : he7
    END  : he11
    BEGIN: he12
      vx4 : he11
      fa2 : he3
    END  : he12
    BEGIN: he13
      vx2 : he15
      fa3 : he8
    END  : he13
    BEGIN: he14
      vx5 : he9
      fa3 : he13
    END  : he14
    BEGIN: he15
      vx2 : he4
      fa4 : he16
    END  : he15
    BEGIN: he16
      vx5 : he14
      fa4 : he5
    END  : he16
    BEGIN: he17
      vx3 : he6
      fa0 : he9
    END  : he17
  END  : Half edges : size = 16
  BEGIN: Boundary half edges : size = 4
    he7 he9 he11 he17
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 4}, {VXIndex(3), 3}, {VXIndex(4), 1}, {VXIndex(5), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +---+ |
// | |\ /| |
// | | + | |
// | |/ \| |
// | +---+ |
TEST(DelaunayTriangulatorTest, SquareWithCenterPoint) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 2.0}),
    math::VectorOwned<double, 2>({0.0, 2.0}),
    math::VectorOwned<double, 2>({1.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 5 - 8 + 4 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 5
    BEGIN: vx1 : valence = 3
      he2
      vx2 vx3 vx4
      fa1 fa0 fa2
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 4
      he4
      vx3 vx1 vx4 vx5
      fa1 fa2 fa3 fa4
      point = {1, 1}
    END  : vx2
    BEGIN: vx3 : valence = 3
      he6
      vx1 vx2 vx5
      fa1 fa4 fa0
      point = {0, 2}
    END  : vx3
    BEGIN: vx4 : valence = 3
      he8
      vx5 vx2 vx1
      fa3 fa2 fa0
      point = {2, 0}
    END  : vx4
    BEGIN: vx5 : valence = 3
      he9
      vx4 vx3 vx2
      fa0 fa4 fa3
      point = {2, 2}
    END  : vx5
  END  : Vertices : size = 5
  BEGIN: Facets : size = 4
    BEGIN: fa1 : valence = 3
      he2
      vx1 vx2 vx3
      fa2 fa4 fa0
    END  : fa1
    BEGIN: fa2 : valence = 3
      he10
      vx1 vx4 vx2
      fa0 fa3 fa1
    END  : fa2
    BEGIN: fa3 : valence = 3
      he13
      vx2 vx4 vx5
      fa2 fa0 fa4
    END  : fa3
    BEGIN: fa4 : valence = 3
      he15
      vx2 vx5 vx3
      fa3 fa0 fa1
    END  : fa4
  END  : Facets : size = 4
  BEGIN: Half edges : size = 16
    BEGIN: he2
      vx1 : he7
      fa1 : he4
    END  : he2
    BEGIN: he3
      vx2 : he13
      fa2 : he10
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he17
      fa4 : he15
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he2
    END  : he6
    BEGIN: he7
      vx1 : he10
      fa0 : he17
    END  : he7
    BEGIN: he8
      vx4 : he12
      fa3 : he14
    END  : he8
    BEGIN: he9
      vx5 : he16
      fa0 : he11
    END  : he9
    BEGIN: he10
      vx1 : he2
      fa2 : he12
    END  : he10
    BEGIN: he11
      vx4 : he8
      fa0 : he7
    END  : he11
    BEGIN: he12
      vx4 : he11
      fa2 : he3
    END  : he12
    BEGIN: he13
      vx2 : he15
      fa3 : he8
    END  : he13
    BEGIN: he14
      vx5 : he9
      fa3 : he13
    END  : he14
    BEGIN: he15
      vx2 : he4
      fa4 : he16
    END  : he15
    BEGIN: he16
      vx5 : he14
      fa4 : he5
    END  : he16
    BEGIN: he17
      vx3 : he6
      fa0 : he9
    END  : he17
  END  : Half edges : size = 16
  BEGIN: Boundary half edges : size = 4
    he7 he9 he11 he17
  END  : Boundary half edges : size = 4
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 4}, {VXIndex(3), 3}, {VXIndex(4), 1}, {VXIndex(5), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// |     + |
// |    /| |
// |   / | |
// |  /  | |
// | +---+ |
// | |\ /  |
// | | +   |
// | |/    |
// | +     |
TEST(DelaunayTriangulatorTest, RhombusWithPointOnEdge) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 2.0}),
    math::VectorOwned<double, 2>({2.0, 4.0}),
    math::VectorOwned<double, 2>({0.0, 2.0}),
    math::VectorOwned<double, 2>({1.0, 1.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 5 - 7 + 3 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 5
    BEGIN: vx1 : valence = 2
      he2
      vx2 vx3
      fa1 fa0
      point = {0, 0}
    END  : vx1
    BEGIN: vx2 : valence = 3
      he4
      vx3 vx1 vx4
      fa1 fa0 fa2
      point = {1, 1}
    END  : vx2
    BEGIN: vx3 : valence = 4
      he6
      vx1 vx2 vx4 vx5
      fa1 fa2 fa3 fa0
      point = {0, 2}
    END  : vx3
    BEGIN: vx4 : valence = 3
      he8
      vx5 vx3 vx2
      fa3 fa2 fa0
      point = {2, 2}
    END  : vx4
    BEGIN: vx5 : valence = 2
      he9
      vx4 vx3
      fa0 fa3
      point = {2, 4}
    END  : vx5
  END  : Vertices : size = 5
  BEGIN: Facets : size = 3
    BEGIN: fa1 : valence = 3
      he2
      vx1 vx2 vx3
      fa0 fa2 fa0
    END  : fa1
    BEGIN: fa2 : valence = 3
      he10
      vx2 vx4 vx3
      fa0 fa3 fa1
    END  : fa2
    BEGIN: fa3 : valence = 3
      he13
      vx3 vx4 vx5
      fa2 fa0 fa0
    END  : fa3
  END  : Facets : size = 3
  BEGIN: Half edges : size = 14
    BEGIN: he2
      vx1 : he7
      fa1 : he4
    END  : he2
    BEGIN: he3
      vx2 : he10
      fa0 : he7
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx3 : he13
      fa2 : he10
    END  : he5
    BEGIN: he6
      vx3 : he5
      fa1 : he2
    END  : he6
    BEGIN: he7
      vx1 : he2
      fa0 : he15
    END  : he7
    BEGIN: he8
      vx4 : he12
      fa3 : he14
    END  : he8
    BEGIN: he9
      vx5 : he14
      fa0 : he11
    END  : he9
    BEGIN: he10
      vx2 : he4
      fa2 : he12
    END  : he10
    BEGIN: he11
      vx4 : he8
      fa0 : he3
    END  : he11
    BEGIN: he12
      vx4 : he11
      fa2 : he5
    END  : he12
    BEGIN: he13
      vx3 : he15
      fa3 : he8
    END  : he13
    BEGIN: he14
      vx5 : he9
      fa3 : he13
    END  : he14
    BEGIN: he15
      vx3 : he6
      fa0 : he9
    END  : he15
  END  : Half edges : size = 14
  BEGIN: Boundary half edges : size = 5
    he3 he7 he9 he11 he15
  END  : Boundary half edges : size = 5
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 0}, {VXIndex(2), 4}, {VXIndex(3), 3}, {VXIndex(4), 1}, {VXIndex(5), 2}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

// | +       |
// | |\      |
// | + \     |
// | |\ \    |
// | + \ \   |
// |  \ \ \  |
// |   +-+-+ |
TEST(DelaunayTriangulatorTest, CollinearCorner) {
  std::unique_ptr<DelaunayTriangulator<double>> delaunay_triangulator =
      DelaunayTriangulator<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>({0.0, 3.0}),
    math::VectorOwned<double, 2>({0.0, 2.0}),
    math::VectorOwned<double, 2>({0.0, 1.0}),
    math::VectorOwned<double, 2>({1.0, 0.0}),
    math::VectorOwned<double, 2>({2.0, 0.0}),
    math::VectorOwned<double, 2>({3.0, 0.0})
  };

  std::unordered_map<VXIndex, int> index_for_vx;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      delaunay_triangulator->Triangulate(points, &index_for_vx);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 6 - 9 + 4 = 1
  END  : Euler characteristic
  BEGIN: Vertices : size = 6
    BEGIN: vx1 : valence = 3
      he2
      vx2 vx4 vx5
      fa0 fa1 fa2
      point = {0, 1}
    END  : vx1
    BEGIN: vx2 : valence = 4
      he4
      vx3 vx1 vx5 vx6
      fa0 fa2 fa3 fa4
      point = {0, 2}
    END  : vx2
    BEGIN: vx3 : valence = 2
      he5
      vx2 vx6
      fa4 fa0
      point = {0, 3}
    END  : vx3
    BEGIN: vx4 : valence = 2
      he6
      vx5 vx1
      fa1 fa0
      point = {1, 0}
    END  : vx4
    BEGIN: vx5 : valence = 4
      he8
      vx6 vx2 vx1 vx4
      fa3 fa2 fa1 fa0
      point = {2, 0}
    END  : vx5
    BEGIN: vx6 : valence = 3
      he9
      vx5 vx3 vx2
      fa0 fa4 fa3
      point = {3, 0}
    END  : vx6
  END  : Vertices : size = 6
  BEGIN: Facets : size = 4
    BEGIN: fa1 : valence = 3
      he10
      vx1 vx4 vx5
      fa0 fa0 fa2
    END  : fa1
    BEGIN: fa2 : valence = 3
      he13
      vx1 vx5 vx2
      fa1 fa3 fa0
    END  : fa2
    BEGIN: fa3 : valence = 3
      he15
      vx2 vx5 vx6
      fa2 fa0 fa4
    END  : fa3
    BEGIN: fa4 : valence = 3
      he17
      vx2 vx6 vx3
      fa3 fa0 fa0
    END  : fa4
  END  : Facets : size = 4
  BEGIN: Half edges : size = 18
    BEGIN: he2
      vx1 : he10
      fa0 : he4
    END  : he2
    BEGIN: he3
      vx2 : he15
      fa2 : he13
    END  : he3
    BEGIN: he4
      vx2 : he3
      fa0 : he19
    END  : he4
    BEGIN: he5
      vx3 : he19
      fa4 : he17
    END  : he5
    BEGIN: he6
      vx4 : he11
      fa1 : he12
    END  : he6
    BEGIN: he7
      vx5 : he8
      fa0 : he11
    END  : he7
    BEGIN: he8
      vx5 : he14
      fa3 : he16
    END  : he8
    BEGIN: he9
      vx6 : he18
      fa0 : he7
    END  : he9
    BEGIN: he10
      vx1 : he13
      fa1 : he6
    END  : he10
    BEGIN: he11
      vx4 : he6
      fa0 : he2
    END  : he11
    BEGIN: he12
      vx5 : he7
      fa1 : he10
    END  : he12
    BEGIN: he13
      vx1 : he2
      fa2 : he14
    END  : he13
    BEGIN: he14
      vx5 : he12
      fa2 : he3
    END  : he14
    BEGIN: he15
      vx2 : he17
      fa3 : he8
    END  : he15
    BEGIN: he16
      vx6 : he9
      fa3 : he15
    END  : he16
    BEGIN: he17
      vx2 : he4
      fa4 : he18
    END  : he17
    BEGIN: he18
      vx6 : he16
      fa4 : he5
    END  : he18
    BEGIN: he19
      vx3 : he5
      fa0 : he9
    END  : he19
  END  : Half edges : size = 18
  BEGIN: Boundary half edges : size = 6
    he2 he4 he7 he9 he11 he19
  END  : Boundary half edges : size = 6
END  : Mesh
)debug");

  const std::map<VXIndex, int> expected_index_for_vx = {
    {VXIndex(1), 2}, {VXIndex(2), 1}, {VXIndex(3), 0}, {VXIndex(4), 3}, {VXIndex(5), 4},
    {VXIndex(6), 5}};
  EXPECT_EQ((container::MapFromUnorderedMap<VXIndex, int>(index_for_vx)), expected_index_for_vx);
}

}  // namespace
}  // namespace hemesh
