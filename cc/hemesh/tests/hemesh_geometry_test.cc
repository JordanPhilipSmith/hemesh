#include "cc/hemesh/hemesh_geometry.h"

#include <vector>

#include "cc/math/vector.h"
#include "gtest/gtest.h"

namespace hemesh {
namespace {

TEST(MeshGeometryTest, Normal) {
  MeshGeometry<double, 2> mesh;

  // Create a single triangle.
  const std::vector<math::VectorOwned<double, 2>> points = {
    math::VectorOwned<double, 2>(0.0, 0.0),
    math::VectorOwned<double, 2>(1.0, 0.0),
    math::VectorOwned<double, 2>(0.0, 1.0)
  };

  const int size = static_cast<int>(points.size());
  const FAIndex fa = mesh.FAAllocate();
  mesh.FASetValence(fa, size);

  HEIndex he_begin = kHEInvalid;
  HEIndex he_prev = kHEInvalid;
  for (const math::ConstVector<double, 2>& point : points) {
    const VXIndex vx = mesh.VXAllocate();
    mesh.VXSetValence(vx, 2);
    mesh.VXMutablePoint(vx) = point;

    const HEIndex he = mesh.HEAllocate();
    mesh.HESetIsBoundary(mesh.HEGetHE(he), true);
    mesh.VXSetHE(vx, he);
    mesh.HESetVX(he, vx);
    mesh.HESetFA(he, fa);
    if (he_begin == kHEInvalid) {
      mesh.FASetHE(fa, he);
      he_begin = he;
    } else {
      mesh.HESetVX(mesh.HEGetHE(he_prev), vx);
      mesh.Splice(mesh.HEGetHE(he_prev), he); 
    }

    he_prev = he;
  }
  if (he_begin != kHEInvalid) {
      mesh.HESetVX(mesh.HEGetHE(he_prev), mesh.HEGetVX(he_begin));
      mesh.Splice(mesh.HEGetHE(he_prev), he_begin); 
  }

  EXPECT_EQ(mesh.Display(), R"debug(BEGIN: Mesh
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
}

}  // namespace
}  // namespace hemesh
