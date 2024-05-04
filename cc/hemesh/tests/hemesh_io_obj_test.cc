#include "cc/hemesh/hemesh_io_obj.h"

#include <string>

#include "cc/hemesh/hemesh_geometry.h"
#include "cc/math/vector.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gtest/gtest.h"

namespace hemesh::io {
namespace {

TEST(ObjToMeshToObjTest, Triangle2D) {
  static constexpr char kExpectedObj[] = R"obj(v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
)obj";

  // Parse the .obj file.
  absl::StatusOr<std::unique_ptr<MeshGeometry<double, 2>>> mesh_or =
      FromObj<double, 2>(kExpectedObj);
  ASSERT_TRUE(mesh_or.ok());
  std::unique_ptr<MeshGeometry<double, 2>> mesh = *std::move(mesh_or);

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
  
  // Write the .obj file.
  absl::StatusOr<std::string> buffer_or = ToObj<double, 2>(*mesh);
  ASSERT_TRUE(buffer_or.ok());
  const std::string& buffer = *buffer_or;
  
  EXPECT_EQ(buffer, kExpectedObj);
}

TEST(ObjToMeshToObjTest, Cube3D) {
  static constexpr char kExpectedObj[] = R"obj(v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
f 4 3 2 1
f 1 2 6 5
f 2 3 7 6
f 3 4 8 7
f 4 1 5 8
f 5 6 7 8
)obj";

  // Parse the .obj file.
  absl::StatusOr<std::unique_ptr<MeshGeometry<double, 3>>> mesh_or =
      FromObj<double, 3>(kExpectedObj);
  ASSERT_TRUE(mesh_or.ok());
  std::unique_ptr<MeshGeometry<double, 3>> mesh = *std::move(mesh_or);

  EXPECT_EQ(mesh->Display(), R"debug(BEGIN: Mesh
  BEGIN: Euler characteristic
    2 = V - E + F = 8 - 12 + 6 = 2
  END  : Euler characteristic
  BEGIN: Vertices : size = 8
    BEGIN: vx1 : valence = 3
      he8
      vx4 vx2 vx5
      fa1 fa2 fa5
      point = {-1, -1, -1}
    END  : vx1
    BEGIN: vx2 : valence = 3
      he6
      vx1 vx3 vx6
      fa1 fa3 fa2
      point = {1, -1, -1}
    END  : vx2
    BEGIN: vx3 : valence = 3
      he4
      vx2 vx4 vx7
      fa1 fa4 fa3
      point = {1, 1, -1}
    END  : vx3
    BEGIN: vx4 : valence = 3
      he2
      vx3 vx1 vx8
      fa1 fa5 fa4
      point = {-1, 1, -1}
    END  : vx4
    BEGIN: vx5 : valence = 3
      he14
      vx1 vx6 vx8
      fa2 fa6 fa5
      point = {-1, -1, 1}
    END  : vx5
    BEGIN: vx6 : valence = 3
      he12
      vx5 vx2 vx7
      fa2 fa3 fa6
      point = {1, -1, 1}
    END  : vx6
    BEGIN: vx7 : valence = 3
      he18
      vx6 vx3 vx8
      fa3 fa4 fa6
      point = {1, 1, 1}
    END  : vx7
    BEGIN: vx8 : valence = 3
      he22
      vx7 vx4 vx5
      fa4 fa5 fa6
      point = {-1, 1, 1}
    END  : vx8
  END  : Vertices : size = 8
  BEGIN: Facets : size = 6
    BEGIN: fa1 : valence = 4
      he2
      vx4 vx3 vx2 vx1
      fa4 fa3 fa2 fa5
    END  : fa1
    BEGIN: fa2 : valence = 4
      he7
      vx1 vx2 vx6 vx5
      fa1 fa3 fa6 fa5
    END  : fa2
    BEGIN: fa3 : valence = 4
      he5
      vx2 vx3 vx7 vx6
      fa1 fa4 fa6 fa2
    END  : fa3
    BEGIN: fa4 : valence = 4
      he3
      vx3 vx4 vx8 vx7
      fa1 fa5 fa6 fa3
    END  : fa4
    BEGIN: fa5 : valence = 4
      he9
      vx4 vx1 vx5 vx8
      fa1 fa2 fa6 fa4
    END  : fa5
    BEGIN: fa6 : valence = 4
      he13
      vx5 vx6 vx7 vx8
      fa2 fa3 fa4 fa5
    END  : fa6
  END  : Facets : size = 6
  BEGIN: Half edges : size = 24
    BEGIN: he2
      vx4 : he9
      fa1 : he4
    END  : he2
    BEGIN: he3
      vx3 : he16
      fa4 : he20
    END  : he3
    BEGIN: he4
      vx3 : he3
      fa1 : he6
    END  : he4
    BEGIN: he5
      vx2 : he10
      fa3 : he16
    END  : he5
    BEGIN: he6
      vx2 : he5
      fa1 : he8
    END  : he6
    BEGIN: he7
      vx1 : he15
      fa2 : he10
    END  : he7
    BEGIN: he8
      vx1 : he7
      fa1 : he2
    END  : he8
    BEGIN: he9
      vx4 : he20
      fa5 : he15
    END  : he9
    BEGIN: he10
      vx2 : he6
      fa2 : he12
    END  : he10
    BEGIN: he11
      vx6 : he19
      fa3 : he5
    END  : he11
    BEGIN: he12
      vx6 : he11
      fa2 : he14
    END  : he12
    BEGIN: he13
      vx5 : he24
      fa6 : he19
    END  : he13
    BEGIN: he14
      vx5 : he13
      fa2 : he7
    END  : he14
    BEGIN: he15
      vx1 : he8
      fa5 : he24
    END  : he15
    BEGIN: he16
      vx3 : he4
      fa3 : he18
    END  : he16
    BEGIN: he17
      vx7 : he23
      fa4 : he3
    END  : he17
    BEGIN: he18
      vx7 : he17
      fa3 : he11
    END  : he18
    BEGIN: he19
      vx6 : he12
      fa6 : he23
    END  : he19
    BEGIN: he20
      vx4 : he2
      fa4 : he22
    END  : he20
    BEGIN: he21
      vx8 : he25
      fa5 : he9
    END  : he21
    BEGIN: he22
      vx8 : he21
      fa4 : he17
    END  : he22
    BEGIN: he23
      vx7 : he18
      fa6 : he25
    END  : he23
    BEGIN: he24
      vx5 : he14
      fa5 : he21
    END  : he24
    BEGIN: he25
      vx8 : he22
      fa6 : he13
    END  : he25
  END  : Half edges : size = 24
  BEGIN: Boundary half edges : size = 0
  END  : Boundary half edges : size = 0
END  : Mesh
)debug");

  // Write the .obj file.
  absl::StatusOr<std::string> buffer_or = ToObj<double, 3>(*mesh);
  ASSERT_TRUE(buffer_or.ok());
  const std::string& buffer = *buffer_or;
  
  EXPECT_EQ(buffer, kExpectedObj);
}

}  // namespace
}  // namespace hemesh::io
