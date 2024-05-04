#include "cc/hemesh/hemesh_io_svg.h"

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/hemesh/hemesh_io_obj.h"
#include "cc/math/bounding_box.h"
#include "gtest/gtest.h"

namespace hemesh::io {
namespace {

TEST(Mesh2DToSvgTest, Triangle) {
  static constexpr char kObjBuffer[] = R"obj(v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
)obj";

  absl::StatusOr<std::unique_ptr<MeshGeometry<double, 2>>> mesh_or = FromObj<double, 2>(kObjBuffer);
  ASSERT_TRUE(mesh_or.ok());
  std::unique_ptr<MeshGeometry<double, 2>> mesh = *std::move(mesh_or);

  absl::StatusOr<std::string> svg_buffer_or = Mesh2DToSvg<double>(*mesh, nullptr);
  ASSERT_TRUE(svg_buffer_or.ok());
  const std::string& svg_buffer = *svg_buffer_or;

  EXPECT_EQ(svg_buffer, R"svg(<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="1000" height="1000" viewBox="0 0 1000 1000" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink">
  <g transform="translate(500 500) scale(990 -990) translate(-0.5 -0.5)">
    <path d="
     M0 0 L1 0
     M1 0 L0 1
     M0 1 L0 0
     Z"
     style="fill:none;stroke:black;stroke-width:3;vector-effect:non-scaling-stroke"/>
  </g>
</svg>
)svg");
}

}  // namespace
}  // namespace hemesh::io
