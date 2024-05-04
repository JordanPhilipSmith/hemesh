#pragma once

// 2D mesh I/O to .svg format.

#include <string>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/hemesh/hemesh_geometry_util.h"
#include "cc/math/bounding_box.h"
#include "cc/math/vector.h"

namespace hemesh::io {

// Renders a 2D mesh in .svg format.
// Parameters:
//   mesh - 2D mesh.
//   opt_bounding_box - Optional 2D bounding box.  If not nullptr, used as the viewport.
// Returns the .svg buffer or kFailedPrecondition on error.
template<class ScalarT>
absl::StatusOr<std::string> Mesh2DToSvg(
    const MeshGeometry<ScalarT, 2>& mesh, const math::BoundingBox<ScalarT, 2>* opt_bounding_box);

// Implementation.

namespace internal {

template<class ScalarT>
class SvgMesh2DWriter {
 public:
  // Constructor.
  SvgMesh2DWriter() = default;
  ~SvgMesh2DWriter() = default;

  // Renders a 2D mesh in .svg format.
  // Parameters:
  //   mesh - 2D mesh.
  //   opt_bounding_box - Optional 2D bounding box.  If not nullptr, used as the viewport.
  // Returns the .svg buffer or kFailedPrecondition on error.
  absl::StatusOr<std::string> Write(
      const MeshGeometry<ScalarT, 2>& mesh,
      const math::BoundingBox<ScalarT, 2>* opt_bounding_box) const {
    static constexpr ScalarT kZero(0);
    static constexpr ScalarT kOne(1);
    static constexpr ScalarT kTwo(2);

    static constexpr int kViewportSizePixels = 1000;
    static constexpr int kViewportBorderPixels = 5;
    static constexpr char kEdgeColor[] = "black";
    static constexpr int kEdgeWidth = 3;

    // Viewport.
    math::BoundingBox<int, 2> viewport;
    viewport.InsertPoint(math::VectorOwned<int, 2>({0, 0}));
    viewport.InsertPoint(math::VectorOwned<int, 2>({kViewportSizePixels, kViewportSizePixels}));
    
    // Calculate the bounding box for the viewport.
    const math::BoundingBox<ScalarT, 2>* bounding_box = opt_bounding_box;
    math::BoundingBox<ScalarT, 2> local_bounding_box;
    if (bounding_box == nullptr) {
      local_bounding_box = CalculateBoundingBox<ScalarT, 2>(mesh);
      bounding_box = &local_bounding_box;
    }
    CHECK_NE(bounding_box, nullptr);

    // Find the center of the bounding box and the uniform scale.
    ScalarT uniform_scale = kOne;
    math::VectorOwned<ScalarT, 2> center;
    center.setZero();
    if (!bounding_box->IsEmpty()) {
      math::VectorOwned<ScalarT, 2> diagonal = bounding_box->Max();
      diagonal -= bounding_box->Min();

      center = diagonal;
      center /= kTwo;
      center += bounding_box->Min();

      // Calculate the uniform scale.
      const ScalarT max_size = std::max<ScalarT>(diagonal[0], diagonal[1]);
      if (kZero < max_size) {
        uniform_scale =
            static_cast<ScalarT>(kViewportSizePixels - 2 * kViewportBorderPixels) / max_size;
      }
    }

    // Write the .svg buffer.
    std::string buffer;
    absl::StrAppend(&buffer, R"svg(<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width=")svg",
                    kViewportSizePixels, R"svg(" height=")svg", kViewportSizePixels,
                    R"svg(" viewBox=")svg",
                    viewport.Min()[0], " ", viewport.Min()[1], " ", viewport.Max()[0], " ",
                    viewport.Max()[1], R"svg(" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink">
  <g transform="translate()svg",
                    (kViewportSizePixels / 2), " ", (kViewportSizePixels / 2), ") scale(",
                    uniform_scale, " ", (-uniform_scale), ") translate(", (-center[0]), " ", (-center[1]), R"svg()">
    <path d=")svg");

      for (EIndex e : mesh.GetSortedEIndices()) {
        const HEIndex he = mesh.EGetHE(e);
        const math::ConstVector<ScalarT, 2>& v0 = mesh.VXGetPoint(mesh.HEGetVX(he));
        const math::ConstVector<ScalarT, 2>& v1 = mesh.VXGetPoint(mesh.HEGetVX(mesh.HEGetHE(he)));
        absl::StrAppend(&buffer, "\n     M", v0[0], " ", v0[1], " L", v1[0], " ", v1[1]);        
      }
    
    absl::StrAppend(&buffer, R"svg(
     Z"
     style="fill:none;stroke:)svg",
                    kEdgeColor, ";stroke-width:", kEdgeWidth,
                    R"svg(;vector-effect:non-scaling-stroke"/>
  </g>
</svg>
)svg");

    return buffer;
  }
};  // class SvgMesh2DWriter

}  // namespace internal
 
template<class ScalarT>
absl::StatusOr<std::string> Mesh2DToSvg(
    const MeshGeometry<ScalarT, 2>& mesh, const math::BoundingBox<ScalarT, 2>* opt_bounding_box) {
  internal::SvgMesh2DWriter<ScalarT> writer;
  return writer.Write(mesh, opt_bounding_box);
}

}  // namespace hemesh::io
