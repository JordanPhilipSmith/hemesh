#pragma once

#include "cc/hemesh/hemesh_geometry.h"

#include "cc/math/bounding_box.h"
#include "cc/math/vector.h"

namespace hemesh {

// Calculates the bounding box of a mesh.
// Parameters:
//   mesh - Mesh.
// Returns the bounding box.
template<class ScalarT, int dim>
math::BoundingBox<ScalarT, dim> CalculateBoundingBox(const MeshGeometry<ScalarT, dim>& mesh);

// Implementation.

template<class ScalarT, int dim>
math::BoundingBox<ScalarT, dim> CalculateBoundingBox(const MeshGeometry<ScalarT, dim>& mesh) {
  math::BoundingBox<ScalarT, dim> bounding_box;
  for (VXIndex vx : mesh.GetVXIndices()) {
    bounding_box.InsertPoint(mesh.VXGetPoint(vx));
  }
  return bounding_box;
}

}  // namespace hemesh
