#pragma once

// Half edge mesh to represent 2-manifolds with positional geometry.

#include <string>

#include "absl/strings/str_cat.h"
#include "cc/hemesh/hemesh.h"
#include "cc/math/vector.h"

namespace hemesh {

// Mesh with geometric positions.
template<class ScalarT, int dim>
class MeshGeometry : public MeshConnectivity {
 public:
  // Constructor.
  MeshGeometry() : vertex_geometries_(1) {}

  virtual ~MeshGeometry() = default;
  
  // Vertex methods.
  
  const math::ConstVector<ScalarT, dim>& VXGetPoint(VXIndex vx) const {
    return this->VXGetGeometry(vx).point;
  }

  math::VectorOwned<ScalarT, dim>& VXMutablePoint(VXIndex vx) {
    return this->VXMutableGeometry(vx)->point;
  }

 protected:
  struct VertexGeometry {
    void Clear() {
      this->point.setZero();
    }

    math::VectorOwned<ScalarT, dim> point;
  };  // struct VertexGeometry

  // Vertex methods.

  const VertexGeometry& VXGetGeometry(VXIndex vx) const {
    CHECK(this->IsAllocatedVXIndex(vx));
    return this->vertex_geometries_[vx];
  }

  VertexGeometry* VXMutableGeometry(VXIndex vx) {
    CHECK(this->IsAllocatedVXIndex(vx));
    return &(this->vertex_geometries_[vx]);
  }

  VXIndex VXNew() override {
    const VXIndex vx = MeshConnectivity::VXNew();

    this->vertex_geometries_.resize(static_cast<std::size_t>(vx) + 1);
    return vx;
  }

  void VXInit(VXIndex vx) override {
    MeshConnectivity::VXInit(vx);

    this->vertex_geometries_[vx].Clear();
  }

  void VXClear(VXIndex vx) override {
    this->VXMutableGeometry(vx)->Clear();

    MeshConnectivity::VXClear(vx);
  }

  void VXDisplay(VXIndex vx, std::string *buffer) const override {
    MeshConnectivity::VXDisplay(vx, buffer);

    absl::StrAppend(buffer, "      point = ", math::VectorToString<ScalarT>(this->VXGetPoint(vx)),
		    "\n");
  }
  
  std::vector<VertexGeometry> vertex_geometries_;  
};  // class MeshGeometry

}  // namespace hemesh
