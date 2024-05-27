#pragma once

// Half edge mesh to represent 2-manifolds with positional geometry.

#include <string>
#include <unordered_map>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "cc/hemesh/hemesh.h"
#include "cc/math/vector.h"
#include "cc/status/status_macros.h"

namespace hemesh {

// Mesh with geometric positions.
template<class ScalarT, int dim>
class MeshGeometry : public MeshConnectivity {
 public:
  // Constructor.
  MeshGeometry() : vertex_geometries_(1) {}

  virtual ~MeshGeometry() = default;
  
  // Copy the source geometry mesh into the current destination mesh.
  // Parameters:
  //   mesh_src - Source geometry mesh that is copied.
  //   opt_vx_dst_for_src - If not nullptr, receives a map of {vx_src, vx_dst} pairs.
  //   opt_fa_dst_for_src - If not nullptr, receives a map of {fa_src, fa_dst} pairs.
  //   opt_he_dst_for_src - If not nullptr, receives a map of {he_src, he_dst} pairs.
  absl::Status CopyGeometry(
      const MeshGeometry<ScalarT, dim>& mesh_src,
      std::unordered_map<VXIndex, VXIndex>* opt_vx_dst_for_src,
      std::unordered_map<FAIndex, FAIndex>* opt_fa_dst_for_src,
      std::unordered_map<HEIndex, HEIndex>* opt_he_dst_for_src) {
    std::unordered_map<VXIndex, VXIndex>* vx_dst_for_src = opt_vx_dst_for_src;
    std::unordered_map<VXIndex, VXIndex> local_vx_dst_for_src;
    if (vx_dst_for_src == nullptr) {
      vx_dst_for_src = &local_vx_dst_for_src;
    }

    std::unordered_map<FAIndex, FAIndex>* fa_dst_for_src = opt_fa_dst_for_src;
    std::unordered_map<FAIndex, FAIndex> local_fa_dst_for_src;
    if (fa_dst_for_src == nullptr) {
      fa_dst_for_src = &local_fa_dst_for_src;
    }

    std::unordered_map<HEIndex, HEIndex>* he_dst_for_src = opt_he_dst_for_src;
    std::unordered_map<HEIndex, HEIndex> local_he_dst_for_src;
    if (he_dst_for_src == nullptr) {
      he_dst_for_src = &local_he_dst_for_src;
    }

    RETURN_IF_ERROR(
        this->CopyConnectivity(mesh_src, vx_dst_for_src, fa_dst_for_src, he_dst_for_src));

    // Copy vertex fields.
    for (const auto& [vx_src, vx_dst] : *vx_dst_for_src) {
      this->VXMutablePoint(vx_dst) = mesh_src.VXGetPoint(vx_src);
    }

    // Copy facet fields.

    // Copy half edge fields.

    return absl::OkStatus();
  }

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
