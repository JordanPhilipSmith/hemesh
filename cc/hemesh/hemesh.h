#pragma once

// Half edge mesh to represent 2-manifolds.  This is a modification of the quad edge data structure.
//     _
//   _| |_
//  |_ \ _|
//    |_|
//
// The 2-manifold surface mesh is made up of vertices, facets, and half edges.
//
// Vertex:
//   The vertex contains the position.  The vertex points to one half edge in its CCW
//   (counterclockwise) half edge ring.  The valence is the number of half edges (i.e. vertex
//   neighbors).
//
// Facet:
//   A facet is a simple face loop.  The facet points to one half edge in its CCW half edge ring.
//   The valence is the number of half edges (i.e. vertices).
//
// Edge and Half Edge:
//   Every edge is represented by two opposite direction half edge pairs.  The half edge from
//   V1->V2 has an opposite partner pair from V2->V1.  An edge and its two half edges are allocated
//   and deallocated as a unit using HEAllocate() (one of the two half edges is returned).
//
//   Each half edge points to a vertex, a facet, its opposite partner half edge, its edge, the next
//   half edge around the vertex, and the next half edge around the facet.
//
//   Half edges on the boundary of the mesh point to the invalid facet (kFAInvalid).  There may be
//   multiple connected components, so it is not possible to represent the boundary area with a
//   single simple facet.  The boundary half edges are members of a set, hence the boundary can be
//   iterated over.
//
// Example: Square
//
// |             F0             |
// |                            |
// |           H7_              |
// |           _| |_            |
// |       V4-|_ / _|--V3       |
// |       |    |_| H6 |        |
// |       _ H8        _        |
// |     _| |_       _| |_H5    |
// | F0 |_ \ _|  F1 |_ \ _|  F0 |
// |      |_|         |_|       |
// |     H9|     _   H4|        |
// |       | H2_| |_   |        |
// |       V4-|_ / _|--V3       |
// |            |_| H3          |
// |                            |
// |             F0             |
 
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

namespace hemesh {

// Vertex index.
using VXIndex = int;
// Facet index.
using FAIndex = int;
// Edge index.
using EIndex = int;
// Half edge index.
using HEIndex = int;

constexpr VXIndex kVXInvalid = VXIndex(0);
constexpr FAIndex kFAInvalid = FAIndex(0);
constexpr EIndex kEInvalid = EIndex(0);
constexpr HEIndex kHEInvalid = HEIndex(0);
  
template<class IndexT>
class IndicesIterableBase {
 public:
  class const_iterator {
   public:
    const_iterator(typename std::unordered_set<IndexT>::const_iterator iter) : iter_(iter) {}
    const_iterator(const const_iterator& iter) : const_iterator(iter.iter_) {}

    IndexT operator*() const {
      return *(this->iter_);
    }

    bool operator==(const const_iterator& iter) const {
      return this->iter_ == iter.iter_;
    }

    bool operator!=(const const_iterator& iter) const {
      return !(*this == iter);
    }

    const_iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    typename std::unordered_set<IndexT>::const_iterator iter_;
  };  // class const_iterator

  class iterator {
   public:
    iterator(typename std::unordered_set<IndexT>::iterator iter) : iter_(iter) {}
    iterator(const iterator& iter) : iterator(iter.iter_) {}

    IndexT operator*() const {
      return *(this->iter_);
    }

    bool operator==(const iterator& iter) const {
      return this->iter_ == iter.iter_;
    }

    bool operator!=(const iterator& iter) const {
      return !(*this == iter);
    }

    iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    typename std::unordered_set<IndexT>::iterator iter_;
  };  // class iterator

  IndicesIterableBase(const std::unordered_set<IndexT>* indices) : indices_(indices) {
    CHECK_NE(this->indices_, nullptr);
  }

  const_iterator cbegin() {
    return const_iterator(this->indices_->cbegin());
  }

  const_iterator cend() {
    return const_iterator(this->indices_->cend());
  }

  iterator begin() {
    return iterator(this->indices_->begin());
  }

  iterator end() {
    return iterator(this->indices_->end());
  }

 private:
  const std::unordered_set<IndexT>* indices_;
};  // class IndicesIterableBase

using VertexIterable = IndicesIterableBase<VXIndex>;
using FacetIterable = IndicesIterableBase<FAIndex>; 
using EdgeIterable = IndicesIterableBase<EIndex>; 
using BoundaryHalfEdgeIterable = IndicesIterableBase<HEIndex>; 

class HalfEdgeIterable {
 public:
  class const_iterator {
   public:
    const_iterator(typename std::unordered_set<EIndex>::const_iterator e_iter)
        : e_iter_(e_iter), bit_(0) {}
    const_iterator(const const_iterator& iter) : e_iter_(iter.e_iter_), bit_(iter.bit_) {}

    HEIndex operator*() const {
      return static_cast<HEIndex>((static_cast<int>(*(this->e_iter_)) << 1) + this->bit_);
    }

    bool operator==(const const_iterator& iter) const {
      return this->e_iter_ == iter.e_iter_ && this->bit_ == iter.bit_;
    }

    bool operator!=(const const_iterator& iter) const {
      return !(*this == iter);
    }

    const_iterator& operator++() {
      if (this->bit_ == 0) {
	this->bit_ = 1;
      } else {
        ++(this->e_iter_);
        this->bit_ = 0;
      }
      return *this;
    }

   private:
    typename std::unordered_set<EIndex>::const_iterator e_iter_;
    // Half edge index least significant bit.
    int bit_;
  };  // class const_iterator

  class iterator {
   public:
    iterator(typename std::unordered_set<EIndex>::iterator e_iter)
        : e_iter_(e_iter), bit_(0) {}
    iterator(const iterator& iter) : e_iter_(iter.e_iter_), bit_(iter.bit_) {}

    HEIndex operator*() const {
      return static_cast<HEIndex>((static_cast<int>(*(this->e_iter_)) << 1) + this->bit_);
    }

    bool operator==(const iterator& iter) const {
      return this->e_iter_ == iter.e_iter_ && this->bit_ == iter.bit_;
    }

    bool operator!=(const iterator& iter) const {
      return !(*this == iter);
    }

    iterator& operator++() {
      if (this->bit_ == 0) {
	this->bit_ = 1;
      } else {
        ++(this->e_iter_);
        this->bit_ = 0;
      }
      return *this;
    }

   private:
    typename std::unordered_set<EIndex>::iterator e_iter_;
    // Half edge index least significant bit.
    int bit_;
  };  // class iterator

  // Constructor.
  // Parameters:
  //   e_indices - Each edge index corresponds to exactly two half edge indices.
  HalfEdgeIterable(const std::unordered_set<EIndex>* e_indices) : e_indices_(e_indices) {
    CHECK_NE(this->e_indices_, nullptr);
  }

  const_iterator cbegin() {
    return const_iterator(this->e_indices_->cbegin());
  }

  const_iterator cend() {
    return const_iterator(this->e_indices_->cend());
  }

  iterator begin() {
    return iterator(this->e_indices_->begin());
  }

  iterator end() {
    return iterator(this->e_indices_->end());
  }

 private:
  const std::unordered_set<EIndex>* e_indices_;
};  // class HalfEdgeIndicesIterableBase

template<class IndexT>
class SortedIndicesIterableBase {
 public:
  class const_iterator {
   public:
    const_iterator(typename std::vector<IndexT>::const_iterator iter) : iter_(iter) {}
    const_iterator(const const_iterator& iter) : const_iterator(iter.iter_) {}

    IndexT operator*() const {
      return *(this->iter_);
    }

    bool operator==(const const_iterator& iter) const {
      return this->iter_ == iter.iter_;
    }

    bool operator!=(const const_iterator& iter) const {
      return !(*this == iter);
    }

    const_iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    typename std::vector<IndexT>::const_iterator iter_;
  };  // class const_iterator

  class iterator {
   public:
    iterator(typename std::vector<IndexT>::iterator iter) : iter_(iter) {}
    iterator(const iterator& iter) : iterator(iter.iter_) {}

    // Disallow assigning to the temporary sorted vector.
    IndexT operator*() const {
      return *(this->iter_);
    }

    bool operator==(const iterator& iter) const {
      return this->iter_ == iter.iter_;
    }

    bool operator!=(const iterator& iter) const {
      return !(*this == iter);
    }

    iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    typename std::vector<IndexT>::iterator iter_;
  };  // class iterator

  // Constructor.
  SortedIndicesIterableBase(const std::unordered_set<IndexT>& indices) {
    this->indices_.insert(this->indices_.end(), indices.cbegin(), indices.cend());
    std::sort(this->indices_.begin(), this->indices_.end());
  }

  // Constructor used with half edges only.
  // Parameters:
  //   indices - Unsorted indices.
  SortedIndicesIterableBase(std::vector<IndexT> indices) : indices_(indices) {
    std::sort(this->indices_.begin(), this->indices_.end());
  }

  const_iterator cbegin() {
    return const_iterator(this->indices_.cbegin());
  }

  const_iterator cend() {
    return const_iterator(this->indices_.cend());
  }

  iterator begin() {
    return iterator(this->indices_.begin());
  }

  iterator end() {
    return iterator(this->indices_.end());
  }

 private:
  std::vector<IndexT> indices_;
};  // class SortedIndicesIterableBase

using SortedVertexIterable = SortedIndicesIterableBase<VXIndex>;
using SortedFacetIterable = SortedIndicesIterableBase<FAIndex>; 
using SortedEdgeIterable = SortedIndicesIterableBase<EIndex>; 
using SortedHalfEdgeIterable = SortedIndicesIterableBase<HEIndex>; 
using SortedBoundaryHalfEdgeIterable = SortedIndicesIterableBase<HEIndex>; 

// Mesh connectivity without geometric positions.
class MeshConnectivity {
 public:
  // Constructor.
  MeshConnectivity()
    : vertex_connectivities_(1),
      facet_connectivities_(1),
      edge_connectivities_(1),
      half_edge_connectivities_(2) {
     vertex_connectivities_[0].vx_free = kVXInvalid;
     facet_connectivities_[0].fa_free = kFAInvalid;
     edge_connectivities_[0].e_free = kEInvalid;
  }

  virtual ~MeshConnectivity() = default;
  
  // Copy the source connectivity mesh into the current destination mesh.
  // Parameters:
  //   mesh_src - Source connectivity mesh that is copied.
  //   opt_vx_src_for_dst - If not nullptr, receives a map of {vx_src, vx_dst} pairs.
  //   opt_fa_src_for_dst - If not nullptr, receives a map of {fa_src, fa_dst} pairs.
  //   opt_he_src_for_dst - If not nullptr, receives a map of {he_src, he_dst} pairs.
  absl::Status CopyConnectivity(
      const MeshConnectivity& mesh_src,
      std::unordered_map<VXIndex, VXIndex>* opt_vx_src_for_dst,
      std::unordered_map<FAIndex, FAIndex>* opt_fa_src_for_dst,
      std::unordered_map<HEIndex, HEIndex>* opt_he_src_for_dst) {
    std::unordered_map<VXIndex, VXIndex>* vx_src_for_dst = opt_vx_src_for_dst;
    std::unordered_map<VXIndex, VXIndex> local_vx_src_for_dst;
    if (vx_src_for_dst == nullptr) {
      vx_src_for_dst = &local_vx_src_for_dst;
    }

    std::unordered_map<FAIndex, FAIndex>* fa_src_for_dst = opt_fa_src_for_dst;
    std::unordered_map<FAIndex, FAIndex> local_fa_src_for_dst;
    if (fa_src_for_dst == nullptr) {
      fa_src_for_dst = &local_fa_src_for_dst;
    }

    std::unordered_map<HEIndex, HEIndex>* he_src_for_dst = opt_he_src_for_dst;
    std::unordered_map<HEIndex, HEIndex> local_he_src_for_dst;
    if (he_src_for_dst == nullptr) {
      he_src_for_dst = &local_he_src_for_dst;
    }

    // Allocate the vertices.
    for (VXIndex vx_src : mesh_src.GetSortedVXIndices()) {
      VXIndex vx_dst = this->VXAllocate();
      (*vx_src_for_dst)[vx_dst] = vx_src;
    }
    
    // Allocate the facets.
    for (FAIndex fa_src : mesh_src.GetSortedFAIndices()) {
      FAIndex fa_dst = this->FAAllocate();
      (*fa_src_for_dst)[fa_dst] = fa_src;
    }
    
    // Allocate the half edges.
    for (HEIndex he_src : mesh_src.GetSortedHEIndices()) {
      HEIndex he_dst = this->HEAllocate();
      (*he_src_for_dst)[he_dst] = he_src;
      (*he_src_for_dst)[this->HEGetHE(he_dst)] = mesh_src.HEGetHE(he_src);
    }
    
    // Copy vertex fields.
    for (const auto& [vx_dst, vx_src] : *vx_src_for_dst) {
      this->VXSetValence(vx_dst, mesh_src.VXGetValence(vx_src));

      HEIndex he_src = mesh_src.VXGetHE(vx_src);
      HEIndex he_dst = kHEInvalid;
      if (he_src != kHEInvalid) {
        auto iter = he_src_for_dst->find(he_src);
        CHECK(iter != he_src_for_dst->cend());
        he_dst = iter->second;
      }
      this->VXSetHE(vx_dst, he_dst);
    }

    // Copy facet fields.
    for (const auto& [fa_dst, fa_src] : *fa_src_for_dst) {
      this->FASetValence(fa_dst, mesh_src.FAGetValence(fa_src));

      HEIndex he_src = mesh_src.FAGetHE(fa_src);
      HEIndex he_dst = kHEInvalid;
      if (he_src != kHEInvalid) {
        auto iter = he_src_for_dst->find(he_src);
        CHECK(iter != he_src_for_dst->cend());
        he_dst = iter->second;
      }
      this->FASetHE(fa_dst, he_dst);
    }

    // Copy half edge fields.
    for (const auto& [he_dst, he_src] : *he_src_for_dst) {
      HalfEdgeConnectivity* connectivity_dst = this->HEMutableConnectivity(he_dst);
      CHECK_NE(connectivity_dst, nullptr);

      {
        const VXIndex vx_src = mesh_src.HEGetVX(he_src);
        VXIndex vx_dst = kVXInvalid;
        if (vx_src != kVXInvalid) {
          auto iter = vx_src_for_dst->find(vx_src);
          CHECK(iter != vx_src_for_dst->cend());
          vx_dst = iter->second;
        }
        connectivity_dst->vx = vx_dst;
      }

      {
        const FAIndex fa_src = mesh_src.HEGetFA(he_src);
        FAIndex fa_dst = kFAInvalid;
        if (fa_src != kFAInvalid) {
          auto iter = fa_src_for_dst->find(fa_src);
          CHECK(iter != fa_src_for_dst->cend());
          fa_dst = iter->second;
        }
        connectivity_dst->fa = fa_dst;
      }

      {
        const HEIndex he_vx_next_src = mesh_src.HEGetVXNext(he_src);
        CHECK_NE(he_vx_next_src, kHEInvalid);
        auto iter = he_src_for_dst->find(he_vx_next_src);
        CHECK(iter != he_src_for_dst->cend());
        const HEIndex he_vx_next_dst = iter->second;
        connectivity_dst->he_vx_next = he_vx_next_dst;
      }

      {
        const HEIndex he_fa_next_src = mesh_src.HEGetFANext(he_src);
        CHECK_NE(he_fa_next_src, kHEInvalid);
        auto iter = he_src_for_dst->find(he_fa_next_src);
        CHECK(iter != he_src_for_dst->cend());
        const HEIndex he_fa_next_dst = iter->second;
        connectivity_dst->he_fa_next = he_fa_next_dst;
      }
      
      if (mesh_src.HEGetIsBoundary(he_src)) {
        this->HESetIsBoundary(he_dst, true);
      }
    }

    return absl::OkStatus();
  }

  // Vertex methods.
  
  int GetVerticesSize() const {
    return static_cast<int>(this->vxs_.size());
  }

  VertexIterable GetVXIndices() const {
    return VertexIterable(&(this->vxs_));
  }

  SortedVertexIterable GetSortedVXIndices() const {
    return SortedVertexIterable(this->vxs_);
  }

  int VXGetValence(VXIndex vx) const {
    return this->VXGetConnectivity(vx).valence;
  }

  void VXSetValence(VXIndex vx, int valence) {
    this->VXMutableConnectivity(vx)->valence = valence;
  }

  HEIndex VXGetHE(VXIndex vx) const {
    return this->VXGetConnectivity(vx).he;    
  }

  void VXSetHE(VXIndex vx, HEIndex he) {
    this->VXMutableConnectivity(vx)->he = he;
  }

  VXIndex VXAllocate() {
    VXIndex vx = this->vertex_connectivities_[0].vx_free;
    if (vx == kVXInvalid) {
      vx = this->VXNew();
    } else {
      this->vertex_connectivities_[0].vx_free = 
          this->vertex_connectivities_[this->vertex_connectivities_[0].vx_free].vx_free;
    }
    this->VXInit(vx);
    auto [iter, is_inserted] = this->vxs_.insert(vx);
    CHECK(is_inserted);
    return vx;
  }

  void VXDeallocate(VXIndex vx) {
    this->VXClear(vx);
    this->vertex_connectivities_[vx].vx_free = this->vertex_connectivities_[0].vx_free;
    this->vertex_connectivities_[0].vx_free = vx;
    CHECK_EQ(this->vxs_.erase(vx), 1);
  }

  // Facet methods.
  
  int GetFacetsSize() const {
    return static_cast<int>(this->fas_.size());
  }

  FacetIterable GetFAIndices() const {
    return FacetIterable(&(this->fas_));
  }

  SortedFacetIterable GetSortedFAIndices() const {
    return SortedFacetIterable(this->fas_);
  }

  int FAGetValence(FAIndex fa) const {
    return this->FAGetConnectivity(fa).valence;
  }

  void FASetValence(FAIndex fa, int valence) {
    this->FAMutableConnectivity(fa)->valence = valence;
  }

  HEIndex FAGetHE(FAIndex fa) const {
    return this->FAGetConnectivity(fa).he;    
  }

  void FASetHE(FAIndex fa, HEIndex he) {
    this->FAMutableConnectivity(fa)->he = he;
  }

  FAIndex FAAllocate() {
    FAIndex fa = this->facet_connectivities_[0].fa_free;
    if (fa == kFAInvalid) {
      fa = this->FANew();
    } else {
      this->facet_connectivities_[0].fa_free = 
          this->facet_connectivities_[this->facet_connectivities_[0].fa_free].fa_free;
    }
    this->FAInit(fa);
    auto [iter, is_inserted] = this->fas_.insert(fa);
    CHECK(is_inserted);
    return fa;
  }

  void FADeallocate(FAIndex fa) {
    this->FAClear(fa);
    this->facet_connectivities_[fa].fa_free = this->facet_connectivities_[0].fa_free;
    this->facet_connectivities_[0].fa_free = fa;
    CHECK_EQ(this->fas_.erase(fa), 1);
  }

  // Edge methods.
  
  int GetEdgesSize() const {
    return static_cast<int>(this->es_.size());
  }

  EdgeIterable GetEIndices() const {
    return EdgeIterable(&(this->es_));
  }

  SortedEdgeIterable GetSortedEIndices() const {
    return SortedEdgeIterable(this->es_);
  }

  HEIndex EGetHE(EIndex e) const {
    // The first half edge = 2 * e.
    return static_cast<EIndex>(static_cast<int>(e) << 1);
  }

  // Half edge methods.
  
  int GetHalfEdgesSize() const {
    return static_cast<int>(this->es_.size()) << 1;
  }

  HalfEdgeIterable GetHEIndices() const {
    return HalfEdgeIterable(&(this->es_));
  }

  SortedHalfEdgeIterable GetSortedHEIndices() const {
    const int size = this->GetHalfEdgesSize();
    std::vector<HEIndex> unsorted_hes(size);
    int i = 0;
    for (EIndex e : this->es_) {
      const HEIndex he = this->EGetHE(e);
      CHECK_LT(i, size);
      unsorted_hes[i] = he;
      ++i;
      CHECK_LT(i, size);
      unsorted_hes[i] = this->HEGetHE(he);
      ++i;
    }
    CHECK_EQ(i, size);
    return SortedHalfEdgeIterable(unsorted_hes);
  }

  int GetBoundaryHalfEdgesSize() const {
    return static_cast<int>(this->boundary_hes_.size());
  }

  BoundaryHalfEdgeIterable GetBoundaryHEIndices() const {
    return BoundaryHalfEdgeIterable(&(this->boundary_hes_));
  }

  SortedBoundaryHalfEdgeIterable GetSortedBoundaryHEIndices() const {
    return SortedBoundaryHalfEdgeIterable(this->boundary_hes_);
  }

  VXIndex HEGetVX(HEIndex he) const {
    return this->HEGetConnectivity(he).vx;
  }

  void HESetVX(HEIndex he, VXIndex vx) {
    this->HEMutableConnectivity(he)->vx = vx;
  }

  FAIndex HEGetFA(HEIndex he) const {
    return this->HEGetConnectivity(he).fa;
  }

  void HESetFA(HEIndex he, FAIndex fa) {
    this->HEMutableConnectivity(he)->fa = fa;
  }

  EIndex HEGetE(HEIndex he) const {
    return static_cast<EIndex>(static_cast<int>(he) >> 1);
  }

  HEIndex HEGetHE(HEIndex he) const {
    return static_cast<HEIndex>(static_cast<int>(he) ^ 0x1);
  }

  HEIndex HEGetVXNext(HEIndex he) const {
    return this->HEGetConnectivity(he).he_vx_next;
  }

  HEIndex HEGetVXPrev(HEIndex he) const {
    return this->HEGetFANext(this->HEGetHE(he));
  }

  HEIndex HEGetFANext(HEIndex he) const {
    return this->HEGetConnectivity(he).he_fa_next;
  }

  HEIndex HEGetFAPrev(HEIndex he) const {
    return this->HEGetHE(this->HEGetVXNext(he));
  }

  bool HEGetIsBoundary(HEIndex he) const {
    return this->boundary_hes_.find(he) != this->boundary_hes_.cend();
  }
  
  void HESetIsBoundary(HEIndex he, bool is_boundary) {
    if (is_boundary) {
      this->boundary_hes_.insert(he);
    } else {
      this->boundary_hes_.erase(he);
    }
  }
  
  HEIndex HEAllocate() {
    EIndex e = this->edge_connectivities_[0].e_free;
    if (e == kEInvalid) {
      // This allocates the edge and the two associated half edges.
      e = this->ENew();
    } else {
      this->edge_connectivities_[0].e_free = 
          this->edge_connectivities_[this->edge_connectivities_[0].e_free].e_free;
    }
    // Initializes the two associated half edge to be opposite pairs.
    this->EInit(e);
    auto [iter, is_inserted] = this->es_.insert(e);
    CHECK(is_inserted);
    return this->EGetHE(e);
  }

  void HEDeallocate(HEIndex he) {
    const HEIndex hes[2] = {he, this->HEGetHE(he)};
    for (int i = 0; i < 2; ++i) {
      auto iter = this->boundary_hes_.find(hes[i]);
      if (iter != this->boundary_hes_.end()) {
	this->boundary_hes_.erase(iter);
      }
      this->HEClear(hes[i]);
    }

    const EIndex e = this->HEGetE(he);
    this->EClear(e);
    this->edge_connectivities_[e].e_free = this->edge_connectivities_[0].e_free;
    this->edge_connectivities_[0].e_free = e;
    CHECK_EQ(this->es_.erase(e), 1);
  }

  void Splice(HEIndex he0, HEIndex he1) {
    const HEIndex he_fa_prev0 = this->HEGetFAPrev(he0);
    const HEIndex he_fa_prev1 = this->HEGetFAPrev(he1);
    this->half_edge_connectivities_[he_fa_prev0].he_fa_next = he1;
    this->half_edge_connectivities_[he_fa_prev1].he_fa_next = he0;
    this->half_edge_connectivities_[he0].he_vx_next = this->HEGetHE(he_fa_prev1);
    this->half_edge_connectivities_[he1].he_vx_next = this->HEGetHE(he_fa_prev0);
  }

  // Debugging methods.
  
  std::string Display() const {
    std::string buffer;

    absl::StrAppend(&buffer, "BEGIN: Mesh\n");

    absl::StrAppend(&buffer, "  BEGIN: Euler characteristic\n");
    const int vertices_size = this->GetVerticesSize();
    const int facets_size = this->GetFacetsSize();
    const int half_edges_size = this->GetHalfEdgesSize();
    const int edges_size = this->GetEdgesSize();
    absl::StrAppend(&buffer, "    2 = V - E + F = ", vertices_size, " - ", edges_size, " + ",
		    facets_size, " = ", (vertices_size - edges_size + facets_size), "\n");
    absl::StrAppend(&buffer, "  END  : Euler characteristic\n");

    absl::StrAppend(&buffer, "  BEGIN: Vertices : size = ", vertices_size, "\n");
    for (VXIndex vx : this->GetSortedVXIndices()) {
      absl::StrAppend(&buffer, "    BEGIN: vx", vx, " : valence = ", this->VXGetValence(vx),
		      "\n");
      this->VXDisplay(vx, &buffer);
      absl::StrAppend(&buffer, "    END  : vx", vx, "\n");
    }
    absl::StrAppend(&buffer, "  END  : Vertices : size = ", vertices_size, "\n");

    absl::StrAppend(&buffer, "  BEGIN: Facets : size = ", facets_size, "\n");
    for (FAIndex fa : this->GetSortedFAIndices()) {
      absl::StrAppend(&buffer, "    BEGIN: fa", fa, " : valence = ", this->FAGetValence(fa),
		      "\n");
      this->FADisplay(fa, &buffer);
      absl::StrAppend(&buffer, "    END  : fa", fa, "\n");
    }
    absl::StrAppend(&buffer, "  END  : Facets : size = ", facets_size, "\n");

    absl::StrAppend(&buffer, "  BEGIN: Half edges : size = ", half_edges_size, "\n");
    for (HEIndex he : this->GetSortedHEIndices()) {
      absl::StrAppend(&buffer, "    BEGIN: he", he, "\n");
      this->HEDisplay(he, &buffer);
      absl::StrAppend(&buffer, "    END  : he", he, "\n");
    }
    absl::StrAppend(&buffer, "  END  : Half edges : size = ", half_edges_size, "\n");

    const int boundary_half_edges_size = this->GetBoundaryHalfEdgesSize();
    absl::StrAppend(&buffer, "  BEGIN: Boundary half edges : size = ", boundary_half_edges_size,
		    "\n");
    if (0 < boundary_half_edges_size) {
      absl::StrAppend(&buffer, "   ");
      for (HEIndex he : this->GetSortedBoundaryHEIndices()) {
        absl::StrAppend(&buffer, " he", he);
      }
      absl::StrAppend(&buffer, "\n");
    }
    absl::StrAppend(&buffer, "  END  : Boundary half edges : size = ", boundary_half_edges_size,
		    "\n");

    absl::StrAppend(&buffer, "END  : Mesh\n");

    return buffer;
  }

 protected:
  static constexpr VXIndex kVXFreeInvalid = VXIndex(-1);
  static constexpr FAIndex kFAFreeInvalid = FAIndex(-1);
  static constexpr EIndex kEFreeInvalid = EIndex(-1);

  struct VertexConnectivity {
    bool IsAllocated() const {
      return this->vx_free == kVXFreeInvalid;
    }

    void Clear() {
      this->valence = 0;
      this->he = kHEInvalid;
      this->vx_free = kVXInvalid;
    }
    
    int valence = 0;
    // One half edge in the vertex ring.
    HEIndex he = kHEInvalid;
    VXIndex vx_free = kVXFreeInvalid;
  };  // struct VertexConnectivity

  struct FacetConnectivity {
    bool IsAllocated() const {
      return this->fa_free == kFAFreeInvalid;
    }

    void Clear() {
      this->valence = 0;
      this->he = kHEInvalid;
      this->fa_free = kFAInvalid;
    }
    
    int valence = 0;
    // One half edge in the facet ring.
    HEIndex he = kHEInvalid;
    FAIndex fa_free = kFAFreeInvalid;
  };  // struct FacetConnectivity

  struct EdgeConnectivity {
    bool IsAllocated() const {
      return this->e_free == kEFreeInvalid;
    }

    void Clear() {
      this->e_free = kEInvalid;
    }
    
    FAIndex e_free = kEFreeInvalid;
  };  // struct EdgeConnectivity

  struct HalfEdgeConnectivity {
    void Clear() {
      this->vx = kVXInvalid;
      this->fa = kFAInvalid;
      this->he_vx_next = kHEInvalid;
      this->he_fa_next = kHEInvalid;
    }
    
    // Origin vertex of the half edge.
    VXIndex vx = kVXInvalid;
    // Left facet of the half edge.
    FAIndex fa = kFAInvalid;
    // Next half edge in the vertex loop.
    HEIndex he_vx_next = kHEInvalid;
    // Next half edge in the facet loop.
    HEIndex he_fa_next = kHEInvalid;
  };  // struct HalfEdgeConnectivity

  // Vertex methods.

  bool IsValidVXIndex(VXIndex vx) const {
    return VXIndex(0) < vx && vx < static_cast<int>(this->vertex_connectivities_.size());
  }

  bool IsAllocatedVXIndex(VXIndex vx) const {
    if (!this->IsValidVXIndex(vx)) {
      return false;
    }
    return this->vertex_connectivities_[vx].IsAllocated();
  }

  const VertexConnectivity& VXGetConnectivity(VXIndex vx) const {
    CHECK(this->IsValidVXIndex(vx));
    const VertexConnectivity& vertex_connectivity = this->vertex_connectivities_[vx];
    CHECK(vertex_connectivity.IsAllocated());
    return vertex_connectivity;
  }

  VertexConnectivity* VXMutableConnectivity(VXIndex vx) {
    CHECK(this->IsValidVXIndex(vx));
    VertexConnectivity* vertex_connectivity = &(this->vertex_connectivities_[vx]);
    CHECK(vertex_connectivity->IsAllocated());
    return vertex_connectivity;
  }

  // Resize the vertex arrays to add a new vertex.
  virtual VXIndex VXNew() {
    VXIndex vx = static_cast<VXIndex>(this->vertex_connectivities_.size());
    this->vertex_connectivities_.resize(static_cast<std::size_t>(vx) + 1);
    return vx;
  }

  virtual void VXInit(VXIndex vx) {
    CHECK(this->IsValidVXIndex(vx));
    VertexConnectivity* vertex_connectivity = &(this->vertex_connectivities_[vx]);
    vertex_connectivity->Clear();
    vertex_connectivity->vx_free = kVXFreeInvalid;
  }

  virtual void VXClear(VXIndex vx) {
    this->VXMutableConnectivity(vx)->Clear();
  }

  virtual void VXDisplay(VXIndex vx, std::string *buffer) const {
    CHECK_NE(buffer, nullptr);
    const HEIndex he_begin = this->VXGetHE(vx);
    absl::StrAppend(buffer, "      he", he_begin, "\n");

    // Vertex neighbors.
    HEIndex he = he_begin;
    absl::StrAppend(buffer, "     ");
    if (he != kHEInvalid) {
      do {
	absl::StrAppend(buffer, " vx", this->HEGetVX(this->HEGetHE(he)));
	he = this->HEGetVXNext(he);
      } while (he != he_begin);
    }
    absl::StrAppend(buffer, "\n");
    
    // Facet neighbors.
    he = he_begin;
    absl::StrAppend(buffer, "     ");
    if (he != kHEInvalid) {
      do {
	absl::StrAppend(buffer, " fa", this->HEGetFA(he));
	he = this->HEGetVXNext(he);
      } while (he != he_begin);
    }
    absl::StrAppend(buffer, "\n");
  }

  // Facet methods.

  bool IsValidFAIndex(FAIndex fa) const {
    return FAIndex(0) < fa && fa < static_cast<int>(this->facet_connectivities_.size());
  }

  bool IsAllocatedFAIndex(FAIndex fa) const {
    if (!this->IsValidFAIndex(fa)) {
      return false;
    }
    return this->facet_connectivities_[fa].IsAllocated();
  }

  const FacetConnectivity& FAGetConnectivity(FAIndex fa) const {
    CHECK(this->IsValidFAIndex(fa));
    const FacetConnectivity& facet_connectivity = this->facet_connectivities_[fa];
    CHECK(facet_connectivity.IsAllocated());
    return facet_connectivity;
  }

  FacetConnectivity* FAMutableConnectivity(FAIndex fa) {
    CHECK(this->IsValidFAIndex(fa));
    FacetConnectivity* facet_connectivity = &(this->facet_connectivities_[fa]);
    CHECK(facet_connectivity->IsAllocated());
    return facet_connectivity;
  }

  // Resize the facet arrays to add a new facet.
  virtual FAIndex FANew() {
    FAIndex fa = static_cast<FAIndex>(this->facet_connectivities_.size());
    this->facet_connectivities_.resize(static_cast<std::size_t>(fa) + 1);
    return fa;
  }

  virtual void FAInit(FAIndex fa) {
    CHECK(this->IsValidFAIndex(fa));
    FacetConnectivity* facet_connectivity = &(this->facet_connectivities_[fa]);
    facet_connectivity->Clear();
    facet_connectivity->fa_free = kFAFreeInvalid;
  }

  virtual void FAClear(FAIndex fa) {
    this->FAMutableConnectivity(fa)->Clear();
  }

  virtual void FADisplay(FAIndex fa, std::string *buffer) const {
    CHECK_NE(buffer, nullptr);
    const HEIndex he_begin = this->FAGetHE(fa); 
    absl::StrAppend(buffer, "      he", he_begin, "\n");    

    // Vertex neighbors.
    HEIndex he = he_begin;
    absl::StrAppend(buffer, "     ");
    if (he != kHEInvalid) {
      do {
	absl::StrAppend(buffer, " vx", this->HEGetVX(he));
	he = this->HEGetFANext(he);
      } while (he != he_begin);
    }
    absl::StrAppend(buffer, "\n");
    
    // Facet neighbors.
    he = he_begin;
    absl::StrAppend(buffer, "     ");
    if (he != kHEInvalid) {
      do {
	absl::StrAppend(buffer, " fa", this->HEGetFA(this->HEGetHE(he)));
	he = this->HEGetFANext(he);
      } while (he != he_begin);
    }
    absl::StrAppend(buffer, "\n");
  }
  
  // Edge methods.

  bool IsValidEIndex(EIndex e) const {
    return EIndex(0) < e && e < static_cast<int>(this->edge_connectivities_.size());
  }

  bool IsAllocatedEIndex(EIndex e) const {
    if (!this->IsValidEIndex(e)) {
      return false;
    }
    return this->edge_connectivities_[e].IsAllocated();
  }

  const EdgeConnectivity& EGetConnectivity(EIndex e) const {
    CHECK(this->IsValidEIndex(e));
    const EdgeConnectivity& edge_connectivity = this->edge_connectivities_[e];
    CHECK(edge_connectivity.IsAllocated());
    return edge_connectivity;
  }

  EdgeConnectivity* EMutableConnectivity(EIndex e) {
    CHECK(this->IsValidEIndex(e));
    EdgeConnectivity* edge_connectivity = &(this->edge_connectivities_[e]);
    CHECK(edge_connectivity->IsAllocated());
    return edge_connectivity;
  }

  // Resize the edge arrays to add a new edge.
  virtual EIndex ENew() {
    const EIndex e = static_cast<EIndex>(this->edge_connectivities_.size());
    this->edge_connectivities_.resize(static_cast<std::size_t>(e) + 1);
    // Allocate the two associated half edges.
    this->HENew();
    return e;
  }

  virtual void EInit(EIndex e) {
    CHECK(this->IsValidEIndex(e));
    EdgeConnectivity* edge_connectivity = &(this->edge_connectivities_[e]);
    edge_connectivity->Clear();
    edge_connectivity->e_free = kEFreeInvalid;

    const HEIndex he = this->EGetHE(e);
    const HEIndex hes[2] = {he, this->HEGetHE(he)};
    HalfEdgeConnectivity* half_edge_connectivities[2];
    for (int i = 0; i < 2; ++i) {
      this->HEInit(hes[i]);
      half_edge_connectivities[i] = &(this->half_edge_connectivities_[hes[i]]);
    }
    half_edge_connectivities[0]->he_fa_next = hes[1];
    half_edge_connectivities[1]->he_fa_next = hes[0];    
  }

  virtual void EClear(EIndex e) {
    this->EMutableConnectivity(e)->Clear();
  }

  // Half_Edge methods.

  bool IsValidHEIndex(HEIndex he) const {
    return this->IsValidEIndex(this->HEGetE(he));
  }

  bool IsAllocatedHEIndex(HEIndex he) const {
    return this->IsAllocatedEIndex(this->HEGetE(he));
  }

  const HalfEdgeConnectivity& HEGetConnectivity(HEIndex he) const {
    CHECK(this->IsAllocatedEIndex(this->HEGetE(he)));
    return this->half_edge_connectivities_[he];
  }

  HalfEdgeConnectivity* HEMutableConnectivity(HEIndex he) {
    CHECK(this->IsAllocatedEIndex(this->HEGetE(he)));
    return &(this->half_edge_connectivities_[he]);
  }

  // Resize the half edge arrays to add two new half edges.
  virtual void HENew() {
    this->half_edge_connectivities_.resize(this->half_edge_connectivities_.size() + 2);
  }

  virtual void HEInit(HEIndex he) {
    CHECK(this->IsValidHEIndex(he));
    HalfEdgeConnectivity* half_edge_connectivity = &(this->half_edge_connectivities_[he]);
    half_edge_connectivity->Clear();
    half_edge_connectivity->he_vx_next = he;
  }

  virtual void HEClear(HEIndex he) {
    this->HEMutableConnectivity(he)->Clear();
  }

  virtual void HEDisplay(HEIndex he, std::string *buffer) const {
    CHECK_NE(buffer, nullptr);
    absl::StrAppend(buffer, "      vx", this->HEGetVX(he), " : he", this->HEGetVXNext(he), "\n");
    absl::StrAppend(buffer, "      fa", this->HEGetFA(he), " : he", this->HEGetFANext(he), "\n");
  }
  
  // Allocated vertex indices. 
  std::unordered_set<VXIndex> vxs_;
  std::vector<VertexConnectivity> vertex_connectivities_;  

  // Allocated facet indices. 
  std::unordered_set<FAIndex> fas_;
  std::vector<FacetConnectivity> facet_connectivities_;  

  // Allocated edge indices. 
  std::unordered_set<EIndex> es_;
  std::vector<EdgeConnectivity> edge_connectivities_;  

  std::vector<HalfEdgeConnectivity> half_edge_connectivities_;

  std::unordered_set<HEIndex> boundary_hes_;
};  // class MeshConnectivity

}  // namespace hemesh
