#pragma once
// Half edge mesh.
//     _
//   _| |_
//  |_ \ _|
//    |_|

#include <algorithm>
#include <assert>
#include <unordered_set>
#include <vector>

#ifndef CHECK
#define CHECK(a) assert(a);
#endif  // CHECK

#ifndef CHECK_EQ
#define CHECK_EQ(a, b) assert((a) == (b));
#endif  // CHECK_EQ

#ifndef CHECK_NE
#define CHECK_NE(a, b) assert((a) != (b));
#endif  // CHECK_NE

namespace hemesh {

template <class IndexT>
class IndicesIterableBase {
  class const_iterator {
    const_iterator(std::unordered_set<IndexT>::const_iterator iter) : iter_(iter) {}
    const_iterator(const const_iterator& iter) : const_iterator(iter.iter_) {}

    IndexT operator*() const {
      return->iter_;
    }

    const_iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    std::unordered_set<IndexT>::const_iterator iter_;
  };  // class const_iterator

  class iterator {
    iterator(std::unordered_set<IndexT>::iterator iter) : iter_(iter) {}
    iterator(const iterator& iter) : iterator(iter.iter_) {}

    IndexT operator*() const {
      return->iter_;
    }

    iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    std::unordered_set<IndexT>::iterator iter_;
  };  // class iterator

  IndicesIterableBase(const std::vector<IndexT>* indices) : indices_(indices) {
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
using HalfEdgeIterable = IndicesIterableBase<HEIndex>; 
using FacetIterable = IndicesIterableBase<FAIndex>; 

template <class IndexT>
class SortedIndicesIterableBase {
  class const_iterator {
    const_iterator(std::vector<IndexT>::const_iterator iter) : iter_(iter) {}
    const_iterator(const const_iterator& iter) : const_iterator(iter.iter_) {}

    IndexT operator*() const {
      return->iter_;
    }

    const_iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    std::vector<IndexT>::const_iterator iter_;
  };  // class const_iterator

  class iterator {
    iterator(std::vector<IndexT>::iterator iter) : iter_(iter) {}
    iterator(const iterator& iter) : iterator(iter.iter_) {}

    // Disallow assigning to the temporary sorted vector.
    IndexT operator*() const {
      return->iter_;
    }

    iterator& operator++() {
      ++(this->iter_);
      return *this;
    }

   private:
    std::vector<IndexT>::iterator iter_;
  };  // class iterator

  SortedIndicesIterableBase(const std::unordered_set<IndexT>& indices) {
    this->indices_.insert(this->indices_.end(), indices.cbegin(), indices.cend());
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
using SortedHalfEdgeIterable = SortedIndicesIterableBase<HEIndex>; 
using SortedFacetIterable = SortedIndicesIterableBase<FAIndex>; 

class MeshConnectivity {
 public:
  using VXIndex = int;
  using HEIndex = int;
  using FAIndex = int;

  static constexpr VXIndex kVXInvalid = VXIndex(0);
  static constexpr HEIndex kHEInvalid = HEIndex(0);
  static constexpr FAIndex kFAInvalid = FAIndex(0);

  MeshConnectivity()
   : vertex_connectivities_(1) {
     vertex_connectivities_[0].vx_free = kVXInvalid;
  }

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
    CHECK_EQ(this->vxs_.erase(vx), 1);
    this->VXClear(vx);
    this->vertex_connectivities_[vx].vx_free = this->vertex_connectivities_[0].vx_free;
    this->vertex_connectivities_[0].vx_free = vx;
  }

 protected:
  static constexpr VXIndex kVXFreeInvalid = VXIndex(-1);
  static constexpr HEIndex kHEFreeInvalid = HEIndex(-1);
  static constexpr FAIndex kFAFreeInvalid = FAIndex(-1);

  struct VertexConnectivity {
    bool IsAllocated() const {
      return this->vx_free != kVXFreeInvalid;
    }

    int valence = 0;
    HEIndex he = kHEInvalid;
    VXIndex vx_free = kVXInvalid;
  };  // struct VertexConnectivity

  bool IsValidVXIndex(VXIndex vx) const {
    return VXIndex(0) < vx && vx < static_cast<int>(this->vertex_connectivities_.size());
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
    VXIndex vx = static_cast<VXIndex>(this->vertex_connectivity_.size());
    this->vertex_connectivity_.resize(vx + 1);
    return vx;
  }

  virtual void VXInit(VXIndex vx) {
    CHECK(this->IsValidVXIndex(vx));
    VertexConnectivity* vertex_connectivity = &(this->vertex_connectivities_[vx]);
    vertex_connectivity->valence = 0;
    vertex_connectivity->he = kHEInvalid;
    vertex_connectivity->vx_free = kVXFreeInvalid;
  }

  virtual void VXClear(VXIndex vx) {
    VertexConnectivity* vertex_connectivity = this->VXMutableConnectivity(vx);
    vertex_connectivity->valence = 0;
    vertex_connectivity->he = kHEInvalid;
    vertex_connectivity->vx_free = kVXInvalid;
  }

  // Allocated vertex indices. 
  std::unordered_set<VXIndex> vxs_;
  std::vector<VertexConnectivity> vertex_connectivities_;  
};  // class MeshConnectivity

}  // namespace hemesh
