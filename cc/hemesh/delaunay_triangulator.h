#pragma once

// The Delaunay triangulator calculates the 2D Delaunay triangulation of a set of unique points.

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "cc/geom/delaunay_predicates.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/math/vector.h"

namespace hemesh {

// The Delaunay triangulator calculates the 2D Delaunay triangulation of a set of unique points.
template<class ScalarT>
class DelaunayTriangulator {
 public:
  virtual ~DelaunayTriangulator() = default;

  // Factory methods for the default implementation.
  static std::unique_ptr<DelaunayTriangulator> New(const geom::DelaunayPredicates<ScalarT>* preds);
  static std::unique_ptr<DelaunayTriangulator> New(
      std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned);

  // Factory methods for a divide and conquer implementation.
  static std::unique_ptr<DelaunayTriangulator> NewDivideAndConquer(
      const geom::DelaunayPredicates<ScalarT>* preds);
  static std::unique_ptr<DelaunayTriangulator> NewDivideAndConquer(
      std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned);

  // Creates the Delaunay triangulation.  Duplicate points are ignored.
  // Parameters:
  //   points - Input 2D point set.
  //   opt_index_for_vx - If not nullptr, receives a map from the output vx to the input indices.
  // Returns the Delaunay triangulation mesh.
  virtual std::unique_ptr<MeshGeometry<ScalarT, 2>> Triangulate(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      std::unordered_map<VXIndex, int>* opt_index_for_vx) const = 0;
};  // class DelaunayTriangulator

namespace internal {

// DelaunayTriangulator divide and conquer implementation.
template<class ScalarT>
class DelaunayTriangulatorDivideAndConquer : public DelaunayTriangulator<ScalarT> {
 public:
  // Compares two points by x and then y coordinate.
  struct PointComparator {
    PointComparator() = default;

    // Returns true if a.x < b.x || (a.x == b.x && a.y < b.y).
    bool operator()(const math::ConstVector<ScalarT, 2>& a,
                    const math::ConstVector<ScalarT, 2>& b) const {
      return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    }
  };  // struct PointComparator
  
  // Compares two points by x and then y coordinate.
  struct PointIndexComparator {
    PointIndexComparator(const std::vector<math::VectorOwned<ScalarT, 2>>* points)
        : points(points) {
      CHECK_NE(this->points, nullptr);
    }

    // Returns true if a.x < b.x || (a.x == b.x && a.y < b.y).
    bool operator()(int a_index, int b_index) const {
      const int size = static_cast<int>(this->points->size());
      CHECK_LE(0, a_index);
      CHECK_LT(a_index, size);
      const math::ConstVector<ScalarT, 2>& a = (*(this->points))[a_index];
      CHECK_LE(0, b_index);
      CHECK_LT(b_index, size);
      const math::ConstVector<ScalarT, 2>& b = (*(this->points))[b_index];
      return this->point_comparator(a, b);
    }

    const std::vector<math::VectorOwned<ScalarT, 2>>* points;
    const PointComparator point_comparator;
  };  // struct PointIndexComparator

  // Constructor.
  // Parameters:
  //   preds - Delaunay predicates must not be nullptr.
  //   opt_preds_owned - If not nullptr, owned pointer to the same Delaunay predicates.
  DelaunayTriangulatorDivideAndConquer(
      const geom::DelaunayPredicates<ScalarT>* preds,
      std::unique_ptr<geom::DelaunayPredicates<ScalarT>> opt_preds_owned)
      : preds_(preds), preds_owned_(std::move(opt_preds_owned)) {
    CHECK_NE(this->preds_, nullptr);
  }
  virtual ~DelaunayTriangulatorDivideAndConquer() = default;

  std::unique_ptr<MeshGeometry<ScalarT, 2>> Triangulate(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      std::unordered_map<VXIndex, int>* opt_index_for_vx) const override {
    std::unordered_map<VXIndex, int>* index_for_vx = opt_index_for_vx;

    std::unique_ptr<MeshGeometry<ScalarT, 2>> mesh = std::make_unique<MeshGeometry<ScalarT, 2>>();

    const int raw_size = static_cast<int>(points.size());
    if (raw_size < 1) {
      // Empty mesh.
      return mesh;
    }

    // Sort the points by x and then y coordinate.
    std::vector<int> sorted_point_indices(raw_size);
    for (int i = 0; i < raw_size; ++i) {
      sorted_point_indices[i] = i;
    }    
    const PointIndexComparator comparator(&points);
    std::sort(sorted_point_indices.begin(), sorted_point_indices.end(), comparator);

    // Remove duplicate points.
    int size = 0;
    int i = 0;
    for (++i; i < raw_size; ++i) {
      if (comparator(sorted_point_indices[size], sorted_point_indices[i])) {
        // Not coincident.
        ++size;
        if (size < i) {
          sorted_point_indices[size] = sorted_point_indices[i];
        }
      }
    }
    ++size;
    sorted_point_indices.resize(size);

    const HEIndex he_bottom = DivideAndConquer(
        points, sorted_point_indices, 0, size - 1, mesh.get(), index_for_vx);
    if (he_bottom == kHEInvalid) {
      // There is only one vertex.
      return mesh;
    }
    
    // Mark the boundary half edges.
    HEIndex he = he_bottom;
    do {
      mesh->HESetIsBoundary(he, true);

      he = mesh->HEGetFANext(he);
    } while (he != he_bottom);

    return mesh;
  }

 private:
  // The recursive divide and conquer algorithm splits the points in x. Creates simple
  // triangulations of 1, 2, or 3 points. Then it merges two submeshes using the rising bubble
  // algorithm.
  // Parameters:
  //   points - Input 2D point set.
  //   sorted_point_indices - Point indices sorted by x and then y coordinate.
  //   min_index - Minimum index into the sorted point indices for the span.
  //   max_index - Maximum index into the sorted point indices for the span.
  //   mesh - Mesh that receives the Delaunay triangulation.
  //   index_for_vx - If not nullptr, receives a map from the output vx to the input indices.
  // Returns the bottom convex hull half edge.
  HEIndex DivideAndConquer(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      const std::vector<int>& sorted_point_indices,
      int min_index, int max_index,
      MeshGeometry<ScalarT, 2>* mesh,
      std::unordered_map<VXIndex, int>* index_for_vx) const {
    CHECK_LE(min_index, max_index);
    CHECK_NE(mesh, nullptr);

    static constexpr ScalarT kZero(0);
    
    if (max_index <= min_index) {
      // There is only one vertex in the mesh.
      const VXIndex vx = this->VXAllocate(
          points, sorted_point_indices, min_index, mesh, index_for_vx);
      mesh->VXSetValence(vx, 0);
      // There are no convex hull half edges.
      return kHEInvalid;
    }

    if (min_index + 1 == max_index) {
      // There are two points, so create two vertices connected by one edge.
      const int indices[2] = {min_index, max_index};
      const HEIndex he_bottom = mesh->HEAllocate();
      const HEIndex hes[2] = {he_bottom, mesh->HEGetHE(he_bottom)};
      VXIndex vxs[2];
      for (int i = 0; i < 2; ++i) {
        vxs[i] = this->VXAllocate(
            points, sorted_point_indices, indices[i], mesh, index_for_vx);
        mesh->VXSetValence(vxs[i], 1);
        mesh->VXSetHE(vxs[i], hes[i]);
        mesh->HESetVX(hes[i], vxs[i]);
      }
      return he_bottom;
    }

    if (min_index + 2 == max_index) {
      // There are three points.
      int indices[3] = {min_index, min_index + 1, max_index};
      const ScalarT ccw = this->preds_->CCWInscribedCircle_VVV(
          points[sorted_point_indices[indices[0]]],
          points[sorted_point_indices[indices[1]]],
          points[sorted_point_indices[indices[2]]]);
      if (ccw < kZero) {
        // The three points are CW, so swap two of them.
        std::swap(indices[1], indices[2]);
      }

      // Allocate the vertices.
      VXIndex vxs[3];
      for (int i = 0; i < 3; ++i) {
        vxs[i] = this->VXAllocate(
            points, sorted_point_indices, indices[i], mesh, index_for_vx);
      }

      HEIndex he_bottom = kHEInvalid;
      if (ccw == kZero) {
        // The three points are collinear.
        mesh->VXSetValence(vxs[0], 1);
        mesh->VXSetValence(vxs[1], 2);
        mesh->VXSetValence(vxs[2], 1);
        HEIndex hes[2];
        for (int i = 0; i < 2; ++i) {
          hes[i] = mesh->HEAllocate();
          mesh->VXSetHE(vxs[i], hes[i]);
          mesh->HESetVX(hes[i], vxs[i]);
          mesh->HESetVX(mesh->HEGetHE(hes[i]), vxs[i + 1]);
        }
        mesh->VXSetHE(vxs[2], mesh->HEGetHE(hes[1]));

        mesh->Splice(mesh->HEGetHE(hes[0]), hes[1]);

        he_bottom = mesh->HEGetHE(hes[1]);
      } else {
        // Create a triangle.
        const FAIndex fa = mesh->FAAllocate();
        mesh->FASetValence(fa, 3);
        HEIndex he_begin = kHEInvalid;
        HEIndex he_prev = kHEInvalid;
        for (int i = 0; i < 3; ++i) {
          mesh->VXSetValence(vxs[i], 2);

          const HEIndex he = mesh->HEAllocate();
          mesh->VXSetHE(vxs[i], he);
          mesh->HESetVX(he, vxs[i]);
          if (he_begin == kHEInvalid) {
            he_begin = he;
            mesh->FASetHE(fa, he_begin);
          }
          mesh->HESetFA(he, fa);

          if (he_prev != kHEInvalid) {
            mesh->HESetVX(mesh->HEGetHE(he_prev), vxs[i]);
            mesh->Splice(mesh->HEGetHE(he_prev), he);
          }
          
          he_prev = he;
        }
        CHECK_NE(he_begin, kHEInvalid);
        CHECK_NE(he_prev, kHEInvalid);
        mesh->HESetVX(mesh->HEGetHE(he_prev), mesh->HEGetVX(he_begin));
        mesh->Splice(mesh->HEGetHE(he_prev), he_begin);

        he_bottom = mesh->HEGetHE(he_begin);
      }

      return he_bottom;
    }

    // Divide and create the left and right submeshes.
    const int mid_index = (min_index + max_index) / 2;
    const HEIndex he_left = DivideAndConquer(
      points, sorted_point_indices, min_index, mid_index, mesh, index_for_vx);
    const HEIndex he_right = DivideAndConquer(
      points, sorted_point_indices, mid_index + 1, max_index, mesh, index_for_vx);

    // Merge.
    return this->Merge(he_left, he_right, mesh);
  }

  // The bottom convex hull half edge connects the left and right half edges.
  struct ConvexHullBottom {
    HEIndex he_left;
    HEIndex he_right;
  };  // struct ConvexHullBottom

  // Merges two Delaunay submeshes.
  // Parameters:
  //   he_left - Bottom convex hull half edge from the left submesh.
  //   he_right - Bottom convex hull half edge from the right submesh.
  //   mesh - Mesh that receives the Delaunay triangulation.
  // Returns the bottom convex hull half edge.
  HEIndex Merge(HEIndex he_left, HEIndex he_right, MeshGeometry<ScalarT, 2>* mesh) const {
    CHECK_NE(mesh, nullptr);

    static constexpr ScalarT kZero(0);
    static constexpr ScalarT kOne(1);
    
    // Find where the submesh convex hull should be connected at the bottom.
    const ConvexHullBottom bottom = this->FindConvexHullBottom(*mesh, he_left, he_right);
    he_left = bottom.he_left;
    CHECK_NE(he_left, kHEInvalid);
    CHECK_EQ(mesh->HEGetFA(he_left), kFAInvalid);
    he_right = bottom.he_right;
    CHECK_NE(he_right, kHEInvalid);
    CHECK_EQ(mesh->HEGetFA(he_right), kFAInvalid);
    
    // Allocate the bottom convex hull edge and connect the left and right submeshes.
    HEIndex he_base = mesh->HEAllocate();
    const HEIndex he_bottom = mesh->HEGetHE(he_base);

    he_left = mesh->HEGetFAPrev(he_left);
    he_right = mesh->HEGetFANext(he_right);

    mesh->HESetVX(he_base, mesh->HEGetVX(mesh->HEGetHE(he_left)));
    mesh->VXSetValence(mesh->HEGetVX(he_base), mesh->VXGetValence(mesh->HEGetVX(he_base)) + 1);
    mesh->HESetVX(mesh->HEGetHE(he_base), mesh->HEGetVX(he_right));
    mesh->VXSetValence(mesh->HEGetVX(mesh->HEGetHE(he_base)),
                       mesh->VXGetValence(mesh->HEGetVX(mesh->HEGetHE(he_base))) + 1);
    
    mesh->Splice(mesh->HEGetFANext(he_left), he_base);
    mesh->Splice(he_right, he_bottom);

    while (true) {
      // Remove non-Delaunay triangles on the left.
      while (mesh->HEGetFA(mesh->HEGetHE(he_left)) != kFAInvalid &&
             mesh->HEGetFA(mesh->HEGetHE(he_left)) != mesh->HEGetFA(mesh->HEGetHE(he_base)) &&
             this->preds_->InInscribedCircle_VVV_V(
                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetFAPrev(mesh->HEGetHE(he_left)))),
                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_left))),
                 mesh->VXGetPoint(mesh->HEGetVX(he_left)),
                 mesh->VXGetPoint(mesh->HEGetVX(he_right))) < kZero) {
        // Remove the facet.
        HEIndex he_remove = he_left;
        he_left = mesh->HEGetVXPrev(he_left);
        mesh->FADeallocate(mesh->HEGetFA(he_left));
        mesh->HESetFA(he_left, kFAInvalid);

        mesh->VXSetValence(mesh->HEGetVX(he_remove),
                           mesh->VXGetValence(mesh->HEGetVX(he_remove)) - 1);
        if (mesh->VXGetHE(mesh->HEGetVX(he_remove)) == he_remove) {
          mesh->VXSetHE(mesh->HEGetVX(he_remove), he_left);
        }

        mesh->Splice(he_left, he_remove);

        he_left = mesh->HEGetFANext(he_left);
        mesh->HESetFA(he_left, kFAInvalid);

        he_remove = mesh->HEGetHE(he_remove);
        mesh->VXSetValence(mesh->HEGetVX(he_remove),
                           mesh->VXGetValence(mesh->HEGetVX(he_remove)) - 1);
        if (mesh->VXGetHE(mesh->HEGetVX(he_remove)) == he_remove) {
          mesh->VXSetHE(mesh->HEGetVX(he_remove), mesh->HEGetHE(he_left));
        }

        mesh->Splice(mesh->HEGetVXPrev(he_remove), he_remove);

        mesh->HEDeallocate(he_remove);
      }

      // Remove non-Delaunay triangles on the right.
      while (mesh->HEGetFA(mesh->HEGetHE(he_right)) != kFAInvalid &&
             mesh->HEGetFA(mesh->HEGetHE(he_right)) != mesh->HEGetFA(mesh->HEGetHE(he_base)) &&
             this->preds_->InInscribedCircle_VVV_V(
                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_right))),
                 mesh->VXGetPoint(mesh->HEGetVX(he_right)),
                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(mesh->HEGetVXPrev(he_right)))),
                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_left)))) < kZero) {
        // Remove the facet.
        he_right = mesh->HEGetHE(he_right);
        HEIndex he_remove = he_right;
        he_right = mesh->HEGetFAPrev(he_right);
        mesh->FADeallocate(mesh->HEGetFA(he_right));
        mesh->HESetFA(he_right, kFAInvalid);

        mesh->VXSetValence(mesh->HEGetVX(he_remove),
                           mesh->VXGetValence(mesh->HEGetVX(he_remove)) - 1);
        if (mesh->VXGetHE(mesh->HEGetVX(he_remove)) == he_remove) {
          mesh->VXSetHE(mesh->HEGetVX(he_remove), mesh->HEGetHE(he_right));
        }

        mesh->Splice(mesh->HEGetVXPrev(he_remove), he_remove);

        he_right = mesh->HEGetFAPrev(he_right);
        mesh->HESetFA(he_right, kFAInvalid);

        he_remove = mesh->HEGetHE(he_remove);
        mesh->VXSetValence(mesh->HEGetVX(he_remove),
                           mesh->VXGetValence(mesh->HEGetVX(he_remove)) - 1);
        if (mesh->VXGetHE(mesh->HEGetVX(he_remove)) == he_remove) {
          mesh->VXSetHE(mesh->HEGetVX(he_remove), he_right);
        }

        mesh->Splice(he_right, he_remove);

        mesh->HEDeallocate(he_remove);
      }

      // Choose whether to connect to the left or right.
      const ScalarT ccw_left = this->preds_->CCWInscribedCircle_VVV(
          mesh->VXGetPoint(mesh->HEGetVX(he_left)),
          mesh->VXGetPoint(mesh->HEGetVX(he_base)),
          mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_base))));
      const ScalarT ccw_right = this->preds_->CCWInscribedCircle_VVV(
          mesh->VXGetPoint(mesh->HEGetVX(he_base)),
          mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_base))),
          mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_right))));
      if (ccw_left <= kZero && ccw_right <= kZero) {
        // The top convex hull edge is found.
        break;
      }

      ScalarT in_circle_left = -kOne;
      bool is_delaunay_left = false;
      ScalarT in_circle_right = -kOne;
      bool is_delaunay_right = false;
      if (ccw_right <= kZero) {
        in_circle_left = kOne;
        is_delaunay_left = true;
      } else if (ccw_left <= kZero) {
        in_circle_right = kOne;
        is_delaunay_right = true;
      } else {
        in_circle_left = this->preds_->InInscribedCircle_VVV_V(
            mesh->VXGetPoint(mesh->HEGetVX(he_left)),
            mesh->VXGetPoint(mesh->HEGetVX(he_base)),
            mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_base))),
            mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_right))));
        is_delaunay_left = (kZero <= in_circle_left);

        in_circle_right = this->preds_->InInscribedCircle_VVV_V(
            mesh->VXGetPoint(mesh->HEGetVX(he_base)),
            mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_base))),
            mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_right))),
            mesh->VXGetPoint(mesh->HEGetVX(he_left)));
        is_delaunay_right = (kZero <= in_circle_right);
      }

      const FAIndex fa_new = mesh->FAAllocate();
      mesh->FASetValence(fa_new, 3);
      mesh->FASetHE(fa_new, he_base);
      mesh->HESetFA(he_base, fa_new);
      if (!is_delaunay_right ||
          (is_delaunay_left && (kZero < in_circle_left && in_circle_right == kZero))) {
        // Connect to the left.
        CHECK(is_delaunay_left);
        mesh->HESetFA(he_left, fa_new);

        HEIndex he_new = mesh->HEAllocate();
        mesh->HESetFA(he_new, fa_new);
        mesh->HESetVX(he_new, mesh->HEGetVX(mesh->HEGetHE(he_base)));
        mesh->VXSetValence(mesh->HEGetVX(he_new), mesh->VXGetValence(mesh->HEGetVX(he_new)) + 1);

        mesh->Splice(mesh->HEGetFANext(he_base), he_new);
        
        he_new = mesh->HEGetHE(he_new);
        
        mesh->HESetVX(he_new, mesh->HEGetVX(he_left));
        mesh->VXSetValence(mesh->HEGetVX(he_new), mesh->VXGetValence(mesh->HEGetVX(he_new)) + 1);

        mesh->Splice(he_left, he_new);
        
        he_base = he_new;
        he_left = mesh->HEGetFAPrev(he_base);
      } else {
        // Connect to the right.
        CHECK(is_delaunay_right);
        mesh->HESetFA(he_right, fa_new);

        HEIndex he_new = mesh->HEAllocate();
        he_new = mesh->HEGetHE(he_new);
        mesh->HESetVX(he_new, mesh->HEGetVX(he_base));
        mesh->VXSetValence(mesh->HEGetVX(he_new), mesh->VXGetValence(mesh->HEGetVX(he_new)) + 1);

        mesh->Splice(he_base, he_new);

        he_base = he_new;
        he_new = mesh->HEGetHE(he_new);
        
        mesh->HESetFA(he_new, fa_new);
        mesh->HESetVX(he_new, mesh->HEGetVX(mesh->HEGetHE(he_right)));
        mesh->VXSetValence(mesh->HEGetVX(he_new), mesh->VXGetValence(mesh->HEGetVX(he_new)) + 1);

        mesh->Splice(mesh->HEGetFANext(he_right), he_new);
        
        he_right = mesh->HEGetFANext(he_base);
      }
    }

    return he_bottom;
  }

  // Find the left and right convex hull half edges to be connected by the bottom convex hull half
  // edge.
  // Parameters:
  //   mesh - Mesh with two the Delaunay triangulation submeshes.
  //   he_left - Bottom convex hull half edge from the left submesh.
  //   he_right - Bottom convex hull half edge from the right submesh.
  // Returns the bottom convex hull left and right half edges pair.
  ConvexHullBottom FindConvexHullBottom(
      const MeshGeometry<ScalarT, 2>& mesh, HEIndex he_left, HEIndex he_right) const {
    static constexpr ScalarT kZero(0);

    const PointComparator comparator;

    while (true) {
      // | L0  L2 R0  R2 |
      // | L\ /    \ /R  |
      // |   L1     R1   |
      const VXIndex left_vxs[3] = {
        mesh.HEGetVX(mesh.HEGetHE(he_left)),
        mesh.HEGetVX(he_left),
        mesh.HEGetVX(mesh.HEGetFAPrev(he_left))
      };
      const VXIndex right_vxs[3] = {
        mesh.HEGetVX(mesh.HEGetHE(mesh.HEGetFANext(he_right))),
        mesh.HEGetVX(mesh.HEGetFANext(he_right)),
        mesh.HEGetVX(he_right)
      };
      const ScalarT ccws[4] = {
        this->preds_->CCWInscribedCircle_VVV(
            mesh.VXGetPoint(right_vxs[1]),
            mesh.VXGetPoint(left_vxs[1]),
            mesh.VXGetPoint(left_vxs[0])),
        this->preds_->CCWInscribedCircle_VVV(
            mesh.VXGetPoint(right_vxs[1]),
            mesh.VXGetPoint(left_vxs[2]),
            mesh.VXGetPoint(left_vxs[1])),
        this->preds_->CCWInscribedCircle_VVV(
            mesh.VXGetPoint(right_vxs[1]),
            mesh.VXGetPoint(right_vxs[0]),
            mesh.VXGetPoint(left_vxs[1])),
        this->preds_->CCWInscribedCircle_VVV(
            mesh.VXGetPoint(right_vxs[2]),
            mesh.VXGetPoint(right_vxs[1]),
            mesh.VXGetPoint(left_vxs[1]))
      };
      
      if (kZero < ccws[0]) {
        // Move the left to the left.
        he_left = mesh.HEGetFANext(he_left);
      } else if (kZero < ccws[3]) {
        // Move the right to the right.
        he_right = mesh.HEGetFAPrev(he_right);
      } else if ((ccws[1] < kZero) ||
            (ccws[0] <= kZero && ccws[1] == kZero &&
             comparator(mesh.VXGetPoint(left_vxs[1]), mesh.VXGetPoint(left_vxs[2])))) {
        // Move the left to the right.
        he_left = mesh.HEGetFAPrev(he_left);
      } else if ((ccws[2] < kZero) ||
            (ccws[2] == kZero && ccws[3] <= kZero &&
             comparator(mesh.VXGetPoint(right_vxs[0]), mesh.VXGetPoint(right_vxs[1])))) {
        // Move the right to the left.
        he_right = mesh.HEGetFANext(he_right);
      } else {
        // Without collinear points:
        //   (ccws[0] <= kZero && kZero < ccws[1] && kZero < ccws[2] && ccws[3] <= kZero).
        CHECK_LE(ccws[0], kZero);
        CHECK_LE(kZero, ccws[1]);
        CHECK_LE(kZero, ccws[2]);
        CHECK_LE(ccws[3], kZero);

        // Found the bottom convex hull connector pair.
        break;
      }
    }

    return {.he_left = he_left, .he_right = he_right}; 
  }
  
  // Allocate a vertex and set its position.
  // Parameters:
  //   points - Input 2D point set.
  //   sorted_point_indices - Point indices sorted by x and then y coordinate.
  //   index - Index into the sorted point indices.
  //   mesh - Mesh that receives the Delaunay triangulation.
  //   index_for_vx - If not nullptr, receives a map from the output vx to the input indices.
  // Returns the allocated vertex index.
  VXIndex VXAllocate(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      const std::vector<int>& sorted_point_indices,
      int index,
      MeshGeometry<ScalarT, 2>* mesh,
      std::unordered_map<VXIndex, int>* index_for_vx) const {
    CHECK_NE(mesh, nullptr);

    const VXIndex vx = mesh->VXAllocate();
    const int point_index = sorted_point_indices[index];
    mesh->VXMutablePoint(vx) = points[point_index];
    if (index_for_vx != nullptr) {
      (*index_for_vx)[vx] = point_index;
    }
    return vx;
  }
  
  const geom::DelaunayPredicates<ScalarT>* preds_;
  std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned_;
};  // class DelaunayTriangulatorDivideAndConquer

}  // namespace internal

template<class ScalarT>
std::unique_ptr<DelaunayTriangulator<ScalarT>> DelaunayTriangulator<ScalarT>::New(
    const geom::DelaunayPredicates<ScalarT>* preds) {
  return DelaunayTriangulator<ScalarT>::NewDivideAndConquer(preds);
}

template<class ScalarT>
std::unique_ptr<DelaunayTriangulator<ScalarT>> DelaunayTriangulator<ScalarT>::New(
    std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned) {
  return DelaunayTriangulator<ScalarT>::NewDivideAndConquer(std::move(preds_owned));
}
 
template<class ScalarT>
std::unique_ptr<DelaunayTriangulator<ScalarT>> DelaunayTriangulator<ScalarT>::NewDivideAndConquer(
    const geom::DelaunayPredicates<ScalarT>* preds) {
  return std::make_unique<internal::DelaunayTriangulatorDivideAndConquer<ScalarT>>(preds, nullptr);
}

template<class ScalarT>
std::unique_ptr<DelaunayTriangulator<ScalarT>> DelaunayTriangulator<ScalarT>::NewDivideAndConquer(
    std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned) {
  return std::make_unique<internal::DelaunayTriangulatorDivideAndConquer<ScalarT>>(
      preds_owned.get(), std::move(preds_owned));
}
 
}  // namespace hemesh
