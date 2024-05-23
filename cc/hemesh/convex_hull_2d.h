#pragma once

// Calculates the convex hull of a set of 2D points.

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include "absl/log/check.h"
#include "cc/geom/delaunay_predicates.h"
#include "cc/hemesh/hemesh_geometry.h"

namespace hemesh {

// Calculates the convex hull of a set of 2D points.
template<class ScalarT>
class ConvexHull2D {
 public:
  virtual ~ConvexHull2D() = default;

  // Factory method.
  // Parameters:
  //   preds - Delaunay predicates must not be nullptr.
  static std::unique_ptr<ConvexHull2D> New(const geom::DelaunayPredicates<ScalarT>* preds);
  static std::unique_ptr<ConvexHull2D> New(
      std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds);
  
  // Creates the convex hull of a set of 2D points.
  // Parameters:
  //   points - Set of points.
  //   keep_collinear_points - Always remove coincident points.  If true, include points that are
  //     on the interior of collinear edges.  If false, only include the endpoints of convex hull
  //     edges.
  //   opt_index_for_vx - If not nullptr, receives a map from the output vx to the input indices.
  // Returns a convex hull mesh.
  virtual std::unique_ptr<MeshGeometry<ScalarT, 2>> CreateConvexHull(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      bool keep_collinear_points,
      std::unordered_map<VXIndex, int>* opt_index_for_vx) const = 0;
};  // class ConvexHull2D

// Implementation.
 
namespace internal {

// ConvexHull2D implementation.
template<class ScalarT>
class ConvexHull2DImpl : public ConvexHull2D<ScalarT> {
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
      return this->comparator_(a, b);
    }

    const std::vector<math::VectorOwned<ScalarT, 2>>* points;
    const PointComparator comparator_;
  };  // struct PointIndexComparator

  // Constructor.
  // Parameters:
  //   preds - Delaunay predicates must not be nullptr.
  //   opt_preds_owned - If not nullptr, owned pointer to the same Delaunay predicates.
  ConvexHull2DImpl(
      const geom::DelaunayPredicates<ScalarT>* preds,
      std::unique_ptr<geom::DelaunayPredicates<ScalarT>> opt_preds_owned)
    : preds_(preds), preds_owned_(std::move(opt_preds_owned)) {
    CHECK_NE(this->preds_, nullptr);
  }

  virtual ~ConvexHull2DImpl() = default;
  
  std::unique_ptr<MeshGeometry<ScalarT, 2>> CreateConvexHull(
      const std::vector<math::VectorOwned<ScalarT, 2>>& points,
      bool keep_collinear_points,
      std::unordered_map<VXIndex, int>* opt_index_for_vx) const override {
    static constexpr ScalarT kZero(0);

    std::unordered_map<VXIndex, int>* index_for_vx = opt_index_for_vx;

    std::unique_ptr<MeshGeometry<ScalarT, 2>> mesh = std::make_unique<MeshGeometry<ScalarT, 2>>();

    const int size = static_cast<int>(points.size());
    if (size < 1) {
      // Empty mesh.
      return mesh;
    }

    // Sort the points by x and then y coordinate.
    std::vector<int> sorted_point_indices(size);
    for (int i = 0; i < size; ++i) {
      sorted_point_indices[i] = i;
    }    
    const PointComparator point_comparator;
    const PointIndexComparator comparator(&points);
    std::sort(sorted_point_indices.begin(), sorted_point_indices.end(), comparator);
    
    // Create the first vertex.
    int index = 0;
    CHECK_LT(index, size);
    int vertex_index = 0;
    int point_indices[3];
    VXIndex vxs[3];
    vxs[vertex_index] = mesh->VXAllocate();
    mesh->VXSetValence(vxs[vertex_index], 0);
    point_indices[vertex_index] = sorted_point_indices[index];
    mesh->VXMutablePoint(vxs[vertex_index]) = points[point_indices[vertex_index]];
    if (index_for_vx != nullptr) {
      (*index_for_vx)[vxs[vertex_index]] = point_indices[vertex_index];
    }
    // Skip duplicate vertices.
    for (++index; index < size; ++index) {
      if (comparator(point_indices[vertex_index], sorted_point_indices[index])) {
        // A non-coincident point is found.
        break;
      }
    }

    if (size <= index) {
      // All points are coincident, so there is only one vertex in the convex hull.
      return mesh;
    }

    // Create an edge.
    mesh->VXSetValence(vxs[vertex_index], 1);

    ++vertex_index;
    point_indices[vertex_index] = sorted_point_indices[index];
    if (keep_collinear_points) {
      // Skip duplicate vertices.
      for (++index; index < size; ++index) {
        if (comparator(point_indices[vertex_index], sorted_point_indices[index])) {
          // A non-coincident point is found.
          break;
        }
        // The points are coincident, so skip the previous point.
        point_indices[vertex_index] = sorted_point_indices[index];
      }
    } else {
      // Skip collinear points.
      for (++index; index < size; ++index) {
        if (this->preds_->CCWInscribedCircle_VVV(
                points[point_indices[0]],
                points[point_indices[vertex_index]],
                points[sorted_point_indices[index]]) != kZero) {
          // A non-collinear point is found.
          break;
        }
        // The points are collinear, so skip the previous point.
        point_indices[vertex_index] = sorted_point_indices[index];
      }
    }

    // Create the second vertex.
    vxs[vertex_index] = mesh->VXAllocate();
    mesh->VXSetValence(vxs[vertex_index], 1);
    mesh->VXMutablePoint(vxs[vertex_index]) = points[point_indices[vertex_index]];
    if (index_for_vx != nullptr) {
      (*index_for_vx)[vxs[vertex_index]] = point_indices[vertex_index];
    }

    HEIndex he = mesh->HEAllocate();
    HEIndex hes[2] = {he, mesh->HEGetHE(he)};
    for (int i = 0; i < 2; ++i) {
      mesh->VXSetHE(vxs[i], hes[i]);
      mesh->HESetVX(hes[i], vxs[i]);
    }

    if (size <= index) {
      // All points are collinear, so there are only two vertices in the convex hull.
      for (int i = 0; i < 2; ++i) {
        mesh->HESetIsBoundary(hes[i], true);
      }
      return mesh;
    }

    // Create the first triangle or collinear polyline.
    ++vertex_index;
    point_indices[vertex_index] = sorted_point_indices[index];
    
    vxs[vertex_index] = mesh->VXAllocate();
    mesh->VXMutablePoint(vxs[vertex_index]) = points[point_indices[vertex_index]];
    if (index_for_vx != nullptr) {
      (*index_for_vx)[vxs[vertex_index]] = point_indices[vertex_index];
    }
    
    FAIndex fa = kFAInvalid;
    HEIndex he_right = kHEInvalid;
    const ScalarT ccw = this->preds_->CCWInscribedCircle_VVV(
        points[point_indices[0]],
        points[point_indices[1]],
        points[point_indices[2]]);
    if (keep_collinear_points && ccw == kZero) {
      // Create a collinear polyline.

      mesh->VXSetValence(vxs[1], 2);
      mesh->VXSetValence(vxs[2], 1);
      
      // Create the second edge.
      HEIndex he_prev = he;
      he = mesh->HEAllocate();
      mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
      mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

      mesh->VXSetHE(vxs[2], mesh->HEGetHE(he));
      mesh->HESetVX(mesh->HEGetHE(he), vxs[2]);
    
      mesh->Splice(mesh->HEGetHE(he_prev), he);

      he_right = mesh->HEGetHE(he);
    } else {
      // Create a triangle.
      mesh->VXSetValence(vxs[0], 2);
      mesh->VXSetValence(vxs[1], 2);
      mesh->VXSetValence(vxs[2], 2);

      if (ccw < kZero) {
        // Clockwise, so swap the first edge.
        he = mesh->HEGetHE(he);
      }

      fa = mesh->FAAllocate();
      mesh->FASetValence(fa, 3);
      mesh->FASetHE(fa, he);
      mesh->HESetFA(he, fa);

      // Create the second edge.
      HEIndex he_begin = he;
      HEIndex he_prev = he;
      he = mesh->HEAllocate();
      mesh->HESetFA(he, fa);
      mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
      mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

      mesh->HESetVX(mesh->HEGetHE(he), vxs[2]);
    
      mesh->Splice(mesh->HEGetHE(he_prev), he);

      he_prev = he;

      // Create the third edge.
      he = mesh->HEAllocate();
      mesh->HESetFA(he, fa);
      mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
      mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

      mesh->HESetVX(mesh->HEGetHE(he), mesh->HEGetVX(he_begin));
    
      mesh->Splice(mesh->HEGetHE(he_prev), he);
      mesh->Splice(mesh->HEGetHE(he), he_begin);

      he_right = mesh->HEGetHE(he_prev);
    }
    CHECK_NE(he_right, kHEInvalid);

    // Insert the rest of the points.
    for (++index; index < size; ++index) {
      // Find the top silhouette vertex.
      HEIndex he_top = mesh->HEGetFAPrev(he_right);
      ScalarT ccw_top(0);
      int point_index = -1;
      if (keep_collinear_points) {
        // Skip duplicate vertices.
        for (; index < size; ++index) {
          if (point_comparator(mesh->VXGetPoint(mesh->HEGetVX(he_right)),
                               points[sorted_point_indices[index]])) {
            // A non-coincident point is found.
            break;
          }
          // Skip the coincident point.
        }
        if (size <= index) {
          break;
        }
        point_index = sorted_point_indices[index];
        for (; he_top != he_right; he_top = mesh->HEGetFAPrev(he_top)) {
          ccw_top = this->preds_->CCWInscribedCircle_VVV(
              mesh->VXGetPoint(mesh->HEGetVX(he_top)),
              mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_top))),
              points[point_index]);
          if (ccw_top <= kZero) {
            // Found the silhouette vertex.
            break;
          }
        }
        if (fa == kFAInvalid) {
          if (ccw_top == kZero) {
            // Extend the collinear polyline.
            const VXIndex vx = mesh->VXAllocate();
            mesh->VXSetValence(vx, 1);
            mesh->VXMutablePoint(vx) = points[point_index];
            if (index_for_vx != nullptr) {
              (*index_for_vx)[vx] = point_index;
            }
            HEIndex he_prev = he_top;
            he = mesh->HEAllocate();
            mesh->VXSetValence(mesh->HEGetVX(mesh->HEGetHE(he_prev)), 2);
            mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
            mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

            mesh->VXSetHE(vx, mesh->HEGetHE(he));
            mesh->HESetVX(mesh->HEGetHE(he), vx);
    
            mesh->Splice(mesh->HEGetHE(he_prev), he);

            he_right = mesh->HEGetHE(he);
          } else {
            // A non-collinear point is found, so allocate a facet.
            fa = mesh->FAAllocate();
            mesh->FASetHE(fa, mesh->HEGetHE(he_top));
            HEIndex he_begin = kHEInvalid;
            HEIndex he = he_top;
            int valence = 0;
            do {
              ++valence;
              mesh->HESetFA(mesh->HEGetHE(he), fa);
              if (he != mesh->HEGetFAPrev(he)) {
                he_begin = he;
              }

              he = mesh->HEGetFAPrev(he);
            } while (he != he_top);
            CHECK_NE(he_begin, kHEInvalid);

            valence += 2;
            mesh->FASetValence(fa, valence);

            // Add the new vertex and two new edges.
            const VXIndex vx = mesh->VXAllocate();
            mesh->VXSetValence(vx, 2);
            mesh->VXMutablePoint(vx) = points[point_index];
            if (index_for_vx != nullptr) {
              (*index_for_vx)[vx] = point_index;
            }
            HEIndex he_prev = mesh->HEGetHE(he_begin);
            he = mesh->HEAllocate();
            mesh->HESetFA(he, fa);
            mesh->VXSetValence(mesh->HEGetVX(mesh->HEGetHE(he_prev)), 2);
            mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
            mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

            mesh->HESetVX(mesh->HEGetHE(he), vx);
    
            mesh->Splice(mesh->HEGetHE(he_prev), he);

            he_prev = he;

            he = mesh->HEAllocate();
            mesh->HESetFA(he, fa);
            mesh->VXSetValence(mesh->HEGetVX(mesh->HEGetHE(he_top)), 2);
            mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_prev)), he);
            mesh->HESetVX(he, mesh->HEGetVX(mesh->HEGetHE(he_prev)));

            mesh->VXSetHE(mesh->HEGetVX(mesh->HEGetHE(he_top)), mesh->HEGetHE(he_top));
            mesh->HESetVX(mesh->HEGetHE(he), mesh->HEGetVX(mesh->HEGetHE(he_top)));
    
            mesh->Splice(mesh->HEGetHE(he_prev), he);
            mesh->Splice(mesh->HEGetHE(he_top), mesh->HEGetHE(he));

            if (point_comparator(mesh->VXGetPoint(mesh->HEGetVX(he_begin)),
                                 mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_top))))) {
              he_right = mesh->HEGetHE(he);
            } else {
              he_right = mesh->HEGetFANext(mesh->HEGetVXPrev(he));
            }
          }
          // The convex hull has been updated, and he_right has been moved.
          continue;
        }
      } else {
        // Remove collinear points.
        point_index = sorted_point_indices[index];
        for (; he_top != he_right; he_top = mesh->HEGetFAPrev(he_top)) {
          ccw_top = this->preds_->CCWInscribedCircle_VVV(
              mesh->VXGetPoint(mesh->HEGetVX(he_top)),
              mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_top))),
              points[point_index]);
          if (ccw_top <= kZero) {
            // Found the silhouette vertex.
            break;
          }
        }
      }
      CHECK_NE(fa, kFAInvalid);
      CHECK_NE(he_top, he_right);
      CHECK_LE(0, point_index);

      // Find the bottom silhouette vertex.
      HEIndex he_bottom = he_right;
      ScalarT ccw_bottom(1);
      do {
        ccw_bottom = this->preds_->CCWInscribedCircle_VVV(
            mesh->VXGetPoint(mesh->HEGetVX(he_bottom)),
            mesh->VXGetPoint(mesh->HEGetVX(mesh->HEGetHE(he_bottom))),
            points[point_index]);
        if (ccw_bottom <= kZero) {
          // Found the silhouette vertex.
          break;
        }

        he_bottom = mesh->HEGetFANext(he_bottom);
      } while (he_bottom != he_right);
      CHECK_LE(ccw_bottom, kZero);

      // Attach the new vertex.
      if (!keep_collinear_points) {
        if (ccw_top == kZero && ccw_bottom == kZero) {
          // The point is coincident to an existing vertex, so skip it.
          CHECK_EQ(he_bottom, he_right);
          CHECK_EQ(mesh->HEGetFANext(he_top), he_bottom);
          continue;
        }
      }

      CHECK_NE(mesh->HEGetFANext(he_top), he_bottom);

      HEIndex he_begin = he_right;
      if (!keep_collinear_points && ccw_top == kZero) {
        // Replace the top collinear vertex.
        const VXIndex vx = mesh->HEGetVX(mesh->HEGetHE(he_top));
        mesh->VXMutablePoint(vx) = points[point_index];
        if (index_for_vx != nullptr) {
          (*index_for_vx)[vx] = point_index;
        }

        he_right = mesh->HEGetFANext(he_top);

        he_begin = he_right;
        HEIndex he = mesh->HEGetFANext(he_begin);
        if (he == he_bottom) {
          // Only one vertex needed to move.
          continue;
        }
        this->RemoveHiddenVerticesAndEdges(fa, he, he_bottom, mesh.get(), index_for_vx);
      } else if (!keep_collinear_points && ccw_bottom == kZero) {
        // Replace the bottom collinear vertex.
        const VXIndex vx = mesh->HEGetVX(he_bottom);
        mesh->VXMutablePoint(vx) = points[point_index];
        if (index_for_vx != nullptr) {
          (*index_for_vx)[vx] = point_index;
        }

        he_right = he_bottom;
      
        he_begin = mesh->HEGetFANext(he_top);
        HEIndex he = mesh->HEGetFANext(he_begin);
        if (he == he_bottom) {
          // Only one vertex needed to move.
          continue;
        }
        this->RemoveHiddenVerticesAndEdges(fa, he, he_bottom, mesh.get(), index_for_vx);
      } else {
        // Neither top nor bottom is collinear (or keep collinear points).
        HEIndex he = mesh->HEGetFANext(he_top);
        if (mesh->HEGetFANext(he) == he_bottom) {
          // Add a new vertex and edge.
          const VXIndex vx = mesh->VXAllocate();
          mesh->VXSetValence(vx, 2);
          mesh->VXMutablePoint(vx) = points[point_index];
          if (index_for_vx != nullptr) {
            (*index_for_vx)[vx] = point_index;
          }

          mesh->Splice(mesh->HEGetHE(he), mesh->HEGetFANext(he));

          mesh->VXSetHE(vx, mesh->HEGetHE(he));
          mesh->HESetVX(mesh->HEGetHE(he), vx);

          he_right = mesh->HEAllocate();
          mesh->HESetFA(mesh->HEGetHE(he_right), fa);
          mesh->HESetVX(he_right, vx);

          mesh->Splice(mesh->HEGetHE(he), he_right);

          he_begin = he_right;
        } else {
          // Reuse a vertex.
          he = mesh->HEGetFANext(he);
          he_right = he;
          he_begin = he_right;

          const VXIndex vx = mesh->HEGetVX(he);
          mesh->VXMutablePoint(vx) = points[point_index];
          if (index_for_vx != nullptr) {
            (*index_for_vx)[vx] = point_index;
          }

          he = mesh->HEGetFANext(he);
          if (he == he_bottom) {
            // Only one vertex needed to move.
            continue;
          }
          this->RemoveHiddenVerticesAndEdges(fa, he, he_bottom, mesh.get(), index_for_vx);
        }
      }
      
      this->ReconnectContour(he_begin, he_bottom, mesh.get());
    }

    // Mark the boundary half edges and facet valence.
    int valence = 0;
    he = he_right;
    do {
      mesh->HESetIsBoundary(he, true);
      ++valence;

      he = mesh->HEGetFANext(he);
    } while (he != he_right);
    if (fa != kFAInvalid) {
      mesh->FASetValence(fa, valence);
    }
    return mesh;
  }

 private:
  // Removes hidden vertices and edges. All half edges are on the outside pointing to kFAInvalid.
  // Parameters:
  //   fa - Convex hull facet.
  //   he - First half edge (and origin vertex) to remove.
  //   he_bottom - Bottom silhouette half edge that is not removed.
  //   mesh - Modified convex hull mesh must not be nullptr.
  //   opt_index_for_vx - If not nullptr, receives a map from the output vx to the input indices.
  void RemoveHiddenVerticesAndEdges(FAIndex fa, HEIndex he, HEIndex he_bottom, 
                                    MeshGeometry<ScalarT, 2>* mesh,
                                    std::unordered_map<VXIndex, int>* index_for_vx) const {
    CHECK_NE(mesh, nullptr);

    // Remove hidden edges and vertices.
    while (he != he_bottom) {
      const HEIndex he_remove = he;
      he = mesh->HEGetFANext(he);

      if (mesh->FAGetHE(fa) == mesh->HEGetHE(he_remove)) {
        mesh->FASetHE(fa, mesh->HEGetHE(he_bottom));
      }
      mesh->Splice(he_remove, mesh->HEGetVXPrev(he_remove));
      mesh->Splice(mesh->HEGetHE(he_remove), mesh->HEGetFANext(he_remove));
      const VXIndex vx_remove = mesh->HEGetVX(he_remove);
      if (index_for_vx != nullptr) {
        CHECK_EQ(index_for_vx->erase(vx_remove), 1);
      }
      mesh->VXDeallocate(vx_remove);
      mesh->HEDeallocate(he_remove);
    }
  }

  // Removes hidden vertices and edges. All half edges are on the outside pointing to kFAInvalid.
  // Parameters:
  //   he_begin - Half edge to connect to he_bottom after removing hidden edges.
  //   he_bottom - Bottom silhouette half edge that is not removed.
  //   mesh - Modified convex hull mesh must not be nullptr.
  void ReconnectContour(HEIndex he_begin, HEIndex he_bottom, MeshGeometry<ScalarT, 2>* mesh) const {
    CHECK_NE(mesh, nullptr);

    // Reconnect the contour.
    mesh->VXSetHE(mesh->HEGetVX(he_bottom), mesh->HEGetHE(he_begin));
    mesh->HESetVX(mesh->HEGetHE(he_begin), mesh->HEGetVX(he_bottom));
    mesh->Splice(he_bottom, mesh->HEGetHE(he_begin));
  }
        
  const geom::DelaunayPredicates<ScalarT>* preds_;
  std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds_owned_;
};  // class ConvexHull2DImpl

}  // namespace internal

template<class ScalarT>
std::unique_ptr<ConvexHull2D<ScalarT>> ConvexHull2D<ScalarT>::New(
    const geom::DelaunayPredicates<ScalarT>* preds) {
  return std::make_unique<internal::ConvexHull2DImpl<ScalarT>>(preds, nullptr);
}

template<class ScalarT>
std::unique_ptr<ConvexHull2D<ScalarT>> ConvexHull2D<ScalarT>::New(
    std::unique_ptr<geom::DelaunayPredicates<ScalarT>> preds) {
  return std::make_unique<internal::ConvexHull2DImpl<ScalarT>>(preds.get(), std::move(preds));
}

}  // namespace hemesh
