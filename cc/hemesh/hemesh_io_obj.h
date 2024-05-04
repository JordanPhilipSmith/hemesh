#pragma once

// Half edge mesh .obj file format IO.

#include <memory>
#include <string>
#include <unordered_map>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/math/vector.h"

namespace hemesh::io {

// Writes a half edge mesh to a .obj buffer.
// Parameters:
//   buffer - .obj buffer.
template<class ScalarT, int dim>
absl::StatusOr<std::string> ToObj(const MeshGeometry<ScalarT, dim>& mesh);
  
// Reads a half edge mesh from a .obj buffer.
// Parameters:
//   buffer - .obj buffer.
template<class ScalarT, int dim>
absl::StatusOr<std::unique_ptr<MeshGeometry<ScalarT, dim>>> FromObj(absl::string_view buffer);
  
// Implementation.

namespace internal {

constexpr char kObjComment = '#';
constexpr char kObjVertex[] = "v";
constexpr char kObjTextureCoordinates[] = "vt";
constexpr char kObjNormal[] = "n";
constexpr char kObjFacet[] = "f";

template<class ScalarT, int dim>
absl::StatusOr<std::string> ToObjImpl(const MeshGeometry<ScalarT, dim>& mesh) {
  static constexpr int kObjDim = 3;
  const int max_dim = std::min(kObjDim, dim);
  
  std::string buffer;

  // Vertices.
  std::unordered_map<VXIndex, int> index_for_vx;
  int index = 0;
  for (VXIndex vx : mesh.GetSortedVXIndices()) {
    ++index;

    index_for_vx[vx] = index;
    
    const math::ConstVector<ScalarT, dim>& point = mesh.VXGetPoint(vx);
    absl::StrAppend(&buffer, kObjVertex);

    int i = 0;
    for (; i < max_dim; ++i) {
      absl::StrAppendFormat(&buffer, " %.18g", point[i]);
    }
    for (; i < kObjDim; ++i) {
      absl::StrAppendFormat(&buffer, " 0");
    }
    absl::StrAppend(&buffer, "\n");
  }

  // Facets.
  for (FAIndex fa : mesh.GetSortedFAIndices()) {
    absl::StrAppend(&buffer, kObjFacet);

    const HEIndex he_begin = mesh.FAGetHE(fa);
    if (he_begin == kHEInvalid) {
      return absl::FailedPreconditionError(absl::StrCat("fa", fa, " : he_begin is invalid."));
    }

    HEIndex he = he_begin;
    do {
      if (he == kHEInvalid) {
	return absl::FailedPreconditionError(absl::StrCat("fa", fa, " : he is invalid."));
      }

      const VXIndex vx = mesh.HEGetVX(he);
      auto iter = index_for_vx.find(vx);
      if (iter == index_for_vx.cend()) {
	return absl::FailedPreconditionError(
	    absl::StrCat("fa", fa, " : Vertex not found vx", vx, "."));
      }
      absl::StrAppend(&buffer, " ", iter->second);
      
      he = mesh.HEGetFANext(he);
    } while (he != he_begin);

    absl::StrAppend(&buffer, "\n");
  }

  return buffer;
}

template<class ScalarT, int dim>
class ObjParser {
 public:
  struct EdgeKey {
    EdgeKey(VXIndex vx0, VXIndex vx1) {
      if (vx1 < vx0) {
        this->vxs[0] = vx1;
        this->vxs[1] = vx0;
      } else {
        this->vxs[0] = vx0;
        this->vxs[1] = vx1;
      }
    }

    // Equality operator for use with std::unordered_map.
    bool operator==(const EdgeKey& key) const {
      return this->vxs[0] == key.vxs[0] && this->vxs[1] == key.vxs[1];
    }

    VXIndex vxs[2];
  };  // struct EdgeKey

  struct EdgeKeyHasher {
    std::size_t operator()(const EdgeKey& key) const {
      // Assumes that the largest VXIndex fits in 32 bits.
      static constexpr int kBits = 32;
      static constexpr std::size_t kMask = 0xffffffff;
      const std::size_t min_vx = static_cast<std::size_t>(key.vxs[0]);
      const std::size_t max_vx = static_cast<std::size_t>(key.vxs[1]);
      return std::hash<std::size_t>()(((max_vx & kMask) << kBits) | (min_vx & kMask));
    }
  };  // struct EdgeKeyHasher

  ObjParser() = default;
  ~ObjParser() = default;
  
  absl::StatusOr<std::unique_ptr<MeshGeometry<ScalarT, dim>>> FromObj(
      absl::string_view buffer) const {
    std::unique_ptr<MeshGeometry<ScalarT, dim>> mesh =
        std::make_unique<MeshGeometry<ScalarT, dim>>();
  
    const std::vector<std::string> lines = absl::StrSplit(buffer, "\n");

    std::unordered_map<int, VXIndex> vx_for_index;
    std::unordered_map<EdgeKey, HEIndex, EdgeKeyHasher> he_for_key;
  
    int line_index = 0;
    int vertex_index = 0;
    for (const std::string& line : lines) {
      ++line_index;

      if (line.empty()) {
        // Skip an empty line.
        continue;
      }
      if (line[0] == kObjComment) {
        // Skip a commented line.
        continue;
      }

      const std::vector<std::string> tokens = absl::StrSplit(line, " ");

      const int tokens_size = static_cast<int>(tokens.size());
      if (tokens_size < 1) {
        // Skip a line with no token.
        continue;
      }
      
      int token_index = 0;
      if (tokens[token_index] == kObjVertex) {
        // Vertex.
        ++token_index;
        math::VectorOwned<ScalarT, dim> point;
        point.setZero();
        for (int i = 0; i < dim && token_index < tokens_size; ++i, ++token_index) {
          double value = 0.0;
          if (!absl::SimpleAtod(tokens[token_index], &value)) {
            return absl::FailedPreconditionError(absl::StrCat(
                "line ", line_index, " : Invalid vertex coordinate \"", tokens[token_index],
                "\"."));
          }
          point[i] = value;
        }

        const VXIndex vx = mesh->VXAllocate();
        mesh->VXMutablePoint(vx) = point;

        ++vertex_index;
        vx_for_index[vertex_index] = vx;
      } else if (tokens[token_index] == kObjTextureCoordinates) {
        // Texture coordinates.
        // TODO: Handle texture coordinates.
      } else if (tokens[token_index] == kObjNormal) {
        // Normal.
        // TODO: Handle normals.
      } else if (tokens[token_index] == kObjFacet) {
        // Facet.
        std::vector<int> vertex_indices;
        for (++token_index; token_index < tokens_size; ++token_index) {
          const std::vector<std::string> corner_tokens = absl::StrSplit(tokens[token_index], "/");
          int corner_tokens_size = static_cast<int>(corner_tokens.size());
          if (corner_tokens_size < 1) {
            return absl::FailedPreconditionError(absl::StrCat(
                "line ", line_index, " : Invalid facet corner ", token_index, " = \"",
                tokens[token_index], "\"."));
          }

          // vertex/texture/normal

          int vertex_index = 0;
          if (!absl::SimpleAtoi(corner_tokens[0], &vertex_index)) {
            return absl::FailedPreconditionError(absl::StrCat(
                "line ", line_index, " : Invalid facet vertex index[", token_index, "] = \"",
                corner_tokens[0], "\"."));
          }
          vertex_indices.push_back(vertex_index);

          // TODO: Handle texture coordinates and normals.
        }

        const int valence = static_cast<int>(vertex_indices.size());
        if (valence < 3) {
            return absl::FailedPreconditionError(absl::StrCat(
                "line ", line_index, " : Facet with vertices = ", valence, " < 3."));
        }
        const FAIndex fa = mesh->FAAllocate();
        mesh->FASetValence(fa, valence);
        
        VXIndex vx_prev = kVXInvalid;
        HEIndex he_begin = kHEInvalid;
        HEIndex he_prev = kHEInvalid;
        for (int vertex_index : vertex_indices) {
          VXIndex vx = kVXInvalid;
          {
            auto iter = vx_for_index.find(vertex_index);
            if (iter == vx_for_index.cend()) {
              return absl::FailedPreconditionError(absl::StrCat(
                  "line ", line_index, " : Facet with invalid vertex index = ", vertex_index, "."));
            }
            vx = iter->second;
          }
          CHECK_NE(vx, kVXInvalid);
          
          if (vx_prev != kVXInvalid) {
            // Find the half edge.
            HEIndex he = kHEInvalid;
            const EdgeKey key(vx_prev, vx);
            auto iter = he_for_key.find(key);
            if (iter == he_for_key.cend()) {
              // Create a new half edge.
              he = mesh->HEAllocate();
              he_for_key[key] = he;

              if (mesh->VXGetHE(vx_prev) == kHEInvalid) {
                mesh->VXSetHE(vx_prev, he);
              }
              mesh->VXSetValence(vx_prev, mesh->VXGetValence(vx_prev) + 1);
              mesh->HESetVX(he, vx_prev);

              mesh->VXSetValence(vx, mesh->VXGetValence(vx) + 1);
              mesh->HESetVX(mesh->HEGetHE(he), vx);
            } else {
              // Use the opposite half edge side of the shared edge.
              he = iter->second;
              CHECK_NE(he, kHEInvalid);
              if (mesh->HEGetVX(he) != vx_prev) {
                he = mesh->HEGetHE(he);
                CHECK_EQ(mesh->HEGetVX(he), vx_prev);
              }
              CHECK_EQ(mesh->HEGetVX(mesh->HEGetHE(he)), vx);
              if (mesh->HEGetFA(he) != kFAInvalid) {
                return absl::FailedPreconditionError(absl::StrCat(
                    "line ", line_index, " : Facet has a non-2-manifold edge vx", vx_prev, " => vx",
                    vx, " is already used by fa", mesh->HEGetFA(he), "."));
              }
            }
            CHECK_NE(he, kHEInvalid);

            mesh->HESetFA(he, fa);
            if (he_begin == kHEInvalid) {
              // First edge of the facet.
              he_begin = he;
              mesh->FASetHE(fa, he);
            } else {
              CHECK_NE(he_prev, kHEInvalid);
              if (mesh->HEGetFANext(he_prev) != he) {
                mesh->Splice(mesh->HEGetFANext(he_prev), he);
              }
            }

            he_prev = he;
          }

          vx_prev = vx;
        }
        CHECK_NE(he_prev, kHEInvalid);
        CHECK_NE(he_begin, kHEInvalid);

        // Create the final edge.
        //LOG(INFO) << "Create the final edge.";
        //LOG(INFO) << mesh->Display();
        //LOG(INFO) << "he_prev = he" << he_prev;
        vx_prev = mesh->HEGetVX(mesh->HEGetHE(he_prev));
        //LOG(INFO) << "vx_prev = vx" << vx_prev;
        //LOG(INFO) << "he_begin = he" << he_begin;
        const VXIndex vx = mesh->HEGetVX(he_begin);
        //LOG(INFO) << "vx = vx" << vx;
        // Find the half edge.
        HEIndex he = kHEInvalid;
        const EdgeKey key(vx_prev, vx);
        auto iter = he_for_key.find(key);
        if (iter == he_for_key.cend()) {
          // Create a new half edge.
          he = mesh->HEAllocate();
          he_for_key[key] = he;
          
          if (mesh->VXGetHE(vx_prev) == kHEInvalid) {
            mesh->VXSetHE(vx_prev, he);
          }
          mesh->VXSetValence(vx_prev, mesh->VXGetValence(vx_prev) + 1);
          mesh->HESetVX(he, vx_prev);
          
          mesh->VXSetValence(vx, mesh->VXGetValence(vx) + 1);
          mesh->HESetVX(mesh->HEGetHE(he), vx);
        } else {
          // Use the opposite half edge side of the shared edge.
          he = iter->second;
          CHECK_NE(he, kHEInvalid);
          if (mesh->HEGetVX(he) != vx_prev) {
            he = mesh->HEGetHE(he);
            CHECK_EQ(mesh->HEGetVX(he), vx_prev);
          }
          CHECK_EQ(mesh->HEGetVX(mesh->HEGetHE(he)), vx);
          if (mesh->HEGetFA(he) != kFAInvalid) {
            return absl::FailedPreconditionError(absl::StrCat(
                "line ", line_index, " : Facet has a non-2-manifold edge vx", vx_prev, " => vx",
                vx, " is already used by fa", mesh->HEGetFA(he), "."));
          }
        }
        CHECK_NE(he, kHEInvalid);

        mesh->HESetFA(he, fa);
        if (mesh->HEGetFANext(he_prev) != he) {
          mesh->Splice(mesh->HEGetFANext(he_prev), he);
        }
        if (mesh->HEGetFANext(he) != he_begin) {
          mesh->Splice(mesh->HEGetFANext(he), he_begin);
        }
      } else {
        // Unknown token.
        return absl::InternalError(absl::StrCat(
            "line ", line_index, " : Unknown token \"", tokens[0], "\"."));
      }
    }

    // Mark the boundary half edges.
    for (HEIndex he : mesh->GetHEIndices()) {
      if (mesh->HEGetFA(he) == kFAInvalid) {
        mesh->HESetIsBoundary(he, true);
      }
    }
    
    return mesh;
  }
};  // class ObjParser
 
}  // namespace internal

template<class ScalarT, int dim>
absl::StatusOr<std::string> ToObj(const MeshGeometry<ScalarT, dim>& mesh) {
  return internal::ToObjImpl<ScalarT, dim>(mesh);
}
  
template<class ScalarT, int dim>
absl::StatusOr<std::unique_ptr<MeshGeometry<ScalarT, dim>>> FromObj(absl::string_view buffer) {
  internal::ObjParser<ScalarT, dim> parser;
  return parser.FromObj(buffer);
}

}  // namespace hemesh::io

