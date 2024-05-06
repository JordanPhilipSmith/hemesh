// Generates the convex hull of a set of 2D points.
// Usage:
//   BAZEL_ROOT=~/hemesh
//   cd ${BAZEL_ROOT}; bazel run //tools:convex_hull_2d_cli -- \
//     --input_points_obj="${BAZEL_ROOT}/data/obj/random_1000_points.obj" \
//     --output_convex_hull_obj="${BAZEL_ROOT}/data/svg/my_random_1000_convex_hull.obj"

#include <memory>
#include <string>
#include <vector>

#include "absl/flags/commandlineflag.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "cc/file/file_helpers.h"
#include "cc/geom/delaunay_predicates.h"
#include "cc/hemesh/convex_hull_2d.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/hemesh/hemesh_io_obj.h"
#include "cc/math/vector2.h"
#include "cc/status/status_macros.h"

ABSL_FLAG(std::string, input_points_obj, "", "Input .obj file with vertex points only.");
ABSL_FLAG(std::string, output_convex_hull_obj, "", "Output 2D convex hull .obj file.");

namespace hemesh {
namespace {

absl::Status Main(absl::string_view input_points_obj_filename,
                  absl::string_view output_convex_hull_obj_filename) {
  LOG(INFO) << "--input_points_obj = \"" << input_points_obj_filename << "\"";
  LOG(INFO) << "--output_convex_hull_obj = \"" << output_convex_hull_obj_filename << "\"";
  
  if (input_points_obj_filename.empty()) {
    return absl::InvalidArgumentError("--input_points_obj is empty.");
  }
  if (output_convex_hull_obj_filename.empty()) {
    return absl::InvalidArgumentError("--output_convex_hull_obj is empty.");
  }

  LOG(INFO) << "BEGIN: Read points from .obj file.";  
  std::string input_buffer;
  RETURN_IF_ERROR(file::GetContents(input_points_obj_filename, &input_buffer));
  LOG(INFO) << "  BEGIN: Parse points from .obj file.";  
  using MeshGeometryT = MeshGeometry<double, 2>;
  ASSIGN_OR_RETURN(std::unique_ptr<MeshGeometryT> points_mesh,
                   (hemesh::io::FromObj<double, 2>(input_buffer)));
  LOG(INFO) << "  END  : Parse points from .obj file.";  
  LOG(INFO) << "END  : Read points from .obj file.";  

  // Convert the points.
  std::vector<math::VectorOwned<double, 2>> points;
  for (VXIndex vx : points_mesh->GetSortedVXIndices()) {
    points.push_back(points_mesh->VXGetPoint(vx));
  }

  // Create the 2D convex hull.
  std::unique_ptr<ConvexHull2D<double>> convex_hull_2d_creator =
      ConvexHull2D<double>::New(geom::DelaunayPredicates<double>::NewLocalOrigin());

  std::unordered_map<VXIndex, int>* index_for_vx = nullptr;
  std::unique_ptr<MeshGeometry<double, 2>> mesh =
      convex_hull_2d_creator->CreateConvexHull(points, index_for_vx);

  LOG(INFO) << "BEGIN: Write 2D convex hull to .obj file.";  
  ASSIGN_OR_RETURN(const std::string output_buffer, (hemesh::io::ToObj<double, 2>(*mesh)));
  absl::Status status = file::SetContents(output_convex_hull_obj_filename, output_buffer);
  LOG(INFO) << "END  : Write 2D convex hull to .obj file.";  

  return status;
}

}  // namespace
}  // namespace hemesh

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const absl::Status status = hemesh::Main(
      absl::GetFlag(FLAGS_input_points_obj), absl::GetFlag(FLAGS_output_convex_hull_obj));
  if (!status.ok()) {
    LOG(ERROR) << "status = " << status;
    return 1;
  }
  return 0;
}
