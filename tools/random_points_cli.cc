// Generates a set of random points in a .obj file.
// Usage:
//   BAZEL_ROOT=~/hemesh
//   cd ${BAZEL_ROOT}; bazel run //tools:random_points_cli -- \
//     --input_points_size=100 \
//     --input_x_min=0.0 \
//     --input_x_max=1.0 \
//     --input_y_min=0.0 \
//     --input_y_max=1.0 \
//     --input_random_seed=1 \
//     --output_obj="${BAZEL_ROOT}/data/obj/random_100_points.obj"

#include <cstdlib>
#include <string>

#include "absl/flags/commandlineflag.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "cc/file/file_helpers.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/hemesh/hemesh_io_obj.h"
#include "cc/math/vector2.h"
#include "cc/status/status_macros.h"

ABSL_FLAG(int, input_points_size, 10, "Number of points to generate.");
ABSL_FLAG(double, input_x_min, 0.0, "Bounding box x_min.");
ABSL_FLAG(double, input_x_max, 1.0, "Bounding box x_max.");
ABSL_FLAG(double, input_y_min, 0.0, "Bounding box y_min.");
ABSL_FLAG(double, input_y_max, 1.0, "Bounding box y_max.");
ABSL_FLAG(std::uint32_t, input_random_seed, 1, "Random seed for srand().");
ABSL_FLAG(std::string, output_points_obj, "", "Output 2D random points .obj file.");

namespace hemesh {
namespace {

absl::Status Main(int input_points_size,
                  double input_x_min,
                  double input_x_max,
                  double input_y_min,
                  double input_y_max,
                  std::uint32_t input_random_seed,
                  absl::string_view output_points_obj_filename) {
  LOG(INFO) << "--input_points_size = " << input_points_size;
  LOG(INFO) << "--input_x_min=" << input_x_min;
  LOG(INFO) << "--input_x_max=" << input_x_max;
  LOG(INFO) << "--input_y_min=" << input_y_min;
  LOG(INFO) << "--input_y_max=" << input_y_max;
  LOG(INFO) << "--input_random_seed=" << input_random_seed;
  LOG(INFO) << "--output_points_obj = \"" << output_points_obj_filename << "\"";
  
  if (input_points_size < 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "--input_points_size=", input_points_size, " < 1."));
  }
  if (input_x_max < input_x_min) {
    return absl::InvalidArgumentError(absl::StrCat(
        "--input_x_max=", input_x_max, " < ", input_x_min, "= --input_x_min."));
  }
  if (input_y_max < input_y_min) {
    return absl::InvalidArgumentError(absl::StrCat(
        "--input_y_max=", input_y_max, " < ", input_y_min, "= --input_y_min."));
  }

  if (output_points_obj_filename.empty()) {
    return absl::InvalidArgumentError("--output_points_obj is empty.");
  }

  // Generate the points.
  const math::VectorOwned<double, 2> origin({input_x_min, input_y_min});
  math::VectorOwned<double, 2> diagonal({input_x_max, input_y_max});
  diagonal -= origin;
  
  MeshGeometry<double, 2> mesh;
  std::srand(input_random_seed);
  static constexpr double kRandMax = RAND_MAX;
  for (int i = 0; i < input_points_size; ++i) {
    math::VectorOwned<double, 2> point;
    for (int j = 0; j < 2; ++j) {
      point[j] = (diagonal[j] * std::rand()) / kRandMax;
    }
    point += origin;

    VXIndex vx = mesh.VXAllocate();
    mesh.VXSetValence(vx, 0);
    mesh.VXMutablePoint(vx) = point;
  }

  LOG(INFO) << "BEGIN: Write 2D points to .obj file.";
  ASSIGN_OR_RETURN(const std::string output_buffer, (hemesh::io::ToObj<double, 2>(mesh)));
  absl::Status status = file::SetContents(output_points_obj_filename, output_buffer);
  LOG(INFO) << "END  : Write 2D points to .obj file.";  

  return status;
}

}  // namespace
}  // namespace hemesh

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const absl::Status status = hemesh::Main(
      absl::GetFlag(FLAGS_input_points_size),
      absl::GetFlag(FLAGS_input_x_min),
      absl::GetFlag(FLAGS_input_x_max),
      absl::GetFlag(FLAGS_input_y_min),
      absl::GetFlag(FLAGS_input_y_max),
      absl::GetFlag(FLAGS_input_random_seed),
      absl::GetFlag(FLAGS_output_points_obj));
  if (!status.ok()) {
    LOG(ERROR) << "status = " << status;
    return 1;
  }
  return 0;
}
