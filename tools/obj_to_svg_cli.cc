// Converts a 2D .obj to a .svg file.
// Usage:
//   BAZEL_ROOT=~/hemesh
//   cd ${BAZEL_ROOT}; bazel run //tools:obj_to_svg_cli -- \
//     --input_obj="${BAZEL_ROOT}/data/obj/triangle.obj" \
//     --output_svg="${BAZEL_ROOT}/data/svg/my_triangle.svg"

#include <memory>
#include <string>

#include "cc/file/file_helper.h"
#include "cc/hemesh/hemesh_geometry.h"
#include "cc/hemesh/hemesh_io_obj.h"
#include "cc/hemesh/hemesh_io_svg.h"
#include "absl/flags/commandlineflag.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

ABSL_FLAG(std::string, input_obj, "", "Input 2D .obj file.");
ABSL_FLAG(std::string, output_svg, "", "Output 2D .svg file.");

namespace hemesh {
namespace {

absl::Status Main(absl::string_view input_obj_filename,
                  absl::string_view output_svg_filename) {
  LOG(INFO) << "--input_obj = \"" << input_obj_filename << "\"";
  LOG(INFO) << "--output_svg = \"" << output_svg_filename << "\"";
  
  if (input_obj_filename.empty()) {
    return absl::InvalidArgumentError("--input_obj is empty.");
  }
  if (output_svg_filename.empty()) {
    return absl::InvalidArgumentError("--output_svg is empty.");
  }

  LOG(INFO) << "BEGIN: Read 2D .obj file.";  
  absl::StatusOr<std::string> input_buffer_or = file::GetContents(input_obj_filename);
  if (!input_buffer_or.ok()) {
    return input_buffer_or.status();
  }
  const std::string& input_buffer = *input_buffer_or;
  LOG(INFO) << "  BEGIN: Parse 2D .obj file.";  
  absl::StatusOr<std::unique_ptr<MeshGeometry<double, 2>>> mesh_or =
      hemesh::io::FromObj<double, 2>(input_buffer);
  if (!mesh_or.ok()) {
    return mesh_or.status();
  }
  std::unique_ptr<MeshGeometry<double, 2>> mesh = *std::move(mesh_or);
  LOG(INFO) << "  END  : Parse 2D .obj file.";  
  LOG(INFO) << "END  : Read 2D .obj file.";  

  // Convert .obj to .svg.

  LOG(INFO) << "BEGIN: Write 2D .svg file.";  
  absl::StatusOr<std::string> output_buffer_or =
      hemesh::io::Mesh2DToSvg<double>(*mesh, /* opt_bounding_box = */ nullptr);
  if (!output_buffer_or.ok()) {
    return output_buffer_or.status();
  }
  const std::string& output_buffer = *output_buffer_or;
  absl::Status status = file::SetContents(output_svg_filename, output_buffer);
  LOG(INFO) << "END  : Write 2D .svg file.";  

  return status;
}

}  // namespace
}  // namespace hemesh

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const absl::Status status = hemesh::Main(
      absl::GetFlag(FLAGS_input_obj), absl::GetFlag(FLAGS_output_svg));
  if (!status.ok()) {
    LOG(ERROR) << "status = " << status;
    return 1;
  }
  return 0;
}
