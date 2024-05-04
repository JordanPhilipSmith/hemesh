#include "cc/file/file_helper.h"

#include <fstream>
#include <iostream>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"

namespace file {

absl::StatusOr<std::string> GetContents(absl::string_view filename) {
  std::ifstream input_stream(std::string(filename.begin(), filename.end()));
  if (!input_stream.is_open()) {
    return absl::NotFoundError(absl::StrCat("Failed to open filename = \"", filename, "\"."));
  }
  std::string buffer((std::istreambuf_iterator<char>(input_stream)),
                     std::istreambuf_iterator<char>());
  input_stream.close();
  return buffer;
}

absl::Status SetContents(absl::string_view filename, absl::string_view buffer) {
  std::ofstream output_stream(std::string(filename.begin(), filename.end()));
  output_stream << buffer;
  output_stream.close();
  return absl::OkStatus();
}

}  // namespace file
