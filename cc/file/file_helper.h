#pragma once

// File I/O helper utilities.

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace file {

// Read the entire contents of a file.
// Parameters:
//   filename - Filename from which to read.
// Returns a buffer with the entire contents or status on error.
absl::StatusOr<std::string> GetContents(absl::string_view filename);

// Write the entire contents of a file.
// Parameters:
//   filename - Filename from which to read.
//   buffer - Buffer to write to the file.
// Returns status on error.
absl::Status SetContents(absl::string_view filename, absl::string_view buffer);

}  // namespace file
