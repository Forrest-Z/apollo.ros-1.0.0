/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/ 

/******************************************************************************
 * Copyright 2022 The Forrest Author. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef MODULES_COMMON_UTIL_FILE_H_
#define MODULES_COMMON_UTIL_FILE_H_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <string>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "apollo_common/log.h"
#include "apollo_common/util/util.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        ascii representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message,
                         const std::string &file_name) {
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::TextFormat;
  int file_descriptor = open(file_name.c_str(), O_WRONLY | O_CREAT, S_IRWXU);
  if (file_descriptor < 0) {
    // Failed to open;
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

/**
 * @brief Parses the content of the file specified by the file_name as ascii
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromASCIIFile(const std::string &file_name, MessageType *message) {
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::TextFormat;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  delete input;
  close(file_descriptor);
  return success;
}

/**
 * @brief Parses the content of the file specified by the file_name as binary
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromBinaryFile(const std::string &file_name,
                            MessageType *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  return message->ParseFromIstream(&input);
}

/**
 * @brief Parses the content of the file specified by the file_name as a
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromFile(const std::string &file_name, MessageType *message) {
  if (EndWith(file_name, ".bin")) {
    if (!GetProtoFromBinaryFile(file_name, message) &&
        !GetProtoFromASCIIFile(file_name, message)) {
      return false;
    }
  } else {
    if (!GetProtoFromASCIIFile(file_name, message) &&
        !GetProtoFromBinaryFile(file_name, message)) {
      return false;
    }
  }
  return true;
}
/**
 * @brief Check if the directory specified by directory_path exists
 *        and is indeed a directory.
 * @param directory_path Directory path.
 * @return If the directory specified by directory_path exists
 *         and is indeed a directory.
 */
bool DirectoryExists(const std::string &directory_path);

/**
 * @brief Check if a specified directory specified by directory_path exists.
 *        If not, recursively create the directory (and its parents).
 * @param directory_path Directory path.
 * @return If the directory does exist or its creation is successful.
 */
bool EnsureDirectory(const std::string &directory_path);

/**
 * @brief Remove all the files under a specified directory. Note that
 *        sub-directories are NOT affected.
 * @param directory_path Directory path.
 * @return If the action is successful.
 */
bool RemoveAllFiles(const std::string &directory_path);

}  // namespace util
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_UTIL_FILE_H_
