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

#include "apollo_localization/localization.h"

#include "apollo_common/log.h"
#include "apollo_common/util/file.h"

#include "apollo_localization/camera/camera_localization.h"
#include "apollo_localization/common/localization_gflags.h"
#include "apollo_localization/rtk/rtk_localization.h"

namespace apollo {
namespace localization {

using apollo::common::Status;
using apollo::common::ErrorCode;

std::string Localization::Name() const {
  return FLAGS_localization_module_name;
}

void Localization::RegisterLocalizationMethods() {
  localization_factory_.Register(
      LocalizationConfig::RTK,
      []() -> LocalizationBase* { return new RTKLocalization(); });

  // TODO(Dong): Implement camera based localization method.
  localization_factory_.Register(
      LocalizationConfig::CAMERA,
      []() -> LocalizationBase* { return new CameraLocalization(); });
}

Status Localization::Init() {
  RegisterLocalizationMethods();
  if (!apollo::common::util::GetProtoFromFile(FLAGS_localization_config_file,
                                              &config_)) {
    AERROR << "failed to load localization config file "
           << FLAGS_localization_config_file;
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "failed to load localization config file: " +
                      FLAGS_localization_config_file);
  }

  return Status::OK();
}

Status Localization::Start() {
  localization_ =
      localization_factory_.CreateObject(config_.localization_type());
  if (!localization_) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "localization is not initialized with config : " +
                      config_.DebugString());
  }
  localization_->Start();

  return Status::OK();
}

void Localization::Stop() {}

}  // namespace localization
}  // namespace apollo
