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

#include "apollo_control/controller/controller_agent.h"

#include <utility>

#include "apollo_common/log.h"
#include "apollo_common/time/time.h"

#include "apollo_control/controller/lat_controller.h"
#include "apollo_control/controller/lon_controller.h"

namespace apollo {
namespace control {

using apollo::common::time::Clock;

void ControllerAgent::RegisterControllers() {
  controller_factory_.Register(
      ControlConf::LAT_CONTROLLER,
      []() -> Controller * { return new LatController(); });
  controller_factory_.Register(
      ControlConf::LON_CONTROLLER,
      []() -> Controller * { return new LonController(); });
}

Status ControllerAgent::InitializeConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "control_conf is null";
    return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load config");
  }
  control_conf_ = control_conf;
  AINFO << control_conf_->DebugString();
  for (auto controller_type : control_conf_->active_controllers()) {
    auto controller = controller_factory_.CreateObject(
        static_cast<ControlConf::ControllerType>(controller_type));
    if (controller) {
      controller_list_.emplace_back(std::move(controller));
    } else {
      AERROR << "Controller: " << controller_type << "is not supported";
      return Status(ErrorCode::CONTROL_INIT_ERROR,
                    "Invalid controller type:" + controller_type);
    }
  }
  return Status::OK();
}

Status ControllerAgent::Init(const ControlConf *control_conf) {
  RegisterControllers();
  CHECK(InitializeConf(control_conf).ok()) << "Fail to initialize config.";
  for (auto &controller : controller_list_) {
    if (controller == NULL || !controller->Init(control_conf_).ok()) {
      if (controller != NULL) {
        AERROR << "Controller <" << controller->Name() << "> init failed!";
        return Status(ErrorCode::CONTROL_INIT_ERROR,
                      "Failed to init Controller:" + controller->Name());
      } else {
        return Status(ErrorCode::CONTROL_INIT_ERROR,
                      "Failed to init Controller");
      }
    }
    AINFO << "Controller <" << controller->Name() << "> init done!";
  }
  return Status::OK();
}

Status ControllerAgent::ComputeControlCommand(
    const ::apollo::localization::LocalizationEstimate *localization,
    const ::apollo::canbus::Chassis *chassis,
    const ::apollo::planning::ADCTrajectory *trajectory,
    ::apollo::control::ControlCommand *cmd) {
  for (auto &controller : controller_list_) {
    ADEBUG << "controller:" << controller->Name() << " processing ...";
    double start_timestamp = apollo::common::time::ToSecond(Clock::Now());
    controller->ComputeControlCommand(localization, chassis, trajectory, cmd);
    double end_timestamp = apollo::common::time::ToSecond(Clock::Now());
    AINFO << "controller: " << controller->Name() << " calculation time is: "
          << (end_timestamp - start_timestamp) * 1000 << " ms.";
    cmd->mutable_latency_stats()->add_controller_time_ms(
        (end_timestamp - start_timestamp) * 1000);
  }
  return Status::OK();
}

Status ControllerAgent::Reset() {
  for (auto &controller : controller_list_) {
    ADEBUG << "controller:" << controller->Name() << " reset...";
    controller->Reset();
  }
  return Status::OK();
}

}  // namespace control
}  // namespace apollo
