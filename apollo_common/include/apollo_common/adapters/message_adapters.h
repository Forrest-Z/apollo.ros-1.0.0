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

#ifndef MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
#define MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_

#include "apollo_common/adapters/adapter.h"

#include "apollo_msgs/proto/common/monitor.pb.h"
#include "apollo_msgs/proto/canbus/chassis.pb.h"
#include "apollo_msgs/proto/canbus/chassis_detail.pb.h"
#include "apollo_msgs/proto/localization/camera.pb.h"
#include "apollo_msgs/proto/localization/gps.pb.h"
#include "apollo_msgs/proto/localization/imu.pb.h"
#include "apollo_msgs/proto/localization/localization.pb.h"
#include "apollo_msgs/proto/perception/perception_obstacle.pb.h"
#include "apollo_msgs/proto/perception/traffic_light_detection.pb.h"
#include "apollo_msgs/proto/prediction/prediction_obstacle.pb.h"
#include "apollo_msgs/proto/decision/decision.pb.h"
#include "apollo_msgs/proto/planning/planning.pb.h"
#include "apollo_msgs/proto/control/control_cmd.pb.h"
#include "apollo_msgs/proto/control/pad_msg.pb.h"

/**
 * @file message_adapters.h
 * @namespace apollo::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace apollo {
namespace common {
namespace adapter {

using MonitorAdapter = Adapter<apollo::common::monitor::MonitorMessage>;
using ChassisAdapter = Adapter<::apollo::canbus::Chassis>;
using ChassisDetailAdapter = Adapter<::apollo::canbus::ChassisDetail>;
using GpsAdapter = Adapter<apollo::localization::Gps>;
using ImuAdapter = Adapter<::apollo::localization::Imu>;
using CameraAdapter = Adapter<::apollo::localization::Camera>;
using LocalizationAdapter = Adapter<apollo::localization::LocalizationEstimate>;
using PerceptionObstaclesAdapter = Adapter<::apollo::perception::PerceptionObstacles>;
using TrafficLightDetectionAdapter = Adapter<::apollo::perception::TrafficLightDetection>;
using PredictionAdapter = Adapter<::apollo::prediction::PredictionObstacles>;
using PlanningTrajectoryAdapter = Adapter<::apollo::planning::ADCTrajectory>;
using DecisionAdapter = Adapter<::apollo::decision::DecisionResult>;
using ControlCommandAdapter = Adapter<::apollo::control::ControlCommand>;
using PadAdapter = Adapter<::apollo::control::PadMessage>;

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif  // MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
