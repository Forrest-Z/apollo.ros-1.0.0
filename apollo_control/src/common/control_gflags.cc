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

#include "apollo_control/common/control_gflags.h"

DEFINE_double(min_alert_interval, 1.,
              "minimum alert interval to prevent alerting too frequently.");
DEFINE_string(control_conf_file, "modules/control/conf/lincoln.pb.txt",
              "default control conf data file");
DEFINE_bool(enable_csv_debug, false, "True to write out csv debug file.");
DEFINE_bool(use_state_exact_match, false,
            "whether use state exact match in lateral controller");
DEFINE_bool(enable_speed_station_preview, true, "enable speed/station preview");
DEFINE_bool(enable_control_watchdog, true, "True to enable control watchdog");
DEFINE_string(node_name, "control", "The control node name in proto");
DEFINE_bool(is_control_test_mode, false, "True to run control in test mode");
DEFINE_bool(use_preview_speed_for_table, false,
        "True to use preview speed for table lookup");

DEFINE_bool(enable_input_timestamp_check, true,
            "True to enable input timestamp delay check");

DEFINE_int32(max_localization_miss_num, 20,
             "Max missing number of localization before entering estop mode");
DEFINE_int32(max_chassis_miss_num, 20,
             "Max missing number of chassis before entering estop mode");
DEFINE_int32(max_planning_miss_num, 20,
             "Max missing number of planning before entering estop mode");

DEFINE_double(max_acceleration_when_stopped, 0.01,
              "max acceleration can be observed when vehicle is stopped");
DEFINE_double(max_abs_speed_when_stopped, 0.01,
              "max absolute speed can be observed when vehicle is stopped");
DEFINE_double(steer_angle_rate, 100.0,
              "Steer angle change rate in percentage.");
