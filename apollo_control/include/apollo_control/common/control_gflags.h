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

#ifndef MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_
#define MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_

#include <gflags/gflags.h>

// data file
DECLARE_string(control_conf_file);

DECLARE_double(min_alert_interval);
DECLARE_bool(enable_csv_debug);

// gflags for test purpose
DECLARE_bool(use_state_exact_match);

// temporary gflag for test purpose
DECLARE_bool(enable_speed_station_preview);

DECLARE_bool(enable_control_watchdog);
DECLARE_string(node_name);
DECLARE_bool(is_control_test_mode);
DECLARE_bool(use_preview_speed_for_table);

DECLARE_bool(enable_input_timestamp_check);

DECLARE_int32(max_localization_miss_num);
DECLARE_int32(max_chassis_miss_num);
DECLARE_int32(max_planning_miss_num);

DECLARE_double(max_acceleration_when_stopped);
DECLARE_double(max_abs_speed_when_stopped);

DECLARE_double(steer_angle_rate);

#endif  // MODULES_CONTROL_COMMON_CONTROL_GFLAGS_H_
