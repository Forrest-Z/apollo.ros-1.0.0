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
 * @file pid_controller.h
 * @brief Defines the PIDController class.
 */

#ifndef MODULES_CONTROL_COMMON_PID_CONTROLLER_H_
#define MODULES_CONTROL_COMMON_PID_CONTROLLER_H_

#include "apollo_msgs/proto/control/lon_controller_conf.pb.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class PIDController
 * @brief A proportional–integral–derivative controller for speed and steering
 */
class PIDController {
 public:
  /**
   * @brief constructor
   */
  PIDController() = default;

  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void Init(const PidConf &pid_conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void SetPID(const PidConf &pid_conf);

  /**
   * @brief reset variables for pid controller
   */
  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  double Control(const double error, const double dt);

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int saturation_status() const;

  /**
   * @brief get status that if integrator is hold
   * @return if integrator is hold return true
   */
  bool integrator_hold() const;

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double saturation_high_ = 0.0;
  double saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int saturation_status_ = 0;
};

}  // namespace control
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_PID_CONTROLLER_H_
