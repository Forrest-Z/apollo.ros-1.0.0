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

#ifndef MODULES_DECISION_DECISION_H_
#define MODULES_DECISION_DECISION_H_

#include <string>

#include <ros/ros.h>

#include "apollo_common/apollo_app.h"
#include "apollo_common/macro.h"

/**
 * @namespace apollo::decision
 * @brief apollo::decision
 */
namespace apollo {
namespace decision {

class Decision : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;
  virtual ~Decision() = default;

 private:
  void OnTimer(const ros::TimerEvent& event);
  void PublishDecision();

 private:
  ros::Timer timer_;
};

}  // namespace decision
}  // namespace apollo

#endif  // MODULES_DECISION_DECISION_H_
