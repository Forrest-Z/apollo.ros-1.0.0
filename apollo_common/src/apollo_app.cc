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

#include "apollo_common/apollo_app.h"

#include <csignal>
#include <string>

#include <gflags/gflags.h>
#include <ros/ros.h>

#include "apollo_common/log.h"
#include "apollo_common/status/status.h"

// #include "modules/hmi/utils/hmi_status_helper.h"

namespace apollo {
namespace common {

int ApolloApp::Spin() {
  ros::AsyncSpinner spinner(1);
  auto status = Init();
  if (!status.ok()) {
    AERROR << Name() << " Init failed: " << status;
    return -1;
  }
  status = Start();
  if (!status.ok()) {
    AERROR << Name() << " Start failed: " << status;
    return -2;
  }
  spinner.start();
  ros::waitForShutdown();
  Stop();
  AINFO << Name() << " exited.";
  return 0;
}

void apollo_app_sigint_handler(int signal_num) {
  AINFO << "Received signal: " << signal_num;
  if (signal_num != SIGINT) {
    return;
  }
  bool static is_stopping = false;
  if (is_stopping) {
    return;
  }
  is_stopping = true;
  ros::shutdown();
}

}  // namespace common
}  // namespace apollo
