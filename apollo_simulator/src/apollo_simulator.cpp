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

#include "apollo_simulator/apollo_simulator.h"

#include <string>
#include <memory>
#include <utility>

#include "apollo_common/adapters/adapter_manager.h"
#include "apollo_common/log.h"
#include "apollo_common/math/quaternion.h"
#include "apollo_common/math/math_utils.h"
#include "apollo_msgs/proto/localization/localization.pb.h"
#include "apollo_msgs/proto/canbus/chassis.pb.h"

using apollo::common::adapter::AdapterManager;

Simulator::Simulator(tf2_ros::Buffer &tf_buffer)
  : tf_buffer_(tf_buffer) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  /* simple_planning_simulator parameters */
  private_nh.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh.param<double>("wheelbase", wheelbase_, 0.0);
  private_nh.param<double>("steer_transmission_ratio", steer_transmission_ratio_, 16.0);
  private_nh.param<double>("steer_single_direction_max_degree", steer_single_direction_max_degree_, 470.0);
  private_nh.param<std::string>("simulation_frame_id", simulation_frame_id_, std::string("base_link"));
  private_nh.param<std::string>("map_frame_id", map_frame_id_, std::string("map"));
  private_nh.param<bool>("add_measurement_noise", add_measurement_noise_, false);
  std::string full_param_name;
  if (private_nh.searchParam("footprint", full_param_name)) {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    private_nh.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      footprint_ = MakeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
    } else {
      ROS_FATAL("Values in the footprint specification must be numbers");
      throw std::runtime_error("Values in the footprint specification must be numbers");
    }
  } else {
    ROS_FATAL("search footprint param fail");
    throw std::runtime_error("search footprint param fail.");
  }

  /* set pub sub topic name */
  pub_pose_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  pub_speed_ = private_nh.advertise<geometry_msgs::Twist>("current_speed", 1);
  pub_footprint_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
  pub_trajectory_ = private_nh.advertise<nav_msgs::Path>("trajectory", 1);

  sub_initialtwist_ = nh.subscribe("/initial_twist", 10, &Simulator::CallbackInitialTwistStamped, this);
  sub_initialpose_ = nh.subscribe("/initial_pose", 10, &Simulator::CallbackInitialPoseWithCov, this);

  const double dt = 1.0 / loop_rate_;

  /* Timer */
  timer_simulation_ = nh.createTimer(ros::Duration(dt), &Simulator::TimerCallbackSimulation, this);

  /* set vehicle model parameters */
  {
    std::string vehicle_model_type_str;
    private_nh.param<std::string>("vehicle_model_type", vehicle_model_type_str, std::string("IDEAL_STEER"));
    ROS_INFO("vehicle_model_type = %s", vehicle_model_type_str.c_str());
    double vel_lim, vel_time_delay, vel_time_constant;
    private_nh.param<double>("vel_lim", vel_lim, 50.0);
    private_nh.param<double>("vel_time_delay", vel_time_delay, 0.25);
    private_nh.param<double>("vel_time_constant", vel_time_constant, 0.5);
    double accel_rate, acc_time_delay, acc_time_constant;
    private_nh.param<double>("accel_rate", accel_rate, 10.0);
    private_nh.param<double>("acc_time_delay", acc_time_delay, 0.1);
    private_nh.param<double>("acc_time_constant", acc_time_constant, 0.1);
    double steer_lim, steer_rate_lim, steer_time_delay, steer_time_constant, deadzone_delta_steer;
    private_nh.param<double>("steer_lim", steer_lim, 1.0);
    private_nh.param<double>("steer_rate_lim", steer_rate_lim, 5.0);
    private_nh.param<double>("steer_time_delay", steer_time_delay, 0.3);
    private_nh.param<double>("steer_time_constant", steer_time_constant, 0.3);
    private_nh.param<double>("deadzone_delta_steer", steer_time_constant, 0.0);

    if (vehicle_model_type_str == "IDEAL_STEER") {
      vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
      vehicle_model_ptr_ = std::make_shared<SimModelIdealSteer>(wheelbase_);
    } else if (vehicle_model_type_str == "DELAY_STEER") {
      vehicle_model_type_ = VehicleModelType::DELAY_STEER;
      vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteer>(
          vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, vel_time_delay,
          vel_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);
    } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
      vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
      vehicle_model_ptr_ = std::make_shared<SimModelTimeDelaySteerAccel>(
          vel_lim, steer_lim, accel_rate, steer_rate_lim, wheelbase_, dt, acc_time_delay,
          acc_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);
    } else {
      ROS_ERROR("Invalid vehicle_model_type. Initialization failed.");
    }
  }

  /* set normal distribution noises */
  {
    int random_seed;
    private_nh.param<int>("random_seed", random_seed, 1);
    if (random_seed >= 0) {
      rand_engine_ptr_ = std::make_shared<std::mt19937>(random_seed);
    } else {
      std::random_device seed;
      rand_engine_ptr_ = std::make_shared<std::mt19937>(seed());
    }
    double pos_noise_stddev, vel_noise_stddev, rpy_noise_stddev, angvel_noise_stddev, steer_noise_stddev;
    private_nh.param<double>("pos_noise_stddev", pos_noise_stddev, 0.01);
    private_nh.param<double>("vel_noise_stddev", vel_noise_stddev, 0.0);
    private_nh.param<double>("rpy_noise_stddev", rpy_noise_stddev, 0.0001);
    private_nh.param<double>("angvel_noise_stddev", angvel_noise_stddev, 0.0);
    private_nh.param<double>("steer_noise_stddev", steer_noise_stddev, 0.0001);
    pos_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    vel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    rpy_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    angvel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, angvel_noise_stddev);
    steer_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);
  }

  current_pose_.orientation.w = 1.0;
  closest_pos_z_ = 0.0;

  std::string adapter_config_path;
  private_nh.param<std::string>("adapter_config_path", adapter_config_path, std::string(""));
  AdapterManager::Init(adapter_config_path);
}

void Simulator::PublishChassis(const double &steering_angle, const geometry_msgs::Twist &twist) {
  apollo::canbus::Chassis chassis;
  AdapterManager::FillChassisHeader("chassis", chassis.mutable_header());
  chassis.set_driving_mode(apollo::canbus::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
  chassis.set_speed_mps(twist.linear.x);

  AdapterManager::PublishChassis(chassis);
  ADEBUG << chassis.ShortDebugString();
}

void Simulator::PublishLocalization(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist) {
  apollo::localization::LocalizationEstimate localization;
  // header
  AdapterManager::FillLocalizationHeader("localization", localization.mutable_header());
  // position
  auto mutable_pose = localization.mutable_pose();
  mutable_pose->mutable_position()->set_x(pose.position.x);
  mutable_pose->mutable_position()->set_y(pose.position.y);
  mutable_pose->mutable_position()->set_z(pose.position.z);
  // orientation(RFU)
  double roll, pitch, yaw;
  tf2::getEulerYPR(pose.orientation, yaw, pitch, roll);
  yaw = apollo::common::math::NormalizeAngle(yaw - M_PI_2);
  auto rfu_orientation = GetQuaternionFromRPY(roll, pitch, yaw);
  mutable_pose->mutable_orientation()->set_qw(rfu_orientation.w);
  mutable_pose->mutable_orientation()->set_qx(rfu_orientation.x);
  mutable_pose->mutable_orientation()->set_qy(rfu_orientation.y);
  mutable_pose->mutable_orientation()->set_qz(rfu_orientation.z);
  // car heading
  double heading = ::apollo::common::math::QuaternionToHeading(
      rfu_orientation.w, rfu_orientation.x,
      rfu_orientation.y, rfu_orientation.z);
  mutable_pose->set_heading(heading);
  // linear velocity(RFU)
  mutable_pose->mutable_linear_velocity()->set_x(-twist.linear.y);
  mutable_pose->mutable_linear_velocity()->set_y(twist.linear.x);
  mutable_pose->mutable_linear_velocity()->set_z(twist.linear.z);
  // angular velocity(RFU)
  mutable_pose->mutable_angular_velocity()->set_x(-twist.angular.y);
  mutable_pose->mutable_angular_velocity()->set_y(twist.angular.x);
  mutable_pose->mutable_angular_velocity()->set_z(twist.angular.z);

  // publish localization messages
  AdapterManager::PublishLocalization(localization);
  AINFO << "[OnTimer]: Localization message publish success!";
}

void Simulator::PublishPlanningTrajectory() {
  if (AdapterManager::GetPlanningTrajectory() == nullptr) {
    return;
  }
  if (AdapterManager::GetPlanningTrajectory()->Empty()) {
    return;
  }
  const auto &trajectory = AdapterManager::GetPlanningTrajectory()->GetLatestObserved();
  // AERROR << "trajectory: " << trajectory.ShortDebugString();
  nav_msgs::Path path;
  path.header.frame_id = map_frame_id_;
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped ps;
  ps.header = path.header;
  for (size_t i = 0; i < trajectory.adc_trajectory_point_size(); ++i) {
    ps.pose.position.x = trajectory.adc_trajectory_point(i).x();
    ps.pose.position.y = trajectory.adc_trajectory_point(i).y();
    ps.pose.position.z = 0;
    ps.pose.orientation = GetQuaternionFromRPY(0, 0, trajectory.adc_trajectory_point(i).theta());
    path.poses.push_back(ps);
  }
  pub_trajectory_.publish(path);
}

void Simulator::TimerCallbackSimulation(const ros::TimerEvent &event) {
  if (!is_initialized_) {
    ROS_INFO("waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr) {
    prev_update_time_ptr_ = std::make_shared<ros::Time>(ros::Time::now());
  }

  AdapterManager::Observe();
  if (AdapterManager::GetControlCommand() != nullptr && !AdapterManager::GetControlCommand()->Empty()) {
    const auto &ctrl_cmd = AdapterManager::GetControlCommand()->GetLatestObserved();
    // AERROR << "control_command: " << ctrl_cmd.ShortDebugString();

    double linear_vel = ctrl_cmd.debug().simple_lon_debug().speed_reference();
    double steer_angle = ctrl_cmd.debug().simple_lat_debug().steer_angle();
    steer_angle /= (180.0 / M_PI * steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100.0);
    if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER ||
      vehicle_model_type_ == VehicleModelType::DELAY_STEER) {
      Eigen::VectorXd input(2);
      input << linear_vel, steer_angle;
      vehicle_model_ptr_->setInput(input);
    } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
      Eigen::VectorXd input(3);
      double drive_shift = (linear_vel < 0) ? -1.0 : 1.0;
      // 注意：这里的input中acceleration不管前进还是后退都是正向增大的，但是msg中后退是负向增大的
      double acceleration = ctrl_cmd.debug().simple_lon_debug().acceleration_cmd();
      input << (acceleration * drive_shift), steer_angle, drive_shift;
      vehicle_model_ptr_->setInput(input);
      // ROS_INFO("target linear_vel: %.2f", linear_vel);
      // ROS_INFO("target acceleration: %.2f", acceleration);
      // ROS_INFO("target steer_angle: %.2f", steer_angle);
    } else {
      ROS_WARN("invalid vehicle_model_type_ error.");
    }
  }

  /* calculate delta time */
  const double dt = (ros::Time::now() - *prev_update_time_ptr_).toSec();
  *prev_update_time_ptr_ = ros::Time::now();

  /* update vehicle dynamics*/
  vehicle_model_ptr_->update(dt);

  /* save current vehicle pose & twist */
  current_pose_.position.x = vehicle_model_ptr_->getX();
  current_pose_.position.y = vehicle_model_ptr_->getY();
  closest_pos_z_ = 0.0;
  current_pose_.position.z = closest_pos_z_;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = vehicle_model_ptr_->getYaw();
  current_twist_.linear.x = vehicle_model_ptr_->getVx();
  current_twist_.angular.z = vehicle_model_ptr_->getWz();

  if (add_measurement_noise_) {
    current_pose_.position.x += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.y += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.z += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    roll += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    pitch += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    yaw += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    if (current_twist_.linear.x >= 0.0) {
      current_twist_.linear.x += (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    } else {
      current_twist_.linear.x -= (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    }
    current_twist_.angular.z += (*angvel_norm_dist_ptr_)(*rand_engine_ptr_);
  }

  current_pose_.orientation = GetQuaternionFromRPY(roll, pitch, yaw);

  /* publish pose */
  ros::Time current_time = ros::Time::now();
  geometry_msgs::PoseStamped sim_ps;
  sim_ps.header.frame_id = map_frame_id_;
  sim_ps.header.stamp = current_time;
  sim_ps.pose = current_pose_;
  pub_pose_.publish(sim_ps);

  /* publish speed */
  pub_speed_.publish(current_twist_);

  /* publish for apollo */
  double steering_angle = vehicle_model_ptr_->getSteer();
  if (add_measurement_noise_) {
    steering_angle += (*steer_norm_dist_ptr_)(*rand_engine_ptr_);
  }
  PublishChassis(steering_angle, current_twist_);
  PublishLocalization(current_pose_, current_twist_);
  PublishPlanningTrajectory();

  /* publish tf */
  PublishTF(current_pose_);

  /* publish footprint polygon */
  geometry_msgs::PolygonStamped footprint;
  footprint.header.frame_id = simulation_frame_id_;
  footprint.header.stamp = current_time;
  for (const auto &point : footprint_) {
    geometry_msgs::Point32 new_pt;
    new_pt.x = point.x;
    new_pt.y = point.y;
    footprint.polygon.points.push_back(new_pt);
  }
  pub_footprint_.publish(footprint);
}

void Simulator::CallbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  if (initial_twist_ptr_) {
    initial_twist = initial_twist_ptr_->twist;
  }
  // save initial pose
  initial_pose_with_cov_ptr_ = msg;
  SetInitialStateWithPoseTransform(*initial_pose_with_cov_ptr_, initial_twist);
}

void Simulator::CallbackInitialPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  if (initial_twist_ptr_) {
    initial_twist = initial_twist_ptr_->twist;
  }
  // save initial pose
  initial_pose_ptr_ = msg;
  SetInitialStateWithPoseTransform(*initial_pose_ptr_, initial_twist);
}

void Simulator::CallbackInitialTwistStamped(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  // save initial pose
  initial_twist_ptr_ = msg;
  if (initial_pose_ptr_) {
    SetInitialStateWithPoseTransform(*initial_pose_ptr_, initial_twist_ptr_->twist);
    // input twist to simulator's internal parameter
    current_pose_ = initial_pose_ptr_->pose;
    current_twist_ = initial_twist_ptr_->twist;
  } else if (initial_pose_with_cov_ptr_) {
    SetInitialStateWithPoseTransform(*initial_pose_with_cov_ptr_, initial_twist_ptr_->twist);
  }
}

void Simulator::SetInitialStateWithPoseTransform(
  const geometry_msgs::PoseStamped &pose_stamped, const geometry_msgs::Twist &twist) {
  geometry_msgs::TransformStamped transform;
  GetTransformFromTF(map_frame_id_, pose_stamped.header.frame_id, transform);
  geometry_msgs::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.transform.translation.x;
  pose.position.y = pose_stamped.pose.position.y + transform.transform.translation.y;
  pose.position.z = pose_stamped.pose.position.z + transform.transform.translation.z;
  pose.orientation = pose_stamped.pose.orientation;
  SetInitialState(pose, twist);
}

void Simulator::SetInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped &pose,
  const geometry_msgs::Twist &twist) {
  geometry_msgs::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  SetInitialStateWithPoseTransform(ps, twist);
}

void Simulator::SetInitialState(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist) {
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);
  const double vx = twist.linear.x;
  const double steer = 0.0;
  const double acc = 0.0;

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER) {
    Eigen::VectorXd state(3);
    state << x, y, yaw;
    vehicle_model_ptr_->setState(state);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER) {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, steer;
    vehicle_model_ptr_->setState(state);
  } else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    Eigen::VectorXd state(6);
    state << x, y, yaw, vx, steer, acc;
    vehicle_model_ptr_->setState(state);
  } else {
    ROS_WARN("undesired vehicle model type! Initialization failed.");
    return;
  }

  is_initialized_ = true;
}

void Simulator::GetTransformFromTF(const std::string parent_frame, const std::string child_frame,
  geometry_msgs::TransformStamped &transform) {
  while (ros::ok()) {
    try {
      transform = tf_buffer_.lookupTransform(parent_frame, child_frame, ros::Time(0));
      break;
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void Simulator::PublishTF(const geometry_msgs::Pose &pose) {
  ros::Time current_time = ros::Time::now();

  // send odom transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = map_frame_id_;
  odom_trans.child_frame_id = simulation_frame_id_;
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  tf_broadcaster_.sendTransform(odom_trans);
}

geometry_msgs::Quaternion Simulator::GetQuaternionFromRPY(
  const double &roll, const double &pitch, const double &yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

double Simulator::GetNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name) {
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string &value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
      full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

std::vector<geometry_msgs::Point> Simulator::MakeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc,
  const std::string &full_param_name, const size_t min_size) {
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xmlrpc.size() < min_size) {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
      full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
      "1 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;
  for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2) {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
        "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
        full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
        "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }
    pt.x = GetNumberFromXMLRPC(point[0], full_param_name);
    pt.y = GetNumberFromXMLRPC(point[1], full_param_name);
    footprint.push_back(pt);
  }
  return footprint;
}
