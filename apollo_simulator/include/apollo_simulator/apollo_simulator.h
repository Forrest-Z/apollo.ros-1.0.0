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

#pragma once

#include <memory>
#include <random>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include "apollo_simulator/vehicle_sim_model/sim_model_constant_acceleration.h"
#include "apollo_simulator/vehicle_sim_model/sim_model_ideal.h"
#include "apollo_simulator/vehicle_sim_model/sim_model_interface.h"
#include "apollo_simulator/vehicle_sim_model/sim_model_time_delay.h"

class Simulator {
 public:
  /**
   * @brief constructor
   */
  explicit Simulator(tf2_ros::Buffer &tf_buffer);

  /**
   * @brief default destructor
   */
  // ~Simulator() = default;

 protected:
  /**
   * @brief set initial pose for simulation with received message
   */
  void CallbackInitialPoseWithCov(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief set initial pose with received message
   */
  void CallbackInitialPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief set initial twist with received message
   */
  void CallbackInitialTwistStamped(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child frame id
   * @param [out] transform transform from parent frame to child frame
   */
  void GetTransformFromTF(
    const std::string parent_frame, const std::string child_frame,
    geometry_msgs::TransformStamped &transform);

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void TimerCallbackSimulation(const ros::TimerEvent &event);

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void SetInitialState(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void SetInitialStateWithPoseTransform(
    const geometry_msgs::PoseStamped &pose, const geometry_msgs::Twist &twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void SetInitialStateWithPoseTransform(
    const geometry_msgs::PoseWithCovarianceStamped &pose,
    const geometry_msgs::Twist &twist);

  /**
   * @brief publish tf
   * @param [in] pose pose used for tf
   */
  void PublishTF(const geometry_msgs::Pose &pose);

  /**
   * @brief convert roll-pitch-yaw Euler angle to ros Quaternion
   * @param [in] roll roll angle [rad]
   * @param [in] pitch pitch angle [rad]
   * @param [in] yaw yaw angle [rad]
   */
  geometry_msgs::Quaternion GetQuaternionFromRPY(
    const double &roll, const double &pitch, const double &yaw);

  void PublishChassis(const double &steering_angle, const geometry_msgs::Twist &twist);
  void PublishLocalization(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist);
  void PublishPlanningTrajectory();

  double GetNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name);
  std::vector<geometry_msgs::Point> MakeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc,
    const std::string &full_param_name, const size_t min_size = 1);

 private:
  /* ros system */
  ros::Publisher pub_pose_;  //!< @brief topic ros publisher for current pose
  ros::Publisher pub_speed_;  //!< @brief topic ros publisher for current speed
  ros::Publisher pub_footprint_;  //!< @brief topic ros publisher for vehicle footprint
  ros::Publisher pub_trajectory_;

  ros::Subscriber sub_initialpose_;  //!< @brief topic subscriber for initialpose topic
  ros::Subscriber sub_initialtwist_;  //!< @brief topic subscriber for initialtwist topic

  ros::Timer timer_simulation_;  //!< @brief timer for simulation

  /* tf */
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer &tf_buffer_;

  /* received & published topics */
  geometry_msgs::PoseStamped::ConstPtr initial_pose_ptr_;  //!< @brief initial vehicle pose
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr initial_pose_with_cov_ptr_;  //!< @brief initial vehicle pose with cov
  geometry_msgs::TwistStamped::ConstPtr initial_twist_ptr_;                      //!< @brief initial vehicle velocity
  geometry_msgs::Pose current_pose_;    //!< @brief current vehicle position and angle
  geometry_msgs::Twist current_twist_;  //!< @brief current vehicle velocity
  double closest_pos_z_;      //!< @brief z position on closest trajectory

  /* frame_id */
  std::string simulation_frame_id_;     //!< @brief vehicle frame id simulated by apollo_simulator
  std::string map_frame_id_;  //!< @brief map frame_id

  /* apollo_simulator parameters */
  double loop_rate_;  //!< @brief frequency to calculate vehicle model & publish result
  double wheelbase_;  //!< @brief wheelbase length to convert angular-velocity & steering
  std::vector<geometry_msgs::Point> footprint_;
  double steer_transmission_ratio_;
  double steer_single_direction_max_degree_;

  /* flags */
  bool is_initialized_ = false;                //!< @brief flag to check the initial position is set
  bool add_measurement_noise_;                 //!< @brief flag to add measurement noise

  /* saved values */
  std::shared_ptr<ros::Time> prev_update_time_ptr_;  //!< @brief previously updated time

  /* vehicle model */
  enum class VehicleModelType {
    IDEAL_TWIST = 0,
    IDEAL_STEER = 1,
    DELAY_TWIST = 2,
    DELAY_STEER = 3,
    CONST_ACCEL_TWIST = 4,
    IDEAL_FORKLIFT_RLS = 5,
    DELAY_FORKLIFT_RLS = 6,
    IDEAL_ACCEL = 7,
    DELAY_STEER_ACC = 8,
  } vehicle_model_type_;  //!< @brief vehicle model type to decide the model dynamics
  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer

  /* to generate measurement noise */
  std::shared_ptr<std::mt19937> rand_engine_ptr_;  //!< @brief random engine for measurement noise
  std::shared_ptr<std::normal_distribution<>> pos_norm_dist_ptr_;    //!< @brief Gaussian noise for position
  std::shared_ptr<std::normal_distribution<>> vel_norm_dist_ptr_;    //!< @brief Gaussian noise for velocity
  std::shared_ptr<std::normal_distribution<>> rpy_norm_dist_ptr_;    //!< @brief Gaussian noise for roll-pitch-yaw
  std::shared_ptr<std::normal_distribution<>> angvel_norm_dist_ptr_;    //!< @brief Gaussian noise for angular velocity
  std::shared_ptr<std::normal_distribution<>> steer_norm_dist_ptr_;    //!< @brief Gaussian noise for steering angle
};
