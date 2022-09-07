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

#include "apollo_control/controller/lat_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/LU>

#include "apollo_common/log.h"
#include "apollo_common/math/linear_quadratic_regulator.h"
#include "apollo_common/math/math_utils.h"
#include "apollo_common/time/time.h"

#include "apollo_control/common/control_gflags.h"

namespace apollo {
namespace control {

using common::Point3D;
using Matrix = Eigen::MatrixXd;
using ::apollo::common::vehicle_state::VehicleState;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           localtime(&raw_time));
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream& file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

LatController::LatController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

LatController::~LatController() { CloseLogFile(); }

bool LatController::LoadControlConf(const ControlConf* control_conf) {
  if (!control_conf) {
    AERROR << "[LatController] control_conf == nullptr";
    return false;
  }
  ts_ = control_conf->lat_controller_conf().ts();
  CHECK_GT(ts_, 0.0) << "[LatController] Invalid control update interval.";
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  wheelbase_ = control_conf->lat_controller_conf().wheelbase();
  steer_transmission_ratio_ =
      control_conf->lat_controller_conf().steer_transmission_ratio();
  steer_single_direction_max_degree_ =
      control_conf->lat_controller_conf().steer_single_direction_max_degree();
  double mass_fl = control_conf->lat_controller_conf().mass_fl();
  double mass_fr = control_conf->lat_controller_conf().mass_fr();
  double mass_rl = control_conf->lat_controller_conf().mass_rl();
  double mass_rr = control_conf->lat_controller_conf().mass_rr();
  double mass_front = mass_fl + mass_fr;
  double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();
  return true;
}

void LatController::ProcessLogs(const SimpleLateralDebug* debug,
                                const canbus::Chassis* chassis) {
  std::stringstream log_stream;
  log_stream << debug->lateral_error() << "," << debug->ref_heading() << ","
             << vehicle_state_.heading() << "," << debug->heading_error() << ","
             << debug->heading_error_rate() << ","
             << debug->lateral_error_rate() << "," << debug->curvature() << ","
             << debug->steer_angle() << "," << debug->steer_angle_feedforward()
             << "," << debug->steer_angle_lateral_contribution() << ","
             << debug->steer_angle_lateral_rate_contribution() << ","
             << debug->steer_angle_heading_contribution() << ","
             << debug->steer_angle_heading_rate_contribution() << ","
             << debug->steer_angle_feedback() << ","
             << chassis->steering_percentage() << ","
             << vehicle_state_.linear_velocity();
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_stream.str() << std::endl;
  }
  // AINFO << "Steer_Control_Detail: " << log_stream.str();
}

void LatController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void LatController::InitializeFilters(const ControlConf* control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(ts_, control_conf->lat_controller_conf().cutoff_freq(), &den,
                  &num);
  digital_filter_.set_coefficients(den, num);
  // Mean filters
  /**
  heading_rate_filter_ = MeanFilter(
      control_conf->lat_controller_conf().mean_filter_window_size());
  **/
  lateral_error_filter_ =
      MeanFilter(control_conf->lat_controller_conf().mean_filter_window_size());
}

Status LatController::Init(const ControlConf* control_conf) {
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(2, 3) = 1.0;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int q_param_size = control_conf->lat_controller_conf().matrix_q_size();
  if (matrix_size != q_param_size) {
    AERROR << "matrix_q size: " << q_param_size
           << " in parameter file not equal to matrix_size: " << matrix_size;
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "lateral controller error: matrix_q size: " +
                      std::to_string(q_param_size) +
                      " in parameter file not equal to matrix_size: " +
                      std::to_string(matrix_size));
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->lat_controller_conf().matrix_q(i);
  }

  InitializeFilters(control_conf);
  LogInitParameters();
  return Status::OK();
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

Status LatController::ComputeControlCommand(
    const localization::LocalizationEstimate* localization,
    const canbus::Chassis* chassis,
    const planning::ADCTrajectory* planning_published_trajectory,
    ControlCommand* cmd) {
  vehicle_state_ = std::move(VehicleState(localization, chassis));
  vehicle_state_.set_linear_velocity(std::max(vehicle_state_.linear_velocity(), 1.0));

  trajectory_analyzer_ = std::move(TrajectoryAnalyzer(planning_published_trajectory));

  SimpleLateralDebug* debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  if (FLAGS_use_state_exact_match) {
    UpdateStateAnalyticalMatching(debug);
  } else {
    UpdateState(debug);
  }

  UpdateMatrix();

  // Compound discrete matrix with road preview model
  UpdateMatrixCompound();

  common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_, matrix_r_,
                                lqr_eps_, lqr_max_iteration_, &matrix_k_);

  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steer degree
  // then to 100% ratio
  double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                M_PI * steer_transmission_ratio_ /
                                steer_single_direction_max_degree_ * 100;

  double steer_angle_feedforward = ComputeFeedForward(debug->curvature());
  double steer_angle = steer_angle_feedback + steer_angle_feedforward;
  // Clamp the steer angle to -100.0 to 100.0
  steer_angle = apollo::common::math::Clamp(steer_angle, -100.0, 100.0);

  steer_angle = digital_filter_.Filter(steer_angle);
  cmd->set_steering_target(steer_angle);
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  // compute extra information for logging and debugging
  double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI *
      steer_transmission_ratio_ / steer_single_direction_max_degree_ * 100;

  // TODO(yifei): move up temporary values to use debug fields.

  debug->set_heading(vehicle_state_.heading());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_position(chassis->steering_percentage());
  debug->set_ref_speed(vehicle_state_.linear_velocity());
  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status LatController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Status::OK();
}

// state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error
// Rate, Preview Lateral1, Preview Lateral2, ...]
void LatController::UpdateState(SimpleLateralDebug* debug) {
  TrajectoryPoint traj_point;
  Eigen::Vector2d com = vehicle_state_.ComputeCOMPosition(lr_);
  double raw_lateral_error = GetLateralError(com, &traj_point);

  // lateral_error_ = lateral_rate_filter_.Filter(raw_lateral_error);
  debug->set_lateral_error(lateral_error_filter_.Update(raw_lateral_error));

  // ref_curvature_ = traj_point.kappa;
  debug->set_curvature(traj_point.kappa);

  // ref_heading_ = traj_point.theta;
  debug->set_ref_heading(traj_point.theta);

  // heading_error_ =
  //    common::math::NormalizeAngle(vehicle_state_.heading() - ref_heading_);
  debug->set_heading_error(common::math::NormalizeAngle(vehicle_state_.heading() - traj_point.theta));

  // heading_error_rate_ = (heading_error_ - previous_heading_error_) / ts_;
  debug->set_heading_error_rate(
      (debug->heading_error() - previous_heading_error_) / ts_);
  // lateral_error_rate_ = (lateral_error_ - previous_lateral_error_) / ts_;
  debug->set_lateral_error_rate(
      (debug->lateral_error() - previous_lateral_error_) / ts_);

  // Prepare for next iteration.
  previous_heading_error_ = debug->heading_error();
  previous_lateral_error_ = debug->lateral_error();

  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    double preview_time = ts_ * (i + 1);
    Eigen::Vector2d future_position_estimate =
        vehicle_state_.EstimateFuturePosition(preview_time);
    double preview_lateral = GetLateralError(future_position_estimate, nullptr);
    matrix_state_(basic_state_size_ + i, 0) = preview_lateral;
  }
  // preview matrix update;
}

void LatController::UpdateStateAnalyticalMatching(SimpleLateralDebug* debug) {
  Eigen::Vector2d com = vehicle_state_.ComputeCOMPosition(lr_);
  ComputeLateralErrors(com.x(), com.y(), vehicle_state_.heading(),
                       vehicle_state_.linear_velocity(),
                       vehicle_state_.angular_velocity(), trajectory_analyzer_,
                       debug);

  // State matrix update;
  // First four elements are fixed;
  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    double preview_time = ts_ * (i + 1);
    auto preview_point =
        trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

    auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
        preview_point.x, preview_point.y);

    double dx = preview_point.x - matched_point.x;
    double dy = preview_point.y - matched_point.y;

    double cos_matched_theta = std::cos(matched_point.theta);
    double sin_matched_theta = std::sin(matched_point.theta);
    double preview_d_error = cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

void LatController::UpdateMatrix() {
  double v = vehicle_state_.linear_velocity();
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *
               (matrix_i - ts_ * 0.5 * matrix_a_).inverse();
}

void LatController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update augument A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LatController::ComputeFeedForward(double ref_curvature) const {
  double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // then change it from rad to %
  double v = vehicle_state_.linear_velocity();
  double steer_angle_feedforwardterm =
      (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
       matrix_k_(0, 2) *
           (lr_ * ref_curvature -
            lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
      180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;
  return steer_angle_feedforwardterm;
}

/*
 * SL coordinate system:
 *  left to the ref_line, L is +
 * right to the ref_line, L is -
 */
double LatController::GetLateralError(const Eigen::Vector2d& point,
                                      TrajectoryPoint* traj_point) const {
  auto closest =
      trajectory_analyzer_.QueryNearestPointByPosition(point.x(), point.y());

  double point_angle = std::atan2(point.y() - closest.y, point.x() - closest.x);
  double point2path_angle = point_angle - closest.theta;
  if (traj_point != nullptr) {
    *traj_point = closest;
  }

  double dx = closest.x - point.x();
  double dy = closest.y - point.y();
  return std::sin(point2path_angle) * std::sqrt(dx * dx + dy * dy);
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const TrajectoryAnalyzer& trajectory_analyzer,
    SimpleLateralDebug* debug) const {
  auto matched_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);

  double dx = x - matched_point.x;
  double dy = y - matched_point.y;

  double cos_matched_theta = std::cos(matched_point.theta);
  double sin_matched_theta = std::sin(matched_point.theta);
  // d_error = cos_matched_theta * dy - sin_matched_theta * dx;
  debug->set_lateral_error(cos_matched_theta * dy - sin_matched_theta * dx);

  double delta_theta =
      common::math::NormalizeAngle(theta - matched_point.theta);
  double sin_delta_theta = std::sin(delta_theta);
  // d_error_dot = linear_v * sin_delta_theta;
  debug->set_lateral_error_rate(linear_v * sin_delta_theta);

  // theta_error = delta_theta;
  debug->set_heading_error(delta_theta);
  // theta_error_dot = angular_v - matched_point.kappa * matched_point.v;
  debug->set_heading_error_rate(angular_v -
                                matched_point.kappa * matched_point.v);

  // matched_theta = matched_point.theta;
  debug->set_ref_heading(matched_point.theta);
  // matched_kappa = matched_point.kappa;
  debug->set_curvature(matched_point.kappa);
}

}  // namespace control
}  // namespace apollo
