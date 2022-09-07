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

#include "apollo_control/filters/digital_filter.h"

#include <cmath>

#include "apollo_common/log.h"

namespace {

const double kDoubleEpsilon = 1.0e-6;

}  // namespace

namespace apollo {
namespace control {

DigitalFilter::DigitalFilter(const std::vector<double>& denominators,
                             const std::vector<double>& numerators) {
  dead_zone_ = 0.0;
  last_ = 0.0;
  set_coefficients(denominators, numerators);
}

void DigitalFilter::set_denominators(const std::vector<double>& denominators) {
  denominators_ = denominators;
  y_values_.resize(denominators_.size(), 0.0);
}

void DigitalFilter::set_numerators(const std::vector<double>& numerators) {
  numerators_ = numerators;
  x_values_.resize(numerators_.size(), 0.0);
}

void DigitalFilter::set_coefficients(const std::vector<double>& denominators,
                                     const std::vector<double>& numerators) {
  set_denominators(denominators);
  set_numerators(numerators);
}

void DigitalFilter::set_dead_zone(const double deadzone) {
  dead_zone_ = std::abs(deadzone);
  AINFO << "Setting digital filter dead zone = " << dead_zone_;
}

double DigitalFilter::Filter(const double x_insert) {
  if (denominators_.empty() || numerators_.empty()) {
    AERROR << "Empty denominators or numerators";
    return 0.0;
  }

  x_values_.pop_back();
  x_values_.push_front(x_insert);
  double xside = Compute(x_values_, numerators_, 0, numerators_.size() - 1);

  y_values_.pop_back();
  double yside = Compute(y_values_, denominators_, 1, denominators_.size() - 1);

  double y_insert = 0.0;
  if (std::abs(denominators_.front()) > kDoubleEpsilon) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);

  return UpdateLast(y_insert);
}

double DigitalFilter::UpdateLast(const double input) {
  double diff = std::abs(input - last_);
  if (diff < dead_zone_) {
    return last_;
  } else {
    last_ = input;
    return input;
  }
}

double DigitalFilter::Compute(const std::deque<double>& values,
                              const std::vector<double>& coefficients,
                              const std::size_t coeff_start,
                              const std::size_t coeff_end) {
  if (coeff_start > coeff_end || coeff_end >= coefficients.size()) {
    AERROR << "Invalid inputs.";
    return 0.0;
  }
  if (coeff_end - coeff_start + 1 != values.size()) {
    AERROR << "Sizes not match.";
    return 0.0;
  }
  double sum = 0.0;
  int i = coeff_start;
  for (auto& value : values) {
    sum += value * coefficients[i];
    ++i;
  }
  return sum;
}

const std::vector<double>& DigitalFilter::denominators() const {
  return denominators_;
}

const std::vector<double>& DigitalFilter::numerators() const {
  return numerators_;
}

double DigitalFilter::dead_zone() const { return dead_zone_; }

}  // namespace control
}  // namespace apollo
