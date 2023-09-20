// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <string>

#include "nav2_coverage/swath_mode.hpp"

namespace nav2_coverage
{

Swaths SwathMode::generateSwaths(const Field & field /*, request*/)
{
  SwathType action_type;  // = toType(request->swath_type)
  SwathAngleType action_angle_type;  // = toAngleType(request->swath_angle_type)
  SwathObjectivePtr objective{nullptr};
  float swath_angle = 0.0f;

  // If not set by action, use default mode
  if (action_type != SwathType::UNKNOWN && action_angle_type != SwathAngleType::UNKNOWN) {
    action_type = default_type_;
    action_angle_type = default_angle_type_;
    objective = default_objective_;
    swath_angle = default_swath_angle_;
  } else {
    objective = createObjective(action_type);
    // swath_angle = request->swath_angle;
  }

  RCLCPP_INFO(
    logger_, "Generating Swaths with: %s", toString(action_type, action_angle_type).c_str());

  generator_->setAllowOverlap(default_allow_overlap_);
  switch (action_angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!objective) {
        throw std::runtime_error("No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE.");
      }
      return generator_->generateBestSwaths(*objective, robot_->getWidth(), field);
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(swath_angle, robot_->getWidth(), field);
    default:
      throw std::runtime_error("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}

void SwathMode::setSwathMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_type_ = toType(mode);
  default_objective_ = createObjective(default_type_);
}

void SwathMode::setSwathAngleMode(const std::string & new_mode)
{
  std::string mode = new_mode;
  default_angle_type_ = toAngleType(mode);
}

SwathObjectivePtr SwathMode::createObjective(const SwathType & type)
{
  switch (type) {
    case SwathType::LENGTH:
      return std::move(std::make_shared<f2c::obj::SwathLength>());
    case SwathType::NUMBER:
      return std::move(std::make_shared<f2c::obj::NSwath>());
    case SwathType::COVERAGE:
      return std::move(std::make_shared<f2c::obj::FieldCoverage>());
    default:
      RCLCPP_WARN(logger_, "Unknown swath type set!");
      return SwathObjectivePtr{nullptr};
  }
}

std::string SwathMode::toString(const SwathType & type, const SwathAngleType & angle_type)
{
  std::string str;
  switch (type) {
    case SwathType::LENGTH:
      str = "Length";
      break;
    case SwathType::NUMBER:
      str = "Number";
      break;
    case SwathType::COVERAGE:
      str = "Coverage";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " Objective and ";

  switch (angle_type) {
    case SwathAngleType::SET_ANGLE:
      str = "Set Angle";
      break;
    case SwathAngleType::BRUTE_FORCE:
      str = "Brute Force";
      break;
    default:
      str = "Unknown";
      break;
  }

  str += " angle.";
  return str;
}

SwathType SwathMode::toType(std::string & str)
{
  toUpper(str);
  if (str == "LENGTH") {
    return SwathType::LENGTH;
  } else if (str == "NUMBER") {
    return SwathType::NUMBER;
  } else if (str == "COVERAGE") {
    return SwathType::COVERAGE;
  } else {
    return SwathType::UNKNOWN;
  }
}

SwathAngleType SwathMode::toAngleType(std::string & str)
{
  toUpper(str);
  if (str == "SET_ANGLE") {
    return SwathAngleType::SET_ANGLE;
  } else if (str == "BRUTE_FORCE") {
    return SwathAngleType::BRUTE_FORCE;
  } else {
    return SwathAngleType::UNKNOWN;
  }
}


}  // namespace nav2_coverage