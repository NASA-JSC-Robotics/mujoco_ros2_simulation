/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <unordered_map>

#include "mujoco_ros2_simulation/mujoco_lidar.hpp"
#include "mujoco_ros2_simulation/utils.hpp"

namespace mujoco_ros2_simulation
{

/**
 * Given a sensor in the form <sensor_name>-id, returns the name and id if possible.
 * Otherwise, return "" and -1 to indicate failure.
 */
std::pair<std::string, int> parse_lidar_name(const std::string& sensor_name)
{
  const auto split_idx = sensor_name.find_last_of("-");
  if (split_idx == std::string::npos)
  {
    return { sensor_name, -1 };
  }

  // Grab the lidar sensor name
  const auto lidar_name = sensor_name.substr(0, split_idx);

  // Grab the index of the sensor name, if possible
  const auto lidar_idx = sensor_name.substr(split_idx + 1);
  if (lidar_idx.empty() || !std::all_of(lidar_idx.begin(), lidar_idx.end(), ::isdigit))
  {
    return { lidar_name, -1 };
  }

  return { lidar_name, std::stoi(lidar_idx) };
}

/**
 * Construct a LidarData object given a string sensor name and hardware_info object to parse.
 */
std::optional<LidarData> get_lidar_data(const hardware_interface::HardwareInfo& hardware_info, const std::string& name)
{
  const auto sensor_info_maybe = get_sensor_from_info(hardware_info, name);
  if (!sensor_info_maybe.has_value())
  {
    return std::nullopt;
  }

  auto get_parameter = [&](const std::string& key) -> std::optional<std::string> {
    if (auto it = hardware_info.hardware_parameters.find(key); it != hardware_info.hardware_parameters.end())
    {
      return it->second;
    }
    return std::nullopt;
  };

  auto frame_name = get_parameter("frame_name");
  auto angle_increment = get_parameter("angle_increment");
  auto num_rangefinders = get_parameter("num_rangefinders");
  auto laserscan_topic = get_parameter("laserscan_topic");

  // If any required parameters are missing fire off an error.
  if (!frame_name || !angle_increment || !num_rangefinders || !laserscan_topic)
  {
    return std::nullopt;
  }

  // Otherwise construct and return a new LidarData object
  LidarData data;
  data.name = name;
  data.frame_name = *frame_name;
  data.num_rangefinders = std::stoi(*num_rangefinders);
  data.angle_increment = std::stod(*angle_increment);
  data.laserscan_topic = *laserscan_topic;

  return data;
}

MujocoLidar::MujocoLidar(rclcpp::Node::SharedPtr& node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                         mjModel* mujoco_model, double lidar_publish_rate)
  : node_(node)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , lidar_publish_rate_(lidar_publish_rate)
{
}

bool MujocoLidar::register_lidar(const hardware_interface::HardwareInfo& hardware_info)
{
  lidar_sensors_.resize(0);

  // Map to store sensor names and LidarData as we iterate.
  std::unordered_map<std::string, LidarData> lidar_names;

  // Iterate over sensors and identify the rangefinders, then attempt to match them to
  // a relevant LidarData object.
  for (int i = 0; i < mj_model_->nsensor; ++i)
  {
    // Skip non-rangefinder sensors.
    if (mj_model_->sensor_type[i] != mjtSensor::mjSENS_RANGEFINDER)
    {
      continue;
    }

    // Grab the name of the sensor, which is required.
    const auto sensor_name_maybe = mj_id2name(mj_model_, mjtObj::mjOBJ_SENSOR, i);
    if (sensor_name_maybe == nullptr)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Cannot find a name for lidar sensor at index: " << i << ", skipping!");
      continue;
    }
    const std::string sensor_name(sensor_name_maybe);

    // If it is a rangefinder, we expect the name to be of the form `<sensor_name>-###`
    // The number of integers after the hyphen varies depending on the replicate.
    const auto [lidar_name, idx] = parse_lidar_name(sensor_name);
    if (idx == -1)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to parse lidar sensor name: " << sensor_name << ", skipping!");
      continue;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Found lidar sensor with name: " << lidar_name << ", idx: " << idx);

    // If we have seen this before, update the data, otherwise create a new one and add it to the map.
    if (auto it = lidar_names.find(lidar_name); it == lidar_names.end())
    {
      auto new_data_maybe = get_lidar_data(hardware_info, lidar_name);
      if (!new_data_maybe.has_value())
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "Failed to parse required configuration from ros2_control xacro: " << sensor_name);
        return false;
      }
      // TODO: Pickup here
    }
  }

  return true;
}

void MujocoLidar::init()
{
}

void MujocoLidar::close()
{
}

void MujocoLidar::update_loop()
{
}

void MujocoLidar::update()
{
}

}  // namespace mujoco_ros2_simulation
