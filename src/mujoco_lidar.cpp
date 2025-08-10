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
  const auto sensor_info = sensor_info_maybe.value();

  auto get_parameter = [&](const std::string& key) -> std::optional<std::string> {
    if (auto it = sensor_info.parameters.find(key); it != sensor_info.parameters.end())
    {
      return it->second;
    }
    return std::nullopt;
  };

  auto frame_name = get_parameter("frame_name");
  auto angle_increment = get_parameter("angle_increment");
  auto num_rangefinders = get_parameter("num_rangefinders");
  auto laserscan_topic = get_parameter("laserscan_topic");
  auto range_min = get_parameter("range_min");
  auto range_max = get_parameter("range_max");

  // If any required parameters are missing fire off an error.
  if (!frame_name || !angle_increment || !num_rangefinders)
  {
    return std::nullopt;
  }

  // Otherwise construct and return a new LidarData object
  LidarData data;
  data.name = name;
  data.frame_name = *frame_name;
  data.num_rangefinders = std::stoi(*num_rangefinders);
  data.angle_increment = std::stod(*angle_increment);
  data.laserscan_topic = laserscan_topic.has_value() ? laserscan_topic.value() : "scan";
  data.range_min = range_min.has_value() ? std::stod(range_min.value()) : 0.0;
  data.range_max = range_max.has_value() ? std::stod(range_max.value()) : 1000.0;

  // Compute based on ROS control config (could we get this from sites or something?)
  data.min_angle = 0.0;
  data.max_angle = data.angle_increment * data.num_rangefinders;
  data.sensor_indexes.resize(data.num_rangefinders);

  // Configure the static parameters of the laserscan message
  data.laser_scan_msg.header.frame_id = data.frame_name;
  data.laser_scan_msg.time_increment = 0.0;  // Does this matter?
  data.laser_scan_msg.scan_time = 0.0;       // Does this matter?
  data.laser_scan_msg.angle_min = data.min_angle;
  data.laser_scan_msg.angle_max = data.max_angle;
  data.laser_scan_msg.angle_increment = data.angle_increment;
  data.laser_scan_msg.range_min = data.range_min;
  data.laser_scan_msg.range_max = data.range_max;
  data.laser_scan_msg.ranges.resize(data.num_rangefinders);
  data.laser_scan_msg.intensities.resize(0);

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

    // If we have seen this before, update the data, otherwise create a new one and add it to the list.
    auto lidar_it = std::find_if(lidar_sensors_.begin(), lidar_sensors_.end(),
                                 [&lidar_name](const LidarData& data) { return data.name == lidar_name; });

    if (lidar_it == lidar_sensors_.end())
    {
      auto new_data_maybe = get_lidar_data(hardware_info, lidar_name);
      if (!new_data_maybe.has_value())
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "Failed to parse required configuration from ros2_control xacro: " << lidar_name);
        return false;
      }

      // Manually configure the publisher
      auto lidar = new_data_maybe.value();
      lidar.scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar.laserscan_topic, 1);

      // Add it to relevant containers
      lidar_sensors_.push_back(std::move(lidar));

      // For updating the index below
      lidar_it = lidar_sensors_.end() - 1;

      // Note that we have added the sensor
      RCLCPP_INFO_STREAM(node_->get_logger(), "Adding lidar sensor: " << lidar.name << ", idx: " << idx);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    frame_name: " << lidar.frame_name);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    num_rangefinders: " << lidar.num_rangefinders);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    min_angle: " << lidar.min_angle);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    max_angle: " << lidar.max_angle);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    angle_increment: " << lidar.angle_increment);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    max_angle: " << lidar.max_angle);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    range_min: " << lidar.range_min);
      RCLCPP_INFO_STREAM(node_->get_logger(), "    range_max: " << lidar.range_max);
    }

    // Add this range to the sensor finders data array. There's technically no guarantee that the data is
    // sequential so we're just tracking this directly.
    lidar_it->sensor_indexes[idx] = i;
  }

  // TODO: Verify that everything actually got filled in correctly...
  for (const auto& lidar : lidar_sensors_)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Lidar Sensor: " << lidar.name);
    for (size_t j = 0; j < lidar.sensor_indexes.size(); ++j)
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "  sensor_indexes[" << j << "] = " << lidar.sensor_indexes[j]);
    }
  }

  return true;
}

void MujocoLidar::init()
{
  // Start the rendering thread process
  publish_lidar_ = true;
  rendering_thread_ = std::thread(&MujocoLidar::update_loop, this);
}

void MujocoLidar::close()
{
  publish_lidar_ = false;
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }
}

void MujocoLidar::update_loop()
{
  // Setup container for lidar data. Someday this should just be a copy of the sensordata in the model.
  mj_lidar_data_ = mj_makeData(mj_model_);

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Starting the lidar processing loop, publishing at " << lidar_publish_rate_ << " Hz");

  rclcpp::Rate rate(lidar_publish_rate_);
  while (rclcpp::ok() && publish_lidar_)
  {
    update();
    rate.sleep();
  }
}

void MujocoLidar::update()
{
  // Step 1: Lock the sime and copy data for use lidar rendering.
  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mjv_copyData(mj_lidar_data_, mj_model_, mj_data_);
  }

  // Step 2: Copy sensor information for lidar to the relevant containers, filtering as needed
  for (auto& lidar : lidar_sensors_)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Lidar Sensor: " << lidar.name);
    for (size_t idx = 0; idx < lidar.sensor_indexes.size(); ++idx)
    {
      const auto& i = lidar.sensor_indexes[idx];
      auto range = mj_lidar_data_->sensordata[i];
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "  sensor_indexes[" << idx << "] = " << lidar.sensor_indexes[idx]
                                                                   << " - " << mj_lidar_data_->sensordata[i]);

      if (range < lidar.range_min || range > lidar.range_max)
      {
        range = -1.0;
      }
      lidar.laser_scan_msg.ranges[idx] = range;
    }
  }

  // Step 3: Publish messages
  for (auto& lidar : lidar_sensors_)
  {
    lidar.laser_scan_msg.header.stamp = node_->now();
    lidar.scan_pub->publish(lidar.laser_scan_msg);
  }
}

}  // namespace mujoco_ros2_simulation
