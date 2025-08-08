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

namespace mujoco_ros2_simulation
{

MujocoLidar::MujocoLidar(rclcpp::Node::SharedPtr& node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                         mjModel* mujoco_model, double lidar_publish_rate)
  : node_(node)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , lidar_publish_rate_(lidar_publish_rate)
{
}

void MujocoLidar::register_lidar(const hardware_interface::HardwareInfo& hardware_info)
{
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
