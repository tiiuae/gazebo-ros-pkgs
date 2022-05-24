/*
 * Copyright 2021 Technology Innovation Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef GAZEBO_PLUGINS__MESH_SIM_PLUGIN_HPP_
#define GAZEBO_PLUGINS__MESH_SIM_PLUGIN_HPP_

#include <map>
#include <mutex>
#include <set>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo_plugins {

struct DroneConfig {
    std::string name, mac;
};

struct Drone {
    DroneConfig config;
    double x = 0, y = 0, z = 0;
};

/// \brief This gazebo plugin simulates a mesh network.
class MeshSimPlugin : public gazebo::WorldPlugin {
public:
    MeshSimPlugin();

    virtual ~MeshSimPlugin();

protected:

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf);
    void OnUpdate(const gazebo::common::UpdateInfo& info);

private:
    void OnDroneConfig(const std_msgs::msg::String::SharedPtr);
    void PublishMeshInfo();

    gazebo::transport::NodePtr node_;

    gazebo::event::ConnectionPtr conn_;
    gazebo::physics::WorldPtr world_;

    double update_rate_;
    double update_interval_;
    gazebo::common::Time last_update_;

    std::string name_prefix_;
    double signal_loss_start_distance_;
    double signal_max_distance_;

    std::map<std::string, Drone> drones_by_name_;
    std::map<std::string, std::string> drone_names_by_mac_;
    std::mutex drones_mutex_;

    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drone_config_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mesh_info_pub_;
};

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__MESH_SIM_PLUGIN_HPP_
