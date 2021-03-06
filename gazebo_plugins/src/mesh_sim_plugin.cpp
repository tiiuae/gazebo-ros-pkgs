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

#include <gazebo_plugins/mesh_sim_plugin.hpp>

#include <cmath>
#include <vector>
#include <unordered_map>

#include <gazebo/physics/Model.hh>

#define JSON_USE_IMPLICIT_CONVERSIONS 0
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace gazebo_plugins {

void from_json(const json& j, DroneConfig& o) {
    j.at("name").get_to(o.name);
    j.at("mac").get_to(o.mac);
}

struct MeshNeighbor {
    std::string& mac;
    double signalStrength;

    MeshNeighbor(std::string& mac, double signalStrength)
        : mac(mac),
          signalStrength(signalStrength) {}

    bool operator<(const MeshNeighbor& o) const {
        if (signalStrength == o.signalStrength) {
            return mac < o.mac;
        }
        return signalStrength > o.signalStrength;
    }
};

void to_json(json& j, const MeshNeighbor& o) {
    j = json{
        {"mac", o.mac},
        {"signalStrength", o.signalStrength}
    };
}

MeshSimPlugin::MeshSimPlugin() = default;

MeshSimPlugin::~MeshSimPlugin() = default;

void MeshSimPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) {

    world_ = world;
    node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node_->Init(world_->Name());

    update_rate_ = 1.0;
    if (sdf->HasElement("updateRate")) {
        update_rate_ = sdf->GetElement("updateRate")->Get<double>();
    }
    update_interval_ = (update_rate_ > 0.0) ? 1/update_rate_ : 0.0;

    signal_loss_start_distance_ = 3000;
    if (sdf->HasElement("signalLossStartDistance")) {
        signal_loss_start_distance_ = sdf->GetElement("signalLossStartDistance")->Get<double>();
    }

    signal_max_distance_ = 10000;
    if (sdf->HasElement("signalMaxDistance")) {
        signal_max_distance_ = sdf->GetElement("signalMaxDistance")->Get<double>();
    }

    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MeshSimPlugin::OnUpdate, this, _1));

    last_update_ = world_->SimTime();

    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!rclcpp::ok()) {
      RCLCPP_FATAL_STREAM(
        ros_node_->get_logger(),
        "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
          "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    drone_config_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "drone_config",
        rclcpp::QoS(30).reliable().transient_local(),
        std::bind(&MeshSimPlugin::OnDroneConfig, this, std::placeholders::_1)
    );
    mesh_info_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
        "network_state",
        1
    );
}

// This gets called by the world update start event.
void MeshSimPlugin::OnUpdate(const gazebo::common::UpdateInfo& data) {
    // Get the current simulation time.
    auto now = world_->SimTime();
    if ((now - last_update_).Double() < update_interval_ || update_interval_ == 0.0) {
        return;
    }
    last_update_ = now;

    PublishMeshInfo();
}

void MeshSimPlugin::OnDroneConfig(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO_STREAM(
        ros_node_->get_logger(),
        "Got drone config: " << msg->data
    );
    try {
        auto config = json::parse(msg->data).get<DroneConfig>();
        if (config.name.empty()) {
            RCLCPP_WARN(
                ros_node_->get_logger(),
                "Invalid drone config: name must not be empty"
            );
            return;
        }
        if (config.mac.empty()) {
            RCLCPP_WARN(
                ros_node_->get_logger(),
                "Invalid drone config: mac must not be empty"
            );
            return;
        }
        const std::lock_guard<std::mutex> lock(drones_mutex_);
        auto name = drone_names_by_mac_.find(config.mac);
        if (name != drone_names_by_mac_.end() && name->second != config.name) {
            RCLCPP_WARN_STREAM(
                ros_node_->get_logger(),
                "Drone " << config.name
                << " will not be added to the mesh simulation because MAC address "
                << config.mac << " is already used by drone " << name->second << "."
            );
            return;
        }
        drone_names_by_mac_.erase(all_drones_by_name_[config.name].mac);
        drone_names_by_mac_.emplace(config.mac, config.name);
        all_drones_by_name_[config.name] = config;
        RCLCPP_INFO(ros_node_->get_logger(), "Drone config processed");
    } catch (json::exception& e) {
        RCLCPP_ERROR_STREAM(
            ros_node_->get_logger(),
            "Failed to parse drone config: " << e.what()
        );
    }
}

void MeshSimPlugin::PublishMeshInfo() {
    const std::lock_guard<std::mutex> lock(drones_mutex_);

    // Update positions of active drones
    for (auto& allIt : all_drones_by_name_) {
        if (auto model = world_->ModelByName(allIt.first)) {
            auto pos = model->WorldPose();
            auto& active = active_drones_by_name_[allIt.first];
            if (active.config.name.empty()) {
                active.config = allIt.second;
            }
            active.x = pos.X();
            active.y = pos.Y();
            active.z = pos.Z();
        } else {
            active_drones_by_name_.erase(allIt.first);
        }
    }

    // Build network graph
    std::unordered_map<std::string, std::set<MeshNeighbor>> mesh_nodes;
    for (auto& it1 : active_drones_by_name_) {
        auto& d1 = it1.second;
        auto& mesh_neighbors = mesh_nodes[d1.config.mac];
        for (auto& it2 : active_drones_by_name_) {
            auto& d2 = it2.second;
            if (d1.config.mac == d2.config.mac) continue;
            auto distance = std::sqrt(
                std::pow(d1.x - d2.x, 2)
                + std::pow(d1.y - d2.y, 2)
                + std::pow(d1.z - d2.z, 2)
            );
            auto slope = -1/(signal_max_distance_-signal_loss_start_distance_);
            auto strength = slope*(distance-signal_loss_start_distance_)+1;
            strength = std::max(0.0, std::min(1.0, strength));
            if (strength > 0) {
                mesh_neighbors.emplace(d2.config.mac, strength);
            }
        }
    }

    // Convert graph to JSON and publish
    try {
        std_msgs::msg::String msg;
        msg.data = json{{"meshGraph", mesh_nodes}}.dump();
        mesh_info_pub_->publish(msg);
        RCLCPP_DEBUG(ros_node_->get_logger(), "Published mesh info");
    } catch (json::exception& e) {
        RCLCPP_ERROR_STREAM(
            ros_node_->get_logger(),
            "Failed to encode mesh info: " << e.what()
        );
    }
}

GZ_REGISTER_WORLD_PLUGIN(MeshSimPlugin)
}
