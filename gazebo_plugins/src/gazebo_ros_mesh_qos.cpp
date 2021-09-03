// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_mesh_qos.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

namespace gazebo_plugins
{

const double g_default_max_distance_meters    = 10000.0; //10km
const double g_default_loss_start_dist_meters =  3000.0; // 3km

typedef struct {
  double x;
  double y;
  double z;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
} DroneInfo;

/// Class to hold private data members (PIMPL pattern)
class GazeboRosMeshQosPrivate
{
public:

  unsigned int GetModelPositions();
  void GenerateMeshQuality();

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::WorldPtr world_;

  double update_rate_;
  double update_interval_;
  gazebo::common::Time last_update_;
  std::string name_prefix_;

  std::map<std::string, DroneInfo> drones_;

  double max_distance_meters_;
  double loss_start_dist_meters_;
};

GazeboRosMeshQos::GazeboRosMeshQos()
: impl_(std::make_unique<GazeboRosMeshQosPrivate>())
{
}

GazeboRosMeshQos::~GazeboRosMeshQos()
{
}

void GazeboRosMeshQos::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  impl_->world_ = world;

  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  RCLCPP_INFO(impl_->ros_node_->get_logger(), world->Name().c_str());

  impl_->update_rate_ = 1.0;
  if (sdf->HasElement("updateRate")) {
      impl_->update_rate_ = sdf->GetElement("updateRate")->Get<double>();
  }
  impl_->update_interval_ = (impl_->update_rate_ > 0.0) ? 1/impl_->update_rate_ : 0.0;

  impl_->name_prefix_ = "ssrc_fog_x-";
  if (sdf->HasElement("namePrefix")) {
      impl_->name_prefix_ = sdf->GetElement("namePrefix")->Get<std::string>();
  }

  impl_->max_distance_meters_ = g_default_max_distance_meters;
  if (sdf->HasElement("maxDistance")) {
      impl_->max_distance_meters_ = sdf->GetElement("maxDistance")->Get<double>();
  }

  impl_->loss_start_dist_meters_ = g_default_loss_start_dist_meters;
  if (sdf->HasElement("lossStartDistance")) {
      impl_->loss_start_dist_meters_ = sdf->GetElement("lossStartDistance")->Get<double>();
  }

  impl_->last_update_ = world->SimTime();

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMeshQos::OnUpdate, this));
}

void GazeboRosMeshQos::OnUpdate()
{
  // Do something every simulation iteration
  gazebo::common::Time now = impl_->world_->SimTime();

  if ((now - impl_->last_update_).Double() < impl_->update_interval_ || impl_->update_interval_ == 0.0) {
    return;
  }
  impl_->last_update_ = now;

  if ( impl_->GetModelPositions() ) {
    impl_->GenerateMeshQuality();
  }
}


// Parse drone frames from simulation model list and store current drone position
unsigned int GazeboRosMeshQosPrivate::GetModelPositions() {
  unsigned int drone_count = 0;
  unsigned int count = world_->ModelCount();

  if (count > 0) {
    for(unsigned int i=0; i<count; i++) {
      gazebo::physics::ModelPtr m = world_->ModelByIndex(i);
      if (m) {
        std::string name = m->GetName();
        if (name.compare((size_t) 0, (size_t) name_prefix_.length(), name_prefix_.c_str()) == 0) {
          name.erase(0, name_prefix_.length());
          ++drone_count;
          ignition::math::Pose3d pos = m->WorldPose();

          if (drones_.find(name) == drones_.end()) {
            // item not found, create publisher
            std::string topic = "/" + name + "/mesh_qos";
            DroneInfo info;
            info.x = pos.Pos()[0];
            info.y = pos.Pos()[1];
            info.z = pos.Pos()[2];
            //info.pub = ros_node_->create_publisher<fog_msgs::msg::MeshQos>(
            info.pub = ros_node_->create_publisher<std_msgs::msg::String>(
                topic, rclcpp::SensorDataQoS() );
            drones_[name] = info;
          } else {
            drones_[name].x = pos.Pos()[0];
            drones_[name].y = pos.Pos()[1];
            drones_[name].z = pos.Pos()[2];
          }
        }
      }
    }
  }
  return drone_count;
}

// Generate mesh link quality for each drone-to-drone connection
//  according to the distances. Send the link quiality data
void GazeboRosMeshQosPrivate::GenerateMeshQuality() {
  for ( auto const& me : drones_) {
    auto message = std_msgs::msg::String();
    for ( auto const& target : drones_) {
      // Skip in case the target is the origin drone itself
      if ( me.first.compare(target.first) != 0 ) {

        double dist = std::sqrt(
          (target.second.x - me.second.x) * (target.second.x - me.second.x) +
          (target.second.y - me.second.y) * (target.second.y - me.second.y) +
          (target.second.z - me.second.z) * (target.second.z - me.second.z)
        );

        // Initially, very simple linear loss percentage..
        int loss = 0;
        if (dist > loss_start_dist_meters_) {
          loss = 100 - (int)100 * (
            (max_distance_meters_ - dist) /
            (max_distance_meters_ - loss_start_dist_meters_)
          );
        }
        if (loss > 100) {
          loss = 100;
        }
        message.data += target.first + ": " + std::to_string(loss) + "\n";
      }
    }
    me.second.pub->publish(message);

  }
}


// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosMeshQos)
}  // namespace gazebo_plugins


