// Copyright 2013 Open Source Robotics Foundation
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

#include <sdf/Element.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/sensors/SensorTypes.hh>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo_plugins/gazebo_ros_camera.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#define FLOAT_SIZE sizeof(float)

namespace gazebo_plugins
{
template <class Func>
class defer {
public:
  defer(Func func) : func(std::move(func)), call(true) {}

  defer(const defer&) = delete;
  defer& operator=(const defer&) = delete;

  defer(defer&& other)
    : func(std::move(other.func)),
      call(other.call) {
    other.call = false;
  }
  defer& operator=(defer&& other) {
    func = std::move(other);
    call = other.call;
    other.call = false;
    return *this;
  }

  ~defer() {
    if (call) func();
  }

private:
 Func func;
 bool call;
};

template <class T>
void safeUnref(T*& ptr) {
  if (ptr) {
    gst_object_unref(ptr);
  }
}

class GazeboRosCameraPrivate
{
public:
  /// Indicates type of camera
  enum SensorType
  {
    /// Depth Camera
    DEPTH,

    /// Normal RGB Camera
    CAMERA,

    /// Multi Camera
    MULTICAMERA
  };

  /// Depth, Normal or Multi Camera
  SensorType sensor_type_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Image publishers.
  std::vector<image_transport::Publisher> image_pub_;

  /// Video streaming.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> video_pub_;
  std::vector<GMainLoop*> g_main_loop_;
  std::vector<GstElement*> gst_source_;
  std::mutex gst_mutex_;

  bool convFbImgToI420 = true;
  bool useCuda = false;
  bool useCudaCustomParams = false;
  bool useVaapi = false;
  std::string ros_namespace_;

  /// Camera info publishers.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pub_;

  /// Depth image publisher.
  image_transport::Publisher depth_image_pub_;

  /// Depth camera info publisher.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_{nullptr};

  /// Point cloud publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  /// Trigger subscriber, in case it's a triggered camera
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};

  /// Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_manager_;

  /// Image encodings
  std::vector<std::string> img_encoding_;

  /// Camera name, to be used on topics.
  std::string camera_name_;

  /// Frame name, to be used by TF.
  std::string frame_name_;

  /// Step sizes for fillImage
  std::vector<uint32_t> img_step_;

  /// Connects to pre-render events.
  gazebo::event::ConnectionPtr pre_render_connection_;

  /// Keeps track of how many times the camera has been triggered since it last published an image.
  int triggered{0};

  /// Protects trigger.
  std::mutex trigger_mutex_;

  /// Lock for image message
  std::mutex image_mutex_;

  /// Store current camera image.
  sensor_msgs::msg::Image image_msg_;

  /// Store current point cloud.
  sensor_msgs::msg::PointCloud2 cloud_msg_;

  /// Pointers to cameras
  std::vector<gazebo::rendering::CameraPtr> camera_;

  /// Horizontal FOV of cameras
  std::vector<double> hfov_;

  /// Min valid depth
  double min_depth_;

  /// Max valid depth
  double max_depth_;

  /// Number of cameras
  uint64_t num_cameras_{1};
};

GazeboRosCamera::GazeboRosCamera()
: impl_(std::make_unique<GazeboRosCameraPrivate>())
{
}

GazeboRosCamera::~GazeboRosCamera()
{
  for (auto pub : impl_->image_pub_) {
    pub.shutdown();
  }
  if (param_change_callback_handler_) {
    impl_->ros_node_->remove_on_set_parameters_callback(param_change_callback_handler_.get());
  }
  param_change_callback_handler_.reset();
}

void GazeboRosCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _sensor);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (std::dynamic_pointer_cast<gazebo::sensors::MultiCameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::MULTICAMERA;
    MultiCameraPlugin::Load(_sensor, _sdf);
    impl_->num_cameras_ = MultiCameraPlugin::parent_sensor_->CameraCount();
  } else if (std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::DEPTH;
    gazebo::DepthCameraPlugin::Load(_sensor, _sdf);
  } else if (std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::CAMERA;
    gazebo::CameraPlugin::Load(_sensor, _sdf);
  } else {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Plugin must be attached to sensor of type `camera`, `depth` or `multicamera`");
    impl_->ros_node_.reset();
    return;
  }

  // Camera name
  impl_->camera_name_ = _sdf->Get<std::string>("cameraName", _sensor->Name()).first;

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->ros_namespace_ = _sdf->Get<std::string>("rosNamespace", "").first;
  auto topic_prefix = impl_->ros_namespace_ + "/" + impl_->camera_name_;
  if (impl_->sensor_type_ != GazeboRosCameraPrivate::MULTICAMERA) {
    // Image publisher
    // TODO(louise) Migrate image_connect logic once SubscriberStatusCallback is ported to ROS2
    const std::string camera_topic = topic_prefix + "/image_raw";
    impl_->image_pub_.push_back(
      image_transport::create_publisher(
        impl_->ros_node_.get(), camera_topic, qos.get_publisher_qos(
          camera_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile()));

    // TODO(louise) Uncomment this once image_transport::Publisher has a function to return the
    // full topic.
    // RCLCPP_INFO(
    //   impl_->ros_node_->get_logger(), "Publishing images to [%s]", impl_->image_pub_.getTopic());

    // Video publisher
    const std::string video_topic = topic_prefix + "/color/video";
    impl_->video_pub_.push_back(
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        video_topic, qos.get_publisher_qos(
          video_topic, rclcpp::SensorDataQoS().reliable())));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing video to [%s]",
      impl_->video_pub_.back()->get_topic_name());

    // Camera info publisher
    // TODO(louise) Migrate ImageConnect logic once SubscriberStatusCallback is ported to ROS2
    const std::string camera_info_topic = topic_prefix + "/camera_info";
    impl_->camera_info_pub_.push_back(
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos.get_publisher_qos(
          camera_info_topic, rclcpp::SensorDataQoS().reliable())));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing camera info to [%s]",
      impl_->camera_info_pub_.back()->get_topic_name());

  } else {
    for (uint64_t i = 0; i < impl_->num_cameras_; ++i) {
      auto camera_name = MultiCameraPlugin::camera_[i]->Name();
      auto topic_prefix = impl_->ros_namespace_ + "/" + camera_name;
      auto camera_topic = topic_prefix + "/image_raw";
      // Image publisher
      impl_->image_pub_.push_back(
        image_transport::create_publisher(
          impl_->ros_node_.get(), camera_topic, qos.get_publisher_qos(
            camera_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile()));

      // RCLCPP_INFO(
      //   impl_->ros_node_->get_logger(), "Publishing %s camera images to [%s]",
      //   camera_name.c_str(),
      //   impl_->image_pub_.back().getTopic());

      // Video publisher
      auto video_topic = topic_prefix + "/video";
      impl_->video_pub_.push_back(
        impl_->ros_node_->create_publisher<sensor_msgs::msg::CompressedImage>(
          video_topic, qos.get_publisher_qos(
            video_topic, rclcpp::SensorDataQoS().reliable())));

      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Publishing %s camera video to [%s]",
        camera_name.c_str(),
        impl_->video_pub_[i]->get_topic_name());

      auto camera_info_topic = topic_prefix + "/camera_info";
      // Camera info publisher
      impl_->camera_info_pub_.push_back(
        impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          camera_info_topic, qos.get_publisher_qos(
            camera_info_topic, rclcpp::SensorDataQoS().reliable())));

      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Publishing %s camera info to [%s]",
        camera_name.c_str(),
        impl_->camera_info_pub_[i]->get_topic_name());
    }
  }

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    const std::string depth_topic = topic_prefix + "/depth/image_raw";
    // Depth image publisher
    impl_->depth_image_pub_ = image_transport::create_publisher(
      impl_->ros_node_.get(), depth_topic, qos.get_publisher_qos(
        depth_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile());

    // RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing depth images to [%s]",
    //   impl_->depth_image_pub_.getTopic().c_str());

    const std::string depth_info_topic = topic_prefix + "/depth/camera_info";
    // Depth info publisher
    impl_->depth_camera_info_pub_ =
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, qos.get_publisher_qos(depth_info_topic, rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing depth camera info to [%s]",
      impl_->depth_camera_info_pub_->get_topic_name());

    const std::string point_cloud_topic = topic_prefix + "/points";
    // Point cloud publisher
    impl_->point_cloud_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic, qos.get_publisher_qos(point_cloud_topic, rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing pointcloud to [%s]",
      impl_->point_cloud_pub_->get_topic_name());
  }

  // Trigger
  if (_sdf->Get<bool>("triggered", false).first) {
    const std::string trigger_topic = topic_prefix + "/image_trigger";
    impl_->trigger_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
      trigger_topic, qos.get_subscription_qos(trigger_topic, rclcpp::QoS(1)),
      std::bind(&GazeboRosCamera::OnTrigger, this, std::placeholders::_1));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Subscribed to [%s]",
      impl_->trigger_sub_->get_topic_name());

    SetCameraEnabled(false);
    impl_->pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&GazeboRosCamera::PreRender, this));
  }

  // Buffer size
  std::vector<std::string> image_format;
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    image_format.push_back(CameraPlugin::format);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    image_format.push_back(DepthCameraPlugin::format);
  } else {
    image_format = MultiCameraPlugin::format_;
  }

  for (const auto & format : image_format) {
    if (format == "L8" || format == "L_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::MONO8);
      impl_->img_step_.push_back(1);
    } else if (format == "L16" || format == "L_INT16") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::MONO16);
      impl_->img_step_.push_back(2);
    } else if (format == "R8G8B8" || format == "RGB_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::RGB8);
      impl_->img_step_.push_back(3);
    } else if (format == "B8G8R8" || format == "BGR_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BGR8);
      impl_->img_step_.push_back(3);
    } else if (format == "R16G16B16" || format == "RGB_INT16") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::RGB16);
      impl_->img_step_.push_back(6);
    } else if (format == "BAYER_RGGB8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_RGGB8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_BGGR8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_BGGR8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GBRG8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GBRG8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GRBG8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GRBG8);
      impl_->img_step_.push_back(1);
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BGR8);
      impl_->img_step_.push_back(3);
    }
  }

  std::vector<unsigned int> width, height;
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    width.push_back(CameraPlugin::width);
    height.push_back(CameraPlugin::height);
    impl_->camera_.push_back(CameraPlugin::camera);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    width.push_back(DepthCameraPlugin::width);
    height.push_back(DepthCameraPlugin::height);
    impl_->camera_.emplace_back(DepthCameraPlugin::depthCamera);
  } else {
    width = MultiCameraPlugin::width_;
    height = MultiCameraPlugin::height_;
    impl_->camera_ = MultiCameraPlugin::camera_;
  }

  for (uint64_t i = 0; i < impl_->num_cameras_; ++i) {
    // C parameters
    auto default_cx = (static_cast<double>(width[i]) + 1.0) / 2.0;
    auto cx = _sdf->Get<double>("cx", default_cx).first;

    auto default_cy = (static_cast<double>(height[i]) + 1.0) / 2.0;
    auto cy = _sdf->Get<double>("cy", default_cy).first;

    impl_->hfov_.push_back(impl_->camera_[i]->HFOV().Radian());

    double computed_focal_length =
      (static_cast<double>(width[i])) / (2.0 * tan(impl_->hfov_[i] / 2.0));

    // Focal length
    auto focal_length = _sdf->Get<double>("focal_length", 0.0).first;
    if (focal_length == 0) {
      focal_length = computed_focal_length;
    } else if (!ignition::math::equal(focal_length, computed_focal_length)) {
      RCLCPP_WARN(
        impl_->ros_node_->get_logger(),
        "The <focal_length> [%f] you have provided for camera [%s]"
        " is inconsistent with specified <image_width> [%d] and"
        " HFOV [%f]. Please double check to see that"
        " focal_length = width / (2.0 * tan(HFOV/2.0))."
        " The expected focal_length value is [%f],"
        " please update your camera model description accordingly.",
        focal_length, _sensor->Name().c_str(), width[i], impl_->hfov_[i], computed_focal_length);
    }

    // CameraInfo
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.frame_id = impl_->frame_name_;
    camera_info_msg.height = height[i];
    camera_info_msg.width = width[i];
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.d.resize(5);

    // Allow the user to disable automatic cropping (used to remove barrel
    // distortion black border. The crop can be useful, but also skewes
    // the lens distortion, making the supplied k and t values incorrect.
    auto border_crop = _sdf->Get<bool>("border_crop", true).first;
    auto hack_baseline = _sdf->Get<double>("hack_baseline", 0.0).first;

    // Get distortion from camera
    double distortion_k1{0.0};
    double distortion_k2{0.0};
    double distortion_k3{0.0};
    double distortion_t1{0.0};
    double distortion_t2{0.0};
    if (impl_->camera_[i]->LensDistortion()) {
      impl_->camera_[i]->LensDistortion()->SetCrop(border_crop);

      distortion_k1 = impl_->camera_[i]->LensDistortion()->K1();
      distortion_k2 = impl_->camera_[i]->LensDistortion()->K2();
      distortion_k3 = impl_->camera_[i]->LensDistortion()->K3();
      distortion_t1 = impl_->camera_[i]->LensDistortion()->P1();
      distortion_t2 = impl_->camera_[i]->LensDistortion()->P2();
    }

    // D = {k1, k2, t1, t2, k3}, as specified in:
    // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    camera_info_msg.d[0] = distortion_k1;
    camera_info_msg.d[1] = distortion_k2;
    camera_info_msg.d[2] = distortion_t1;
    camera_info_msg.d[3] = distortion_t2;
    camera_info_msg.d[4] = distortion_k3;

    // Original camera matrix
    camera_info_msg.k[0] = focal_length;
    camera_info_msg.k[1] = 0.0;
    camera_info_msg.k[2] = cx;
    camera_info_msg.k[3] = 0.0;
    camera_info_msg.k[4] = focal_length;
    camera_info_msg.k[5] = cy;
    camera_info_msg.k[6] = 0.0;
    camera_info_msg.k[7] = 0.0;
    camera_info_msg.k[8] = 1.0;

    // rectification
    camera_info_msg.r[0] = 1.0;
    camera_info_msg.r[1] = 0.0;
    camera_info_msg.r[2] = 0.0;
    camera_info_msg.r[3] = 0.0;
    camera_info_msg.r[4] = 1.0;
    camera_info_msg.r[5] = 0.0;
    camera_info_msg.r[6] = 0.0;
    camera_info_msg.r[7] = 0.0;
    camera_info_msg.r[8] = 1.0;

    // camera_ projection matrix (same as camera_ matrix due
    // to lack of distortion/rectification) (is this generated?)
    camera_info_msg.p[0] = focal_length;
    camera_info_msg.p[1] = 0.0;
    camera_info_msg.p[2] = cx;
    camera_info_msg.p[3] = -focal_length * hack_baseline;
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = focal_length;
    camera_info_msg.p[6] = cy;
    camera_info_msg.p[7] = 0.0;
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0;

    // Initialize camera_info_manager
    impl_->camera_info_manager_.push_back(
      std::make_shared<camera_info_manager::CameraInfoManager>(
        impl_->ros_node_.get(), impl_->camera_name_));
    impl_->camera_info_manager_.back()->setCameraInfo(camera_info_msg);
  }

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    impl_->min_depth_ = _sdf->Get<double>("min_depth", 0.4).first;
    impl_->max_depth_ =
      _sdf->Get<double>("max_depth", std::numeric_limits<float>::infinity()).first;

    // Initialize point cloud message
    impl_->cloud_msg_.fields.resize(4);
    impl_->cloud_msg_.fields[0].name = "x";
    impl_->cloud_msg_.fields[0].offset = 0;
    impl_->cloud_msg_.fields[0].datatype = 7;
    impl_->cloud_msg_.fields[0].count = 1;
    impl_->cloud_msg_.fields[1].name = "y";
    impl_->cloud_msg_.fields[1].offset = 4;
    impl_->cloud_msg_.fields[1].datatype = 7;
    impl_->cloud_msg_.fields[1].count = 1;
    impl_->cloud_msg_.fields[2].name = "z";
    impl_->cloud_msg_.fields[2].offset = 8;
    impl_->cloud_msg_.fields[2].datatype = 7;
    impl_->cloud_msg_.fields[2].count = 1;
    impl_->cloud_msg_.fields[3].name = "rgb";
    impl_->cloud_msg_.fields[3].offset = 16;
    impl_->cloud_msg_.fields[3].datatype = 7;
    impl_->cloud_msg_.fields[3].count = 1;

    impl_->cloud_msg_.header.frame_id = impl_->frame_name_;
  }

  // Dynamic reconfigure
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    impl_->ros_node_->declare_parameter(
      "update_rate", gazebo::CameraPlugin::parentSensor->UpdateRate());
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    impl_->ros_node_->declare_parameter(
      "update_rate", gazebo::DepthCameraPlugin::parentSensor->UpdateRate());
  } else {
    impl_->ros_node_->declare_parameter(
      "update_rate", MultiCameraPlugin::parent_sensor_->UpdateRate());
  }

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();
        if (param_name == "update_rate") {
          if (nullptr != impl_->trigger_sub_) {
            RCLCPP_WARN(
              impl_->ros_node_->get_logger(),
              "Cannot set update rate for triggered camera");
            result.successful = false;
          } else {
            rclcpp::ParameterType parameter_type = parameter.get_type();
            if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
              double rate = parameter.as_double();

              if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
                gazebo::CameraPlugin::parentSensor->SetUpdateRate(rate);
              } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
                gazebo::DepthCameraPlugin::parentSensor->SetUpdateRate(rate);
              } else {
                MultiCameraPlugin::parent_sensor_->SetUpdateRate(rate);
              }

              if (rate >= 0.0) {
                RCLCPP_INFO(
                  impl_->ros_node_->get_logger(),
                  "Camera update rate changed to [%.2f Hz]", rate);
              } else {
                RCLCPP_WARN(
                  impl_->ros_node_->get_logger(),
                  "Camera update rate should be positive. Setting to maximum");
              }
            } else {
              RCLCPP_WARN(
                impl_->ros_node_->get_logger(),
                "Value for param [update_rate] has to be of double type.");
              result.successful = false;
            }
          }
        }
      }
      return result;
    };

  param_change_callback_handler_ =
    impl_->ros_node_->add_on_set_parameters_callback(param_change_callback);

  gst_init(nullptr, nullptr);
  impl_->convFbImgToI420 = _sdf->Get<bool>("convFbImgToI420", true).first;
  impl_->useCuda = _sdf->Get<bool>("useCuda", false).first;
  impl_->useCudaCustomParams = _sdf->Get<bool>("useCudaCustomParams", false).first;
  impl_->useVaapi = _sdf->Get<bool>("useVaapi", false).first;
  for (uint64_t i = 0; i < impl_->num_cameras_; ++i) {
    std::thread thread{
      &GazeboRosCamera::startGstPipeline,
      this
    };
    thread.detach();
  }
}

void GazeboRosCamera::startGstPipeline() {
  size_t camera_num;
  {
    std::lock_guard<std::mutex> gst_lock(impl_->gst_mutex_);
    camera_num = impl_->g_main_loop_.size();
    impl_->g_main_loop_.push_back(nullptr);
    impl_->gst_source_.push_back(nullptr);
  }
  auto main_loop = g_main_loop_new(nullptr, FALSE);
  if (!main_loop) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create main loop"
    );
    return;
  }
  defer _0{[&] { g_main_loop_unref(main_loop); }};

  GstElement* pipeline = gst_pipeline_new(nullptr);
  if (!pipeline) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create pipeline"
    );
    return;
  }
  defer _1{[&] { gst_object_unref(pipeline); }};

  GstElement* source = gst_element_factory_make("appsrc", nullptr);
  if (!source) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create source"
    );
    return;
  }
  defer _2{[&] { gst_object_unref(source); }};
  GString *appSrcFormat = g_string_new("I420");
  if (!impl_->convFbImgToI420) {
    appSrcFormat = g_string_assign(appSrcFormat, "RGB");
  }

  auto image_width = impl_->camera_[camera_num]->ImageWidth();
  auto image_height = impl_->camera_[camera_num]->ImageHeight();
  auto render_rate = impl_->camera_[camera_num]->RenderRate();
  if (!isfinite(render_rate)) {
    render_rate = 60.0;
  }
  g_object_set(G_OBJECT(source), "caps",
      gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, appSrcFormat->str,
        "width", G_TYPE_INT, image_width,
        "height", G_TYPE_INT, image_height,
        "framerate", GST_TYPE_FRACTION, (unsigned int)render_rate, 1, nullptr),
      "is-live", TRUE,
      "do-timestamp", TRUE,
      "stream-type", GST_APP_STREAM_TYPE_STREAM,
      "format", GST_FORMAT_TIME, nullptr);
  g_string_free(appSrcFormat, false);

  GstElement* queue = gst_element_factory_make("queue", nullptr);
  if (!queue) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create queue"
    );
    return;
  }
  defer _3{[&] { gst_object_unref(queue); }};

  GstElement* converter  = gst_element_factory_make("videoconvert", nullptr);
  if (!converter) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create converter"
    );
    return;
  }
  defer _4{[&] { gst_object_unref(converter); }};

  GstElement* filterNV12;
  defer _5{[&] { safeUnref(filterNV12); }};
  GstElement* encoder = nullptr;
  defer _6{[&] { safeUnref(encoder); }};

  if (impl_->useCuda) {
    encoder = gst_element_factory_make("nvh264enc", nullptr);
    if (impl_->useCudaCustomParams) {
      g_object_set(G_OBJECT(encoder), "bitrate", 2000, nullptr);
      // rc-mode: 0 = default; 1 = constqp; 2 = cbr; 3 = vbr; 4 = vbr-minqp
      g_object_set(G_OBJECT(encoder), "rc-mode", 2, nullptr);
      g_object_set(G_OBJECT(encoder), "qos", true, nullptr);
      g_object_set(G_OBJECT(encoder), "gop-size", 60, nullptr);
    } else {
      g_object_set(G_OBJECT(encoder), "bitrate", 800, nullptr);
      g_object_set(G_OBJECT(encoder), "preset", 1, nullptr); //lower = faster, 6=medium
    }
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Use nvh264enc."
    );
  } else if (impl_->useVaapi) {
    encoder = gst_element_factory_make("vaapih264enc", "AvcEncoder");
    g_object_set(G_OBJECT(encoder), "bitrate", 2000, nullptr);
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Use vaapih264enc"
    );
    filterNV12 = gst_element_factory_make("capsfilter", "FilterNV12");
    if (!filterNV12) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Failed to create NV12 filter element"
      );
      return;
    }
    g_object_set(G_OBJECT(filterNV12), "caps",
        gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width", G_TYPE_INT, image_width,
        "height", G_TYPE_INT, image_height,
        "framerate", GST_TYPE_FRACTION, (unsigned int)render_rate, 1,
        nullptr),
        nullptr);
  } else {
    encoder = gst_element_factory_make("x264enc", nullptr);
    g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6, "tune", 4, "key-int-max", 10, nullptr);
  }
  if (!encoder) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create encoder"
    );
    return;
  }

  GstElement* filterH264 = gst_element_factory_make("capsfilter", "FilterH264");
  if (!filterH264) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to create H264 filter element"
    );
    return;
  }
  defer _7{[&] { gst_object_unref(filterH264); }};
  g_object_set(G_OBJECT(filterH264), "caps",
      gst_caps_new_simple ("video/x-h264",
      "profile", G_TYPE_STRING, "main",
      "stream-format", G_TYPE_STRING, "byte-stream",
      nullptr),
      nullptr);

  GstElement* sink = gst_element_factory_make("appsink", nullptr);
  if (!sink) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "H264 filter element failed."
    );
    return;
  }
  defer _8{[&] { gst_object_unref(sink); }};

  // Connect all elements to pipeline
  if (impl_->useVaapi) {
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, filterNV12, encoder, filterH264, sink, nullptr);
  } else {
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder, filterH264, sink, nullptr);
  }

  // Link all elements
  gboolean link_ok;
  if (impl_->useVaapi) {
    link_ok = gst_element_link_many(source, queue, converter, filterNV12, encoder, filterH264, sink, nullptr);
  } else {
    link_ok = gst_element_link_many(source, queue, converter, encoder, filterH264, sink, nullptr);
  }
  if (!link_ok) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Failed to link elements"
    );
    return;
  }

  impl_->g_main_loop_[camera_num] = main_loop;
  g_main_loop_ref(main_loop);
  impl_->gst_source_[camera_num] = source;
  gst_object_ref(source);
  defer _9{[&] {
    std::lock_guard<std::mutex> gst_lock(impl_->gst_mutex_);
    g_main_loop_unref(main_loop);
    impl_->g_main_loop_[camera_num] = nullptr;
    gst_object_unref(source);
    impl_->gst_source_[camera_num] = nullptr;
  }};

  std::thread sink_thread{[&] {
    while (publishVideoFrame(sink, camera_num)) {}
  }};
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(main_loop);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  sink_thread.join();
}

void GazeboRosCamera::NewFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  const int _camera_num)
{
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  gazebo::common::Time sensor_update_time;

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    sensor_update_time = CameraPlugin::parentSensor->LastMeasurementTime();
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    sensor_update_time = DepthCameraPlugin::parentSensor->LastMeasurementTime();
  } else {
    sensor_update_time = MultiCameraPlugin::parent_sensor_->LastMeasurementTime();
  }
  auto sensor_update_time_ros =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_update_time);

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_[_camera_num]->getCameraInfo();
  camera_info_msg.header.stamp = sensor_update_time_ros;

  impl_->camera_info_pub_[_camera_num]->publish(camera_info_msg);

  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  // Publish image
  impl_->image_msg_.header.frame_id = impl_->frame_name_;
  impl_->image_msg_.header.stamp = sensor_update_time_ros;

  // Copy from src to image_msg
  sensor_msgs::fillImage(
    impl_->image_msg_, impl_->img_encoding_[_camera_num], _height, _width,
    impl_->img_step_[_camera_num] * _width, reinterpret_cast<const void *>(_image));

  impl_->image_pub_[_camera_num].publish(impl_->image_msg_);

  pushToGstPipeline(
    _image,
    _width,
    _height,
    _camera_num
  );

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);

    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

void GazeboRosCamera::pushToGstPipeline(
  const unsigned char * image,
  unsigned int width,
  unsigned int height,
  const int camera_num
) {
  guint size = width * height * 1.5;
  if (!impl_->convFbImgToI420) {
    size = width * height * 3;
  }
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

  if (!buffer) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "gst_buffer_new_allocate failed"
    );
    return;
  }

  GstMapInfo map;

  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "gst_buffer_map failed"
    );
    return;
  }

  if (impl_->convFbImgToI420) {
    using namespace cv;
	  // Color Conversion from RGB to YUV
    Mat frame = Mat(height, width, CV_8UC3);
    Mat frameYUV = Mat(height, width, CV_8UC3);
    // frame.data = image;
    frame.data = (uchar*)image;
    cvtColor(frame, frameYUV, COLOR_RGB2YUV_I420);
    memcpy(map.data, frameYUV.data, size);
  } else {
	  memcpy(map.data, image, size);
	  // memcpy(map.data, (uchar *)image, size);
  }
  gst_buffer_unmap(buffer, &map);

  std::lock_guard<std::mutex> gst_lock{impl_->gst_mutex_};
  auto source = impl_->gst_source_[camera_num];
  if (!source) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Gstreamer source has been closed"
    );
    return;
  }
  GstFlowReturn ret = gst_app_src_push_buffer(
    GST_APP_SRC(source),
    buffer
  );

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "gst_app_src_push_buffer failed"
    );
    g_main_loop_quit(impl_->g_main_loop_[camera_num]);
  }
}

bool GazeboRosCamera::publishVideoFrame(
  GstElement* sink,
  size_t camera_num
) {
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
  if (!sample) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "end of stream was reached"
    );
    return true;
  }
  defer _{[&] { gst_sample_unref(sample); }};
  GstBuffer* buf = gst_sample_get_buffer(sample);
  if (!buf) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "sample contains no data"
    );
    return true;
  }
  sensor_msgs::msg::CompressedImage frame;
  frame.header.frame_id = buf->offset;
  frame.header.stamp = impl_->ros_node_->get_clock()->now();
  frame.format = "H264";
  frame.data.resize(gst_buffer_get_size(buf), 0);
  gst_buffer_extract(buf, 0, frame.data.data(), frame.data.size());
  impl_->video_pub_[camera_num]->publish(frame);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosCamera::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  NewFrame(_image, _width, _height, 0);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewImageFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
  NewFrame(_image, _width, _height, 0);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewDepthFrame(
  const float * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosCamera::OnNewDepthFrame");
  IGN_PROFILE_BEGIN("fill ROS depth message");
#endif
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  auto sensor_update_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    DepthCameraPlugin::parentSensor->LastMeasurementTime());

  // Publish depth image
  sensor_msgs::msg::Image image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.header.frame_id = impl_->frame_name_;
  image_msg.header.stamp = sensor_update_time;
  image_msg.width = _width;
  image_msg.height = _height;
  image_msg.step = FLOAT_SIZE * _width;
  image_msg.data.resize(_width * _height * FLOAT_SIZE);
  image_msg.is_bigendian = 0;

  int index = 0;

  float pos_inf = std::numeric_limits<float>::infinity();
  float neg_inf = -pos_inf;

  // Copy from src to image_msg
  for (uint32_t j = 0; j < _height; j++) {
    for (uint32_t i = 0; i < _width; i++) {
      index = i + j * _width;
      float depth = _image[index];
      if (impl_->min_depth_ < depth && depth < impl_->max_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &depth, FLOAT_SIZE);
      } else if (depth <= impl_->min_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &neg_inf, FLOAT_SIZE);
      } else {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &pos_inf, FLOAT_SIZE);
      }
    }
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish depth image");
#endif
  impl_->depth_image_pub_.publish(image_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_[0]->getCameraInfo();
  camera_info_msg.header.stamp = sensor_update_time;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("publish camera depth info");
#endif
  impl_->depth_camera_info_pub_->publish(camera_info_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("fill ROS cloud point message");
#endif
  // Publish point cloud
  impl_->cloud_msg_.header.stamp = sensor_update_time;
  impl_->cloud_msg_.width = _width;
  impl_->cloud_msg_.height = _height;
  impl_->cloud_msg_.row_step = impl_->cloud_msg_.point_step * _width * _height;

  sensor_msgs::PointCloud2Modifier cloud_modifier(impl_->cloud_msg_);
  cloud_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  cloud_modifier.resize(_width * _height);

  impl_->cloud_msg_.is_dense = true;

  int image_index = 0;
  int cloud_index = 0;

  double fl = (static_cast<double>(_width)) / (2.0 * tan(impl_->hfov_[0] / 2.0));

  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  for (uint32_t j = 0; j < _height; j++) {
    double pAngle;
    if (_height > 1) {
      pAngle = atan2(static_cast<double>(j) - 0.5 * static_cast<double>(_height - 1), fl);
    } else {
      pAngle = 0.0;
    }

    for (uint32_t i = 0; i < _width; ++i) {
      double yAngle;
      if (_width > 1) {
        yAngle = atan2(static_cast<double>(i) - 0.5 * static_cast<double>(_width - 1), fl);
      } else {
        yAngle = 0.0;
      }

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame

      float depth = _image[image_index++];

      if (depth > impl_->min_depth_ && depth < impl_->max_depth_) {
        auto x = static_cast<float>(depth * tan(yAngle));
        auto y = static_cast<float>(depth * tan(pAngle));

        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &x, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &y, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &depth, FLOAT_SIZE);
      } else if (depth <= impl_->min_depth_) {
        // point before valid range
        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &neg_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &neg_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &neg_inf, FLOAT_SIZE);
        impl_->cloud_msg_.is_dense = false;
      } else {
        // point after valid range
        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &pos_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &pos_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &pos_inf, FLOAT_SIZE);
        impl_->cloud_msg_.is_dense = false;
      }

      if (impl_->image_msg_.data.size() == _width * _height * 3) {
        // color
        std::memcpy(
          &impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[(i + j * _width) * 3], 3 * sizeof(uint8_t));
      } else if (impl_->image_msg_.data.size() == _height * _width) {
        std::memcpy(
          &impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[i + j * _width], 3 * sizeof(uint8_t));
      } else {
        // no image
        std::memset(&impl_->cloud_msg_.data[cloud_index + 16], 0, 3 * sizeof(uint8_t));
      }
      cloud_index += impl_->cloud_msg_.point_step;
    }
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("publish cloud point");
#endif
  impl_->point_cloud_pub_->publish(impl_->cloud_msg_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);
    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewMultiFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/,
  const int _camera_num)
{
  NewFrame(_image, _width, _height, _camera_num);
}

void GazeboRosCamera::SetCameraEnabled(const bool _enabled)
{
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    CameraPlugin::parentSensor->SetActive(_enabled);
    CameraPlugin::parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    DepthCameraPlugin::parentSensor->SetActive(_enabled);
    DepthCameraPlugin::parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  } else {
    MultiCameraPlugin::parent_sensor_->SetActive(_enabled);
    MultiCameraPlugin::parent_sensor_->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  }
}

void GazeboRosCamera::PreRender()
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  if (impl_->triggered > 0) {
    SetCameraEnabled(true);
  }
}

void GazeboRosCamera::OnTrigger(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  impl_->triggered++;
}

std::vector<gazebo::rendering::CameraPtr> GazeboRosCamera::GetCameras() const
{
  return impl_->camera_;
}

std::string GazeboRosCamera::GetCameraName() const
{
  return impl_->camera_name_;
}

uint64_t GazeboRosCamera::GetNumCameras() const
{
  return impl_->num_cameras_;
}

extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
gazebo::SensorPlugin * RegisterPlugin()
{
  return (gazebo::CameraPlugin *)(new GazeboRosCamera());
}
}  // namespace gazebo_plugins
