#include "elp-camera-ros2-driver/elp_camera_node.hpp"
#include "elp-camera-ros2-driver/elp_camera_core.hpp"

namespace elp_camera {

ElpCameraNode::ElpCameraNode(const rclcpp::NodeOptions &options)
    : Node("elp_camera_node", options) {
  // Declare parameters
  this->declare_parameter("device_id", 0);
  this->declare_parameter("width", 2560);
  this->declare_parameter("height", 720);
  this->declare_parameter("fps", 30.0);
  this->declare_parameter("pixel_format", "MJPG");
  this->declare_parameter("frame_id", "camera_link");
  this->declare_parameter("left_frame_id", "camera_infra1_optical_frame");
  this->declare_parameter("right_frame_id", "camera_infra2_optical_frame");
  this->declare_parameter("publish_frame_alias_tf", true);
  this->declare_parameter("left_frame_alias_parent",
                          "camera_link_left_optical_frame");
  this->declare_parameter("right_frame_alias_parent",
                          "camera_link_right_optical_frame");
  this->declare_parameter("camera_info_url_left", "");
  this->declare_parameter("camera_info_url_right", "");

  // Get parameters
  int device_id = this->get_parameter("device_id").as_int();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double fps = this->get_parameter("fps").as_double();
  std::string pixel_format = this->get_parameter("pixel_format").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  left_frame_id_ = this->get_parameter("left_frame_id").as_string();
  right_frame_id_ = this->get_parameter("right_frame_id").as_string();
  publish_frame_alias_tf_ =
      this->get_parameter("publish_frame_alias_tf").as_bool();
  left_frame_alias_parent_ =
      this->get_parameter("left_frame_alias_parent").as_string();
  right_frame_alias_parent_ =
      this->get_parameter("right_frame_alias_parent").as_string();
  std::string camera_info_url_left =
      this->get_parameter("camera_info_url_left").as_string();
  std::string camera_info_url_right =
      this->get_parameter("camera_info_url_right").as_string();

  if (left_frame_id_.empty()) {
    left_frame_id_ = frame_id_;
  }
  if (right_frame_id_.empty()) {
    right_frame_id_ = frame_id_;
  }

  // Initialize Core
  core_ = new ElpCameraCore();
  if (!core_->setup(device_id, width, height, fps, pixel_format)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup ELP Camera Core");
    rclcpp::shutdown();
    return;
  }

  // Initialize Camera Info Managers
  cinfo_manager_left_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(
          this, "left_camera", camera_info_url_left);
  cinfo_manager_right_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(
          this, "right_camera", camera_info_url_right);

  // Initialize Publishers
  pub_left_ =
      image_transport::create_camera_publisher(this, "camera/left/image_raw");
  pub_right_ =
      image_transport::create_camera_publisher(this, "camera/right/image_raw");

  if (publish_frame_alias_tf_) {
    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto publish_alias = [&](const std::string &parent, const std::string &child,
                             const char *side_label) {
      if (parent.empty() || child.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Skip %s frame alias TF because parent/child frame is "
                    "empty (parent='%s', child='%s')",
                    side_label, parent.c_str(), child.c_str());
        return;
      }

      if (parent == child) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Skip %s frame alias TF because parent and child are the "
                     "same frame ('%s')",
                     side_label, parent.c_str());
        return;
      }

      geometry_msgs::msg::TransformStamped alias_tf;
      alias_tf.header.stamp = this->now();
      alias_tf.header.frame_id = parent;
      alias_tf.child_frame_id = child;
      alias_tf.transform.translation.x = 0.0;
      alias_tf.transform.translation.y = 0.0;
      alias_tf.transform.translation.z = 0.0;
      alias_tf.transform.rotation.x = 0.0;
      alias_tf.transform.rotation.y = 0.0;
      alias_tf.transform.rotation.z = 0.0;
      alias_tf.transform.rotation.w = 1.0;
      static_tf_broadcaster_->sendTransform(alias_tf);
    };

    publish_alias(left_frame_alias_parent_, left_frame_id_, "left");
    publish_alias(right_frame_alias_parent_, right_frame_id_, "right");
  }

  // Initialize Timer
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
  timer_ = this->create_wall_timer(
      period, std::bind(&ElpCameraNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "ELP Camera Node Initialized (Device: %d, %dx%d @ %.1f fps, "
              "Pixel Format: %s, Left Frame: %s, Right Frame: %s)",
              device_id, width, height, fps, pixel_format.c_str(),
              left_frame_id_.c_str(), right_frame_id_.c_str());
}

ElpCameraNode::~ElpCameraNode() {
  core_->release();
  delete core_;
}

void ElpCameraNode::timer_callback() {
  cv::Mat left_frame, right_frame;
  if (!core_->capture(left_frame, right_frame)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to capture frame");
    return;
  }

  // Create per-camera headers.
  const auto stamp = this->now();
  std_msgs::msg::Header header_left;
  header_left.stamp = stamp;
  header_left.frame_id = left_frame_id_;

  std_msgs::msg::Header header_right;
  header_right.stamp = stamp;
  header_right.frame_id = right_frame_id_;

  // Convert to ROS Message
  sensor_msgs::msg::Image::SharedPtr msg_left =
      cv_bridge::CvImage(header_left, "bgr8", left_frame).toImageMsg();

  sensor_msgs::msg::Image::SharedPtr msg_right =
      cv_bridge::CvImage(header_right, "bgr8", right_frame).toImageMsg();

  // Get Camera Info
  sensor_msgs::msg::CameraInfo ci_left = cinfo_manager_left_->getCameraInfo();
  ci_left.header = header_left;

  sensor_msgs::msg::CameraInfo ci_right = cinfo_manager_right_->getCameraInfo();
  ci_right.header = header_right;

  // Publish
  pub_left_.publish(*msg_left, ci_left);
  pub_right_.publish(*msg_right, ci_right);
}

} // namespace elp_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(elp_camera::ElpCameraNode)
