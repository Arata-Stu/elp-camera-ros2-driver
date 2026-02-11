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
  this->declare_parameter("frame_id", "camera_link");
  this->declare_parameter("camera_info_url_left", "");
  this->declare_parameter("camera_info_url_right", "");

  // Get parameters
  int device_id = this->get_parameter("device_id").as_int();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double fps = this->get_parameter("fps").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  std::string camera_info_url_left =
      this->get_parameter("camera_info_url_left").as_string();
  std::string camera_info_url_right =
      this->get_parameter("camera_info_url_right").as_string();

  // Initialize Core
  core_ = new ElpCameraCore();
  if (!core_->setup(device_id, width, height, fps)) {
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

  // Initialize Timer
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
  timer_ = this->create_wall_timer(
      period, std::bind(&ElpCameraNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "ELP Camera Node Initialized (Device: %d, %dx%d @ %.1f fps)",
              device_id, width, height, fps);
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

  // Create Header
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = frame_id_;

  // Convert to ROS Message
  sensor_msgs::msg::Image::SharedPtr msg_left =
      cv_bridge::CvImage(header, "bgr8", left_frame).toImageMsg();

  sensor_msgs::msg::Image::SharedPtr msg_right =
      cv_bridge::CvImage(header, "bgr8", right_frame).toImageMsg();

  // Get Camera Info
  sensor_msgs::msg::CameraInfo ci_left = cinfo_manager_left_->getCameraInfo();
  ci_left.header = header;

  sensor_msgs::msg::CameraInfo ci_right = cinfo_manager_right_->getCameraInfo();
  ci_right.header = header;

  // Publish
  pub_left_.publish(*msg_left, ci_left);
  pub_right_.publish(*msg_right, ci_right);
}

} // namespace elp_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(elp_camera::ElpCameraNode)
