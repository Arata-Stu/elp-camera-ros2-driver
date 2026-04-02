#ifndef ELP_CAMERA_NODE_HPP_
#define ELP_CAMERA_NODE_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace elp_camera {

class ElpCameraCore;

class ElpCameraNode : public rclcpp::Node {
public:
  explicit ElpCameraNode(const rclcpp::NodeOptions &options);
  ~ElpCameraNode();

private:
  void timer_callback();

  ElpCameraCore *core_;
  image_transport::CameraPublisher pub_left_;
  image_transport::CameraPublisher pub_right_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_left_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_right_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  bool publish_frame_alias_tf_;
  std::string left_frame_alias_parent_;
  std::string right_frame_alias_parent_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

} // namespace elp_camera

#endif // ELP_CAMERA_NODE_HPP_
