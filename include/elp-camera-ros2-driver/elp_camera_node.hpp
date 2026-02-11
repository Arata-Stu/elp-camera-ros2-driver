#ifndef ELP_CAMERA_NODE_HPP_
#define ELP_CAMERA_NODE_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

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
};

} // namespace elp_camera

#endif // ELP_CAMERA_NODE_HPP_
