#ifndef ELP_CAMERA_CORE_HPP_
#define ELP_CAMERA_CORE_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace elp_camera {

class ElpCameraCore {
public:
  ElpCameraCore();
  ~ElpCameraCore();

  bool setup(int device_id, int width, int height, double fps,
             const std::string &pixel_format);
  void release();
  bool capture(cv::Mat &left_frame, cv::Mat &right_frame);

private:
  cv::VideoCapture cap_;
  int width_;
  int height_;
};

} // namespace elp_camera

#endif // ELP_CAMERA_CORE_HPP_
