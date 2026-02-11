#include "elp-camera-ros2-driver/elp_camera_core.hpp"
#include <iostream>

namespace elp_camera {

ElpCameraCore::ElpCameraCore() : width_(0), height_(0) {}

ElpCameraCore::~ElpCameraCore() { release(); }

bool ElpCameraCore::setup(int device_id, int width, int height, double fps) {
  cap_.open(device_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    std::cerr << "Failed to open camera device: " << device_id << std::endl;
    return false;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS, fps);

  // Read back to verify
  double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

  if (static_cast<int>(actual_width) != width ||
      static_cast<int>(actual_height) != height) {
    std::cerr << "Warning: Requested resolution " << width << "x" << height
              << " but got " << actual_width << "x" << actual_height
              << std::endl;
    // We accept it, but splitting might need adjustment if logic depended
    // heavily on exact match. For now, we update our stored width/height to
    // match reality for splitting logic.
    width_ = static_cast<int>(actual_width);
    height_ = static_cast<int>(actual_height);
  } else {
    width_ = width;
    height_ = height;
  }

  return true;
}

void ElpCameraCore::release() {
  if (cap_.isOpened()) {
    cap_.release();
  }
}

bool ElpCameraCore::capture(cv::Mat &left_frame, cv::Mat &right_frame) {
  if (!cap_.isOpened()) {
    return false;
  }

  cv::Mat raw_frame;
  if (!cap_.read(raw_frame)) {
    return false;
  }

  if (raw_frame.empty()) {
    return false;
  }

  // Split the image
  // Assuming side-by-side stereo
  int half_width = width_ / 2;

  // Create ROI (Region of Interest) - this creates headers without copying data
  // Left image is 0 to width/2
  cv::Rect left_roi(0, 0, half_width, height_);
  // Right image is width/2 to width
  cv::Rect right_roi(half_width, 0, half_width, height_);

  // Copy data to output frames to ensure they are continuous and independent if
  // needed Or just return the ROI header if the caller handles it. However,
  // returning ROI header keeps reference to 'raw_frame'. 'raw_frame' is local,
  // so we MUST copy.

  // Efficiently copy the sub-regions
  raw_frame(left_roi).copyTo(left_frame);
  raw_frame(right_roi).copyTo(right_frame);

  return true;
}

} // namespace elp_camera
