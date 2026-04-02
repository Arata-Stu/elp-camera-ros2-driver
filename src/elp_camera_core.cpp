#include "elp-camera-ros2-driver/elp_camera_core.hpp"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

namespace {

std::string normalize_pixel_format(std::string pixel_format) {
  std::transform(pixel_format.begin(), pixel_format.end(), pixel_format.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return pixel_format;
}

int fourcc_from_pixel_format(const std::string &pixel_format) {
  if (pixel_format == "MJPG") {
    return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  }

  if (pixel_format == "YUYV") {
    return cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
  }

  return 0;
}

std::string fourcc_to_string(int fourcc) {
  std::string value;
  value += static_cast<char>(fourcc & 0xFF);
  value += static_cast<char>((fourcc >> 8) & 0xFF);
  value += static_cast<char>((fourcc >> 16) & 0xFF);
  value += static_cast<char>((fourcc >> 24) & 0xFF);
  return value;
}

} // namespace

namespace elp_camera {

ElpCameraCore::ElpCameraCore() : width_(0), height_(0) {}

ElpCameraCore::~ElpCameraCore() { release(); }

bool ElpCameraCore::setup(int device_id, int width, int height, double fps,
                          const std::string &pixel_format) {
  cap_.open(device_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    std::cerr << "Failed to open camera device: " << device_id << std::endl;
    return false;
  }

  const std::string normalized_pixel_format =
      normalize_pixel_format(pixel_format);
  const int requested_fourcc =
      fourcc_from_pixel_format(normalized_pixel_format);

  if (requested_fourcc == 0) {
    std::cerr << "Unsupported pixel format: " << pixel_format
              << " (Supported: MJPG, YUYV)" << std::endl;
    return false;
  }

  if (!cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(requested_fourcc))) {
    std::cerr << "Warning: Failed to set pixel format to "
              << normalized_pixel_format << std::endl;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS, fps);

  // Read back to verify configuration.
  double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  const int actual_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));

  if (actual_fourcc != requested_fourcc) {
    std::cerr << "Warning: Requested pixel format " << normalized_pixel_format
              << " but got " << fourcc_to_string(actual_fourcc) << std::endl;
  }

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

  cv::Mat frame_to_split;
  if (raw_frame.channels() == 2) {
    // CAP_PROP_CONVERT_RGB may be disabled by backend for YUYV.
    cv::cvtColor(raw_frame, frame_to_split, cv::COLOR_YUV2BGR_YUY2);
  } else if (raw_frame.channels() == 3) {
    frame_to_split = raw_frame;
  } else if (raw_frame.channels() == 4) {
    cv::cvtColor(raw_frame, frame_to_split, cv::COLOR_BGRA2BGR);
  } else {
    std::cerr << "Unsupported captured frame channels: " << raw_frame.channels()
              << std::endl;
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
  frame_to_split(left_roi).copyTo(left_frame);
  frame_to_split(right_roi).copyTo(right_frame);

  return true;
}

} // namespace elp_camera
