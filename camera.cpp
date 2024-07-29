

#include "camera.h"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "thread"


Camera::Camera(const std::string& rtsp_url, cv::Size size, double framerate)
    : capture_(rtsp_url,cv::CAP_GSTREAMER) {
    if (!capture_.isOpened()) {
        std::cerr << "Error opening video stream or file" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Camera width: " << capture_.get(cv::CAP_PROP_FRAME_WIDTH) << ", height: " 
              << capture_.get(cv::CAP_PROP_FRAME_HEIGHT) << ", fps: " << capture_.get(cv::CAP_PROP_FPS) << std::endl;
}
Camera::Camera(uint16_t index, cv::Size size, double framerate)
  : capture_(index, cv::CAP_V4L2), size_(size) {
  printf("Instantiate a Camera object");
  // 这里使用V4L2捕获，因为使用默认的捕获不可以设置捕获的模式和帧率
  if (!capture_.isOpened()) {
    printf("Error opening video stream or file");
    exit(EXIT_FAILURE);
  }
  capture_.set(cv::CAP_PROP_FOURCC,
               cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
  // 检查是否成功设置格式
  int fourcc = capture_.get(cv::CAP_PROP_FOURCC);
  if (fourcc != cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V')) {
    printf("Set video format failed");
  }
  capture_.set(cv::CAP_PROP_FRAME_WIDTH, size_.width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, size_.height);
  if(!capture_.set(cv::CAP_PROP_FPS, framerate)){
    printf("set framerate failed!!");
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printf("camera width: {}, height: {}, fps: {}", capture_.get(cv::CAP_PROP_FRAME_WIDTH),
                     capture_.get(cv::CAP_PROP_FRAME_HEIGHT), capture_.get(cv::CAP_PROP_FPS));
}

Camera::~Camera() {
  if (capture_.isOpened()){
    printf("Release camera");
    capture_.release();
  }
}

frame_with_time Camera::GetNextFrame() {
  auto frame = std::make_unique<cv::Mat>();
  capture_ >> *frame;
  // cv::cvtColor(*frame, *frame, cv::COLOR_BGR2YUV_I420);
  if (frame->empty()) {
    printf("Get frame error");
    return nullptr;
  }
  auto time = std::chrono::high_resolution_clock::now();
  frame_with_time frame_t;
  frame_t.frame = std::move(frame);
  frame_t.timestamp = time;
  return frame_t;
}