

#pragma once
#include "opencv2/opencv.hpp"
#include "postprocess.h"

class ImageProcess {
 public:
 //ImageProcess():scale_(0),padding_x_(0),padding_y_(0),new_size_(0,0),target_size_(0){}
  ImageProcess(int width, int height, int target_size);
  std::unique_ptr<cv::Mat> Convert(const cv::Mat &src);
  const letterbox_t &get_letter_box();
  void ImagePostProcess(cv::Mat &image, object_detect_result_list &od_results);

private:
  double scale_;
  int padding_x_;
  int padding_y_;
  cv::Size new_size_;
  int target_size_;
  letterbox_t letterbox_;
};
