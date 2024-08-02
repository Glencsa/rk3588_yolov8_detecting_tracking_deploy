
#pragma once
#include "threadpool.h"
#include "yolov8.h"
#include "queue"
#include "opencv2/opencv.hpp"
#include "image_process.h"
#include "camera.h"
class RknnPool {
 public:
  RknnPool(const std::string model_path, const int thread_num, const std::string label_path);
  ~RknnPool();
  void Init();
  void DeInit();
  void AddInferenceTask(frame_with_time& src, ImageProcess &image_process);
  int get_model_id();
  //string pool_id;

  std::shared_ptr<cv::Mat> GetImageResultFromQueue();
  std::shared_ptr<object_detect_result_list> GetObjResultFromQueue();
  int GetTasksSize();
 private:
  int thread_num_{1};
  std::string model_path_{"null"};
  std::string label_path_{"null"};
  uint32_t id{0};
  std::unique_ptr<ThreadPool> pool_;
  std::queue<std::shared_ptr<cv::Mat>> image_results_;
  std::vector<std::shared_ptr<Yolov8>> models_;
  std::mutex id_mutex_;
  std::mutex image_results_mutex_;
  std::queue<std::shared_ptr<object_detect_result_list>> obj_results;
  std::mutex obj_results_mutex_;
};
