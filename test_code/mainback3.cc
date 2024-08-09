#include <thread>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "yolov8.h"
#include <thread>
#include<unistd.h>
#include <iconv.h>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "camera.h"
#include "getopt.h"
#include "rknn_pool.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

using TimeDuration=std::chrono::milliseconds;
class Timeout {
private:
    std::chrono::steady_clock::time_point start_time;
    std::chrono::milliseconds timeout_duration;

public:
    Timeout(std::chrono::milliseconds duration) : timeout_duration(duration) {
        start_time = std::chrono::steady_clock::now();
    }

    bool isTimeout() const {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        return elapsed_time >= timeout_duration;
    }
};

struct ProgramOptions {
  std::string model_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/yolov8s.rknn";
  std::string label_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/coco_80_labels_list.txt";
  int thread_count = 9;
  int camera_index = 21;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=1 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=1 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
  int width = 1280;
  int height = 720;
  double fps = 25.0;
  int width2 = 1920;
  int height2 = 1080;
  double fps2 = 20.0;
};
// 全局变量或者类成员变量
// std::mutex displayMutex;
// std::condition_variable imageAvailableCond;
// bool isImageAvailable = false;
// cv::Mat currentImage;

// void displayThreadFunc() {
//     while (true) {
//         // 等待图像可用的通知
//         {
//             std::unique_lock<std::mutex> lock(displayMutex);
//             imageAvailableCond.wait(lock, []{ return isImageAvailable; });
//         }

//         // 显示图像
//         cv::imshow("Video2", currentImage);
//         cv::waitKey(1);

//         // 重置图像可用标志
//         {
//             std::lock_guard<std::mutex> lock(displayMutex);
//             isImageAvailable = false;
//         }
//     }
// }
// 声明一个用于存储图像数据的缓冲区

std::deque<std::unique_ptr<cv::Mat>> imageBuffer2;
std::mutex bufferMutex2;
std::condition_variable bufferCondition2;
std::deque<std::unique_ptr<cv::Mat>> imageBuffer1;
std::mutex bufferMutex1;
std::condition_variable bufferCondition1;
int main() {
  ProgramOptions options;

  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count, options.label_path);
  auto rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count, options.label_path);
  auto camera1 = std::make_unique<Camera>(
      options.camera_index, cv::Size(options.width, options.height),
      options.fps);
  auto camera2 = std::make_unique<Camera>(
      options.rtsp_url, cv::Size(options.width2, options.height2),
      options.fps);

  ImageProcess image_process(options.width, options.height, 640);
  ImageProcess image_process2(options.width2, options.height2, 640);
  std::unique_ptr<cv::Mat> image1;
  std::shared_ptr<cv::Mat> image_res1;
  std::unique_ptr<cv::Mat> image2;
  std::shared_ptr<cv::Mat> image_res2;

  cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Video2", cv::WINDOW_AUTOSIZE); 
    
  static int image_count1 = 0;
  static int image_res_count1 = 0;
  static int image_count2 = 0;
  static int image_res_count2 = 0;
  TimeDuration time_duration;
  Timeout timeout(std::chrono::seconds(30));
  TimeDuration total_time;
struct timeval time1;
  gettimeofday(&time1, nullptr);
  long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
   struct timeval time2;
  gettimeofday(&time2, nullptr);
  long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;

// 新建一个线程用于获取图像并将其放入缓冲区
auto imageCaptureThread = [&]() {
  while (1) {
    auto image2 = camera2->GetNextFrame();
    if (image2 != nullptr) {
      // 加锁以向缓冲区添加图像
      std::unique_lock<std::mutex> lock(bufferMutex2);
      imageBuffer2.push_back(std::move(image2));
      lock.unlock();
      // 通知等待的线程有新的图像可用
      bufferCondition2.notify_one();
    }
  }
};
auto imageCaptureThread1 = [&]() {
  while (1) {
    auto image1 = camera1->GetNextFrame();
    if (image1 != nullptr) {
      // 加锁以向缓冲区添加图像
      std::unique_lock<std::mutex> lock(bufferMutex1);
      imageBuffer1.push_back(std::move(image1));
      lock.unlock();
      // 通知等待的线程有新的图像可用
      bufferCondition1.notify_one();
    }
  }
};
  // 线程函数1，处理第一个摄像头
  auto thread_func1 = [&]() {
    while ((!timeout.isTimeout()) || (image_count1 != image_res_count1)) {
      // 处理第一个摄像头的帧
       std::unique_lock<std::mutex> lock(bufferMutex1);
    bufferCondition1.wait(lock, [&] { return !imageBuffer1.empty(); });
    auto image1 = std::move(imageBuffer1.front());
    imageBuffer1.pop_front();
    lock.unlock();
    //  image1 = camera1->GetNextFrame();
      if (image1 != nullptr) {
        rknn_pool1->AddInferenceTask(std::move(image1), image_process);
        image_count1++;
      }

      image_res1 = rknn_pool1->GetImageResultFromQueue();
      if (image_res1 != nullptr) {
        image_res_count1++;

        if (image_res_count1 % 60 == 0) {
          gettimeofday(&time1, nullptr);
          tmpTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
          printf("第一个摄像头60帧平均帧率1:\t%f帧\n", 60000.0 / (float)(tmpTime1 - lopTime1));
          lopTime1 = tmpTime1;
        }
        cv::imshow("Video", *image_res1);
        cv::waitKey(1);
      }
    }
  };

  // 线程函数2，处理第二个摄像头
  auto thread_func2 = [&]() {
    while ((!timeout.isTimeout()) || (image_count2 != image_res_count2)) {
      // 处理第二个摄像头的帧
      // 等待缓冲区中有图像可用
    std::unique_lock<std::mutex> lock(bufferMutex2);
    bufferCondition2.wait(lock, [&] { return !imageBuffer2.empty(); });
    auto image2 = std::move(imageBuffer2.front());
    imageBuffer2.pop_front();
    lock.unlock();
      //auto image2 = camera2->GetNextFrame();
      if (image2 != nullptr) {
        rknn_pool2->AddInferenceTask(std::move(image2), image_process2);
        image_count2++;
      }

      auto image_res2 = rknn_pool2->GetImageResultFromQueue();
      if (image_res2 != nullptr) {
        image_res_count2++;

        if (image_res_count2 % 60 == 0) {
          gettimeofday(&time2, nullptr);
          tmpTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
          printf("第二个摄像头60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime2 - lopTime2));
          lopTime2 = tmpTime2;
        }
        cv::imshow("Video2", *image_res2);
        cv::waitKey(1);
       
      }
    }
  };

  // 创建两个线程并启动
  std::thread thread1(thread_func1);
  std::thread thread2(thread_func2);
std::thread thread3(imageCaptureThread);
 std::thread thread4(imageCaptureThread1);
 //std::thread displayThread(displayThreadFunc);
  // 等待两个线程结束
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
//displayThread.join();
  rknn_pool1.reset();
  rknn_pool2.reset();
  cv::destroyAllWindows();
  return 0;
}
