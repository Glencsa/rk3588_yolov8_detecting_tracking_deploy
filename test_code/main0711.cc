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

#include <thread>
#include <mutex>
#include <condition_variable>
#include <SDL2/SDL.h>
#include "rtsp_demo.h"
#include "mpp_decoder.h"
#include "mpp_encoder.h"
#include "drawing.h"
#include "mk_mediakit.h"
#include "im2d.h"
#include "rga.h"
#include "RgaUtils.h"
#define OUT_VIDEO_PATH "out.h264"

rtsp_demo_handle g_rtsplive177 = NULL;
static rtsp_session_handle g_rtsp_session177;
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
//------------------------------------------------------------------------------
//功能：将YUV420视频帧数据填充到MPP buffer
//说明：使用16字节对齐，MPP可以实现零拷贝，提高效率1
//------------------------------------------------------------------------------


struct ProgramOptions {
  std::string model_path = "/home/fast/chm/rknn_model_zoo-main/examples/yolov8/model/bestm3.rknn";
  std::string label_path = "/home/fast/chm/rknn_model_zoo-main/examples/yolov8/model/coco_80_labels_list.txt";
  int thread_count1 = 12;
  int thread_count2 =9;
  int camera_index = 0;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.156/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
   std::string rtsp_url3 = "rtspsrc location=rtsp://admin:admin@192.168.1.169/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
   std::string rtsp_url4 = "rtspsrc location=rtsp://192.168.2.119/554 latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue !  appsink ";
   
  int width = 1920;
  int height = 1080;
  double fps = 27.0;
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
// 声明一个用于存储图像数据的缓冲区ab
//double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }
std::deque<std::unique_ptr<cv::Mat>> imageBuffer2;
std::mutex bufferMutex2;
std::condition_variable bufferCondition2;
std::deque<std::unique_ptr<cv::Mat>> imageBuffer1;
std::mutex bufferMutex1;
std::condition_variable bufferCondition1;







int main() {
  ProgramOptions options;
// 初始化SDL
//    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
//        SDL_Log("无法初始化SDL: %s", SDL_GetError());
//        return -1;
//    }
  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count1, options.label_path);
  // auto rknn_pool2 = std::make_unique<RknnPool>(
  //     options.model_path, options.thread_count2, options.label_path);
  auto camera1 = std::make_unique<Camera>(
      options.rtsp_url4, cv::Size(options.width, options.height),
      options.fps);
  // auto camera2 = std::make_unique<Camera>(
  //     options.rtsp_url, cv::Size(options.width2, options.height2),
  //     options.fps);

  ImageProcess image_process(options.width, options.height, 640);
  // ImageProcess image_process2(options.width2, options.height2, 640);
  std::unique_ptr<cv::Mat> image1;
  std::shared_ptr<cv::Mat> image_res1;
  std::unique_ptr<cv::Mat> image2;
   std::unique_ptr<cv::Mat> frame2;
  std::shared_ptr<cv::Mat> image_res2;
std::unique_ptr<cv::Mat> frame5 (new cv::Mat);
  //auto frame5 = std::make_unique<cv::Mat>();
  cv::Mat result1,result2,yuantu1,yuantu2;
// int width = 1920;
//   int height = 1080;
  // void *mpp_frame = NULL;
  // int mpp_frame_fd = 0;
  // void *mpp_frame_addr = NULL;
  // int enc_data_size;
  // static int frame_index = 0;

  //   // init rtsp
  // g_rtsplive = create_rtsp_demo(554);
  // g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");
  // rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
  // rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

  //   // init encoder
  //   MppEncoder *mpp_encoder = new MppEncoder();
  //   MppEncoderParams enc_params;
  //   memset(&enc_params, 0, sizeof(MppEncoderParams));
  //   enc_params.width = 1920;
  //   enc_params.height = 1080;
  //   enc_params.hor_stride = 1920;
  //   enc_params.ver_stride = 1080;
  //   enc_params.fmt = MPP_FMT_YUV420SP;
  //   enc_params.type = MPP_VIDEO_CodingAVC; //H264
  //   mpp_encoder->Init(enc_params, NULL);
  //   mpp_frame = mpp_encoder->GetInputFrameBuffer();
  //   mpp_frame_fd = mpp_encoder->GetInputFrameBufferFd(mpp_frame);
  //   mpp_frame_addr = mpp_encoder->GetInputFrameBufferAddr(mpp_frame);

  //   int enc_buf_size = mpp_encoder->GetFrameSize();
  //   char *enc_data = (char *)malloc(enc_buf_size);
// 打开RTSP摄像头
  //   cv::VideoCapture cap( options.rtsp_url3, cv::CAP_GSTREAMER);

  //   // 检查摄像头是否成功打开
  //   if (!cap.isOpened()) {
  //       std::cerr << "Error: Cannot open RTSP stream" << std::endl;
  //       return -1;
  //   }

  //   // 获取RTSP摄像头的帧率
  //   double fps = cap.get(cv::CAP_PROP_FPS);
  //   std::cout << "帧率: " << fps << std::endl;

  //   // 获取RTSP摄像头的宽度和高度
  //   int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  //   int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  //   std::cout << "宽度: " << frame_width << "  高度: " << frame_height << std::endl;

  //  cv::Mat frame_rtsp155;
   

 // 创建窗口和渲染器1
//    SDL_Window* window = SDL_CreateWindow("Video2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
//    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
//    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
//    SDL_Window* window1 = SDL_CreateWindow("Video1", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
//    SDL_Renderer* renderer1 = SDL_CreateRenderer(window1, -1, SDL_RENDERER_SOFTWARE);
//    SDL_Texture* texture1 = SDL_CreateTexture(renderer1, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
  //cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
 // cv::namedWindow("Video2", cv::WINDOW_AUTOSIZE); 
    
  static int image_count1 = 0;
  static int image_res_count1 = 0;
  static int image_count2 = 0;
  static int image_res_count2 = 0;
  static int camera_count = 0;
  static int inference_count = 0;
  TimeDuration time_duration;
  Timeout timeout(std::chrono::seconds(30));
  TimeDuration total_time;
struct timeval time1;
  gettimeofday(&time1, nullptr);
  long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
   struct timeval time2;
  gettimeofday(&time2, nullptr);
  long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
 struct timeval start_time, stop_time;
  // struct timeval start_time2, stop_time2;
// 新建一个线程用于获取图像并将其放入缓冲区
// auto imageCaptureThread = [&]() {
//   auto starttime1 = std::chrono::high_resolution_clock::now();
//   while (1) {
     
//     auto image2 = camera2->GetNextFrame();
//     if (image2 != nullptr) {
//       // 加锁以向缓冲区添加图像
//       std::unique_lock<std::mutex> lock(bufferMutex2);
//       imageBuffer2.push_back(std::move(image2));
//       lock.unlock();
//       // 通知等待的线程有新的图像可用
//       bufferCondition2.notify_one();
//       camera_count++;
//       if (camera_count==30){
//       auto endtime1 = std::chrono::high_resolution_clock::now();
//     // // 计算持续时间
//         std::chrono::duration<double,std::milli> duration = endtime1 - starttime1;
//           printf("camera use time: \t%f ms\n", duration.count()/30);
//           starttime1=endtime1;
//           camera_count=0;
//           }
//     }
//   }
// };
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
      void *mpp_frame177 = NULL;
       int mpp_frame_fd177 = 0;
       void *mpp_frame_addr177 = NULL;
       int enc_data_size177;
       static int frame_index177 = 0;
         // init rtsp
       g_rtsplive177 = create_rtsp_demo(177);
       g_rtsp_session177 = rtsp_new_session(g_rtsplive177, "/live177/main_stream");
       rtsp_set_video(g_rtsp_session177, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
       rtsp_sync_video_ts(g_rtsp_session177, rtsp_get_reltime(), rtsp_get_ntptime());

         // init encoder
         MppEncoder *mpp_encoder177 = new MppEncoder();
         MppEncoderParams enc_params177;
         memset(&enc_params177, 0, sizeof(MppEncoderParams));
         enc_params177.width = 1920;
         enc_params177.height = 1080;
         enc_params177.hor_stride = 1920*3;
         enc_params177.ver_stride = 1088;
         enc_params177.fmt = MPP_FMT_BGR888;//MPP_FMT_BGR888  MPP_FMT_YUV420SP  MPP_FMT_RGB888
         enc_params177.type = MPP_VIDEO_CodingAVC; //H264
         mpp_encoder177->Init(enc_params177, NULL);

         mpp_frame177 = mpp_encoder177->GetInputFrameBuffer();
         mpp_frame_fd177 = mpp_encoder177->GetInputFrameBufferFd(mpp_frame177);
         mpp_frame_addr177 = mpp_encoder177->GetInputFrameBufferAddr(mpp_frame177);
       //  cv::Mat bgr_frame(enc_params.height ,enc_params.width,CV_8UC3,mpp_frame_addr);
         int enc_buf_size177 = mpp_encoder177->GetFrameSize();
         char *enc_data177 = (char *)malloc(enc_buf_size177);
    
    while (1) {
      // 处理第一个摄像头的帧
      // gettimeofday(&start_time2, NULL);
       std::unique_lock<std::mutex> lock(bufferMutex1);
    bufferCondition1.wait(lock, [&] { return !imageBuffer1.empty(); });
    auto image1 = std::move(imageBuffer1.front());
    imageBuffer1.pop_front();
    lock.unlock();
    //auto image1 = camera1->GetNextFrame();
    // gettimeofday(&stop_time2, NULL);
    // printf("once run use %f ms\n", (__get_us(stop_time2) - __get_us(start_time2)) / 1000);
    // start_time2=stop_time2;
      if (image1 != nullptr) {
        printf("image_count1:%d \n",image_count1);
        // if(image_count1 % 5 == 0){
        //   yuantu1 = *image1;
        //   std::string save_1 = "/mnt/ssd/yuantu1";
        //   std::string imagesave1=save_1+"/"+std::to_string(image_count1)+".jpg";
        //   cv::imwrite(imagesave1,yuantu1);
        // } 
        rknn_pool1->AddInferenceTask(std::move(image1), image_process);
        image_count1++;
      }
      image_res1 = rknn_pool1->GetImageResultFromQueue();
      if (image_res1 != nullptr) {
         printf("image_res_count1:%d \n",image_res_count1);
        image_res_count1++;
        memcpy(mpp_frame_addr177,image_res1->data,1920 * 1080 * 3);
           if (frame_index177 == 1)
           {
               enc_data_size177 = mpp_encoder177->GetHeader(enc_data177, enc_buf_size177);
               if (g_rtsplive177 && g_rtsp_session177) {
                   rtsp_tx_video(g_rtsp_session177, (const uint8_t *)enc_data177, enc_data_size177,frame_index177);
                   rtsp_do_event(g_rtsplive177);
               }
           }
           memset(enc_data177, 0, enc_buf_size177);
           enc_data_size177 = mpp_encoder177->Encode(mpp_frame177, enc_data177, enc_buf_size177);
           if (g_rtsplive177 && g_rtsp_session177) {
               rtsp_tx_video(g_rtsp_session177, (const uint8_t *)enc_data177, enc_data_size177,frame_index177);
               rtsp_do_event(g_rtsplive177);
           }
          //   gettimeofday(&time1, nullptr);
          // tmpTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
          // printf("inference time :\t%.6fms\n",  (float)(tmpTime1 - lopTime1));
          // lopTime1 = tmpTime1;
        // if (image_res_count1 % 20 == 0) {
        //   gettimeofday(&time1, nullptr);
        //   tmpTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
        //   printf("第一个摄像头20帧平均帧率1:\t%.6fms\n",  (float)(tmpTime1 - lopTime1)/20);
        //   lopTime1 = tmpTime1;
        // }
        // if(image_res_count1 % 10 == 0){
        //   result1 = *image_res1;
        //   std::string save_dir1 = "/mnt/ssd/result1";
        //   std::string imagesavepath1=save_dir1+"/"+std::to_string(image_count1)+".jpg";
        //   cv::imwrite(imagesavepath1,result1);
        // }
       //  cv::imshow("Video", *image_res1);
        // cv::waitKey(1);
        
//        SDL_UpdateTexture(texture1, NULL, image_res1->data, image_res1->step);
//         SDL_RenderClear(renderer1);
//        SDL_RenderCopy(renderer1, texture1, NULL, NULL);
//        SDL_RenderPresent(renderer1);
      }
    }
  };
#if 0
  // 线程函数2，处理第二个摄像头
  auto thread_func2 = [&]() {
    // struct timespec start_time;
    // clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
    
    while (1) {
      // 处理第二个摄像头的帧
// 获取开始时间点
    std::unique_lock<std::mutex> lock(bufferMutex2);
    bufferCondition2.wait(lock, [&] { return !imageBuffer2.empty(); });
    auto image2 = std::move(imageBuffer2.front());
    imageBuffer2.pop_front();
    lock.unlock();
    // printf("检查帧是否为空");
    auto starttime = std::chrono::high_resolution_clock::now();
    //  cap >> frame_rtsp155; // 读取帧
    // //   auto frame2=std::move(frame5);
    // // cv::Mat image(1080, 1920, CV_8UC3, (unsigned char*)map.data);
    // std::unique_ptr<cv::Mat> image_ptr = std::make_unique<cv::Mat>(frame_rtsp155);
        
        
        
    //auto image2 = camera2->GetNextFrame();
    // inference_count++;
    //  if (inference_count % 30 == 0) {
    //       auto endtime = std::chrono::high_resolution_clock::now();
    // // // 计算持续时间
    //   std::chrono::duration<double,std::milli> duration = endtime - starttime;
    //   printf("inference use time: \t%f ms\n", duration.count());
    //   inference_count=0;
    //     }
    //  auto starttime = std::chrono::high_resolution_clock::now();
 // auto image2 = camera2->GetNextFrame();
    // auto image2 = camera2->GetNextFrame();
      if (image2 != nullptr) {
        // if(image_count2 > 10){
          printf("image_count2:%d \n",image_count2);
        //   yuantu2 = *image2;
        //   std::string save_2 = "/mnt/ssd/yuantu2";
        //   std::string imagesave2=save_2+"/"+std::to_string(image_count2)+".jpg";
        //   cv::imwrite(imagesave2,yuantu2);   
       
            // }
       
          rknn_pool2->AddInferenceTask(std::move(image2), image_process2);
          image_count2++;  
      }
    
      auto image_res2 = rknn_pool2->GetImageResultFromQueue();
      
            
      if (image_res2 != nullptr) {
        image_res_count2++;
          // if (image_res_count2 % 30 == 0) {
          
        //  image_res_count2=0;
        
        //}
        // if (image_res_count2 % 60 == 0) {
        //   gettimeofday(&time2, nullptr);
        //   tmpTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
        //   printf("第二个摄像头60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime2 - lopTime2));
        //   lopTime2 = tmpTime2;
        // }
        // if(image_res_count2 % 10 == 0){
          printf("image_res_count2:%d \n",image_res_count2);
        //   result2 = *image_res2;
        //   std::string save_dir2 = "/mnt/ssd/result2";
        //   std::string imagesavepath2=save_dir2+"/"+std::to_string(image_count2)+".jpg";
        //   cv::imwrite(imagesavepath2,result2);
        // }

    
        // cv::imshow("Video3", *image_res2);
        // cv::waitKey(1);
       // 将图像数据复制到SDL纹理
        // 获取结束时间点
    // struct timespec end_time;
    // clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
    // // 计算程序运行时间
    // double elapsed_time = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
    // printf("inference use time %.5f ms\n", elapsed_time);
    //   start_time=end_time;
    //  auto endtime = std::chrono::high_resolution_clock::now();
    // // // 计算持续时间
    //   std::chrono::duration<double,std::milli> duration = endtime - starttime;
    //   printf("inference use time: \t%f ms\n", duration.count());
    cvtColor(*image_res2,*image_res2,cv::COLOR_BGR2YUV_I420);
      read_yuv_buffer((RK_U8*)mpp_frame_addr, *image_res2, 1920, 1080);
    // Encode to file
    // Write header on first frame
    if (frame_index == 1)
    {
        enc_data_size = mpp_encoder->GetHeader(enc_data, enc_buf_size);
        if (g_rtsplive && g_rtsp_session) {
            rtsp_tx_video(g_rtsp_session, (const uint8_t *)enc_data, enc_data_size,frame_index);
            rtsp_do_event(g_rtsplive);
        }
    }
    memset(enc_data, 0, enc_buf_size);
    enc_data_size = mpp_encoder->Encode(mpp_frame, enc_data, enc_buf_size);
    if (g_rtsplive && g_rtsp_session) {
        rtsp_tx_video(g_rtsp_session, (const uint8_t *)enc_data, enc_data_size,frame_index);
        rtsp_do_event(g_rtsplive);
    }
    
        // SDL_UpdateTexture(texture, NULL, image_res2->data, image_res2->step);
        //  SDL_RenderClear(renderer);
        // SDL_RenderCopy(renderer, texture, NULL, NULL);
        // SDL_RenderPresent(renderer);
           auto endtime = std::chrono::high_resolution_clock::now();
    // // 计算持续时间
      std::chrono::duration<double,std::milli> duration = endtime - starttime;
      printf("camera use time: \t%f ms\n", duration.count());
      }
    }
  };
#endif
  // 创建两个线程并启动
 // std::thread thread3(imageCaptureThread);
 std::thread thread4(imageCaptureThread1);
std::thread thread1(thread_func1);
  //std::thread thread2(thread_func2);

 //std::thread displayThread(displayThreadFunc);
  // 等待两个线程结束
 thread1.join();
//  thread2.join();
  //thread3.join();
 thread4.join();
//displayThread.join();
  rknn_pool1.reset();
  //rknn_pool2.reset();
  // 清理资源
// delete[] mpp_frame_addr;
// delete[] mpp_frame;
//SDL_DestroyTexture(texture);
//SDL_DestroyRenderer(renderer);
//SDL_DestroyWindow(window);
//SDL_DestroyTexture(texture1);
//SDL_DestroyRenderer(renderer1);
//SDL_DestroyWindow(window1);
//SDL_Quit();
  //cv::destroyAllWindows();
  return 0;
}
