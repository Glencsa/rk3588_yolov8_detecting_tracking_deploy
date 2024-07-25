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
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))


//init rtsp class
rtsp_demo_handle g_rtsplive155 = NULL;
static rtsp_session_handle g_rtsp_session155;

rtsp_demo_handle g_rtsplive163 = NULL;
static rtsp_session_handle g_rtsp_session163;

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
//说明：使用16字节对齐，MPP可以实现零拷贝，提高效率
//------------------------------------------------------------------------------
void read_yuv_buffer(RK_U8 *buf, cv::Mat &yuvImg, RK_U32 width, RK_U32 height)
{
  //printf("color convert \n");
    RK_U8 *buf_y = buf;
    // RK_U8 *buf_u = buf + MPP_ALIGN(width, 16) * MPP_ALIGN(height, 16);
    // RK_U8 *buf_v = buf_u + MPP_ALIGN(width, 16) * MPP_ALIGN(height, 16) / 4;
    RK_U8 *buf_u = buf + width * height;
    RK_U8 *buf_v = buf_u + width * height / 4;
    //
    RK_U8 *yuvImg_y = yuvImg.data;
    RK_U8 *yuvImg_u = yuvImg_y + width * height;
    RK_U8 *yuvImg_v = yuvImg_u + width * height / 4;
    //
    memcpy(buf_y, yuvImg_y, width * height);
    memcpy(buf_u, yuvImg_u, width * height / 4);
    memcpy(buf_v, yuvImg_v, width * height / 4);
}


struct ProgramOptions {
  std::string model_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/yolov8s.rknn";
  std::string label_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/coco_80_labels_list.txt";
  int thread_count1 = 3;
  int thread_count2 =3;
  int camera_index = 0;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert  !  queue ! appsink";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.163/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink  ";
   std::string rtsp_url3 = "rtspsrc location=rtsp://admin:admin@192.168.1.177/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
   std::string rtsp_url4 = "rtspsrc location=rtsp://192.168.2.119/554 latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue !  appsink ";
   
  int width = 1920;
  int height = 1080;
  double fps = 20.0;
  int width2 = 1920;
  int height2 = 1080;
  double fps2 = 20.0;
};
// 全局变量或者类成员变量1
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
double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }
std::deque<std::unique_ptr<cv::Mat>> imageBuffer2;
std::mutex bufferMutex2;
std::condition_variable bufferCondition2;
std::deque<std::unique_ptr<cv::Mat>> imageBuffer1;
std::mutex bufferMutex1;
std::condition_variable bufferCondition1;




int main() {
  ProgramOptions options;
// // 初始化SDL
//     if (SDL_Init(SDL_INIT_VIDEO) < 0) {
//         SDL_Log("无法初始化SDL: %s", SDL_GetError());
//         return -1;
//     } 
  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count1, options.label_path);
  auto rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);
  auto rknn_pool3 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);
  // auto camera1 = std::make_unique<Camera>(
  //     options.rtsp_url2, cv::Size(options.width, options.height),
  //     options.fps);
  // auto camera2 = std::make_unique<Camera>(
  //     options.rtsp_url, cv::Size(options.width2, options.height2),
  //     options.fps);

  ImageProcess image_process(options.width, options.height, 640);
  ImageProcess image_process2(options.width2, options.height2, 640);
   ImageProcess image_process3(options.width2, options.height2, 640);
  std::unique_ptr<cv::Mat> image1;
  std::shared_ptr<cv::Mat> image_res1;
  std::unique_ptr<cv::Mat> image2;
  std::shared_ptr<cv::Mat> image_res2;
    std::unique_ptr<cv::Mat> image3;
  std::shared_ptr<cv::Mat> image_res3;
  cv::Mat result1,result2,yuantu1,yuantu2;

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


   
// 打开RTSP摄像头  155 camera
    cv::VideoCapture cap155(options.rtsp_url, cv::CAP_GSTREAMER);
    // 检查摄像头是否成功打开
    if (!cap155.isOpened()) {
        std::cerr << "Error: Cannot open RTSP155 stream" << std::endl;
        return -1;
    }
    // 获取RTSP摄像头的帧率
    double fps = cap155.get(cv::CAP_PROP_FPS);
    std::cout << "帧率: " << fps << std::endl;
    // 获取RTSP摄像头的宽度和高度
    int frame_width = cap155.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap155.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "宽度: " << frame_width << "  高度: " << frame_height << std::endl;
   cv::Mat frame_rtsp155;
   

// 打开RTSP摄像头  163 camera
    cv::VideoCapture cap163(options.rtsp_url2, cv::CAP_GSTREAMER);
    // 检查摄像头是否成功打开
    if (!cap163.isOpened()) {
        std::cerr << "Error: Cannot open RTSP163 stream" << std::endl;
        return -1;
    }
   cv::Mat frame_rtsp163;


//  // 创建窗口和渲染器1
//     SDL_Window* window = SDL_CreateWindow("Video2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
//     SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
//     SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
//     SDL_Window* window1 = SDL_CreateWindow("Video1", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
//     SDL_Renderer* renderer1 = SDL_CreateRenderer(window1, -1, SDL_RENDERER_SOFTWARE);
//     SDL_Texture* texture1 = SDL_CreateTexture(renderer1, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
  
  // struct timeval start_time2, stop_time2;
// 新建一个线程用于获取图像并将其放入缓冲区
// auto imageCaptureThread = [&]() {
//   //auto starttime1 = std::chrono::high_resolution_clock::now();
//   while (1) {
     
//     auto image2 = camera2->GetNextFrame();
//     if (image2 != nullptr) {
//       // 加锁以向缓冲区添加图像
//       std::unique_lock<std::mutex> lock(bufferMutex2);
//       imageBuffer2.push_back(std::move(image2));
//       lock.unlock();
//       // 通知等待的线程有新的图像可用
//       bufferCondition2.notify_one();
//       //camera_count++;
//     //   if (camera_count==30){
//     //   auto endtime1 = std::chrono::high_resolution_clock::now();
//     // // // 计算持续时间
//     //     std::chrono::duration<double,std::milli> duration = endtime1 - starttime1;
//     //       printf("camera use time: \t%f ms\n", duration.count()/30);
//     //       starttime1=endtime1;
//     //       camera_count=0;
//     //       }
//     }
//   }
// };
// auto imageCaptureThread1 = [&]() {

//   while (1) {
//     auto image1 = camera1->GetNextFrame();
//     if (image1 != nullptr) {
//       // 加锁以向缓冲区添加图像
//       std::unique_lock<std::mutex> lock(bufferMutex1);
//       imageBuffer1.push_back(std::move(image1));
//       lock.unlock();
//       // 通知等待的线程有新的图像可用
//       bufferCondition1.notify_one();
//     }
//   }
// };
  // 线程函数1，处理第一个摄像头
  auto thread_func1 = [&]() {
      //init value
  void *mpp_frame163 = NULL;
  int mpp_frame_fd163 = 0;
  void *mpp_frame_addr163 = NULL;
  int enc_data_size163;
  static int frame_index163 = 0;
    // init rtsp
  g_rtsplive163 = create_rtsp_demo(163);
  g_rtsp_session163 = rtsp_new_session(g_rtsplive163, "/live163/main_stream");
  rtsp_set_video(g_rtsp_session163, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
  rtsp_sync_video_ts(g_rtsp_session163, rtsp_get_reltime(), rtsp_get_ntptime());
    // init encoder  
    MppEncoder *mpp_encoder163 = new MppEncoder();
    MppEncoderParams enc_params163;
    memset(&enc_params163, 0, sizeof(MppEncoderParams));
    enc_params163.width = 1920;
    enc_params163.height = 1080;
    enc_params163.hor_stride = 1920*3;
    enc_params163.ver_stride = 1088;
    enc_params163.fmt = MPP_FMT_BGR888;//MPP_FMT_BGR888  MPP_FMT_YUV420SP  MPP_FMT_RGB888  
    enc_params163.type = MPP_VIDEO_CodingAVC; //H264
    mpp_encoder163->Init(enc_params163, NULL);
    
    mpp_frame163 = mpp_encoder163->GetInputFrameBuffer();
    mpp_frame_fd163 = mpp_encoder163->GetInputFrameBufferFd(mpp_frame163);
    mpp_frame_addr163 = mpp_encoder163->GetInputFrameBufferAddr(mpp_frame163);
  //  cv::Mat bgr_frame(enc_params.height ,enc_params.width,CV_8UC3,mpp_frame_addr);
    int enc_buf_size163 = mpp_encoder163->GetFrameSize();
    char *enc_data163 = (char *)malloc(enc_buf_size163);
    
    while (1) {
      // 处理第一个摄像头的帧
      // gettimeofday(&start_time2, NULL);
    //    std::unique_lock<std::mutex> lock(bufferMutex1);
    // bufferCondition1.wait(lock, [&] { return !imageBuffer1.empty(); });
    // auto image1 = std::move(imageBuffer1.front());
    // imageBuffer1.pop_front();
    // lock.unlock();
    //auto image1 = camera1->GetNextFrame();
     cap163 >> frame_rtsp163; // 读取帧
    
    std::unique_ptr<cv::Mat> image_ptr163 = std::make_unique<cv::Mat>(frame_rtsp163);
      if (image_ptr163 != nullptr) {
        // if(image_count1 % 5 == 0){
        //   yuantu1 = *image1;
        //   std::string save_1 = "/mnt/ssd/yuantu1";
        //   std::string imagesave1=save_1+"/"+std::to_string(image_count1)+".jpg";
        //   cv::imwrite(imagesave1,yuantu1);
        // } 
        rknn_pool1->AddInferenceTask(std::move(image_ptr163), image_process);
        image_count1++;
      }
      image_res1 = rknn_pool1->GetImageResultFromQueue();
      if (image_res1 != nullptr) {
        image_res_count1++;
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
         memcpy(mpp_frame_addr163,image_res1->data,1920 * 1080 * 3);
    if (frame_index163 == 1)
    {
        enc_data_size163 = mpp_encoder163->GetHeader(enc_data163, enc_buf_size163);
        if (g_rtsplive163 && g_rtsp_session163) {
            rtsp_tx_video(g_rtsp_session163, (const uint8_t *)enc_data163, enc_data_size163,frame_index163);
            rtsp_do_event(g_rtsplive163);
        }
    }
    memset(enc_data163, 0, enc_buf_size163);
    enc_data_size163 = mpp_encoder163->Encode(mpp_frame163, enc_data163, enc_buf_size163);
    if (g_rtsplive163 && g_rtsp_session163) {
        rtsp_tx_video(g_rtsp_session163, (const uint8_t *)enc_data163, enc_data_size163,frame_index163);
        rtsp_do_event(g_rtsplive163);
    }
        // SDL_UpdateTexture(texture1, NULL, image_res1->data, image_res1->step);
        //  SDL_RenderClear(renderer1);
        // SDL_RenderCopy(renderer1, texture1, NULL, NULL);
        // SDL_RenderPresent(renderer1);
      }
    }
  };

  // 线程函数2，处理第二个摄像头
  auto thread_func2 = [&]() {
    //init value
  void *mpp_frame155 = NULL;
  int mpp_frame_fd155 = 0;
  void *mpp_frame_addr155 = NULL;
  int enc_data_size155;
  static int frame_index155 = 0;
    // init rtsp
  g_rtsplive155 = create_rtsp_demo(155);
  g_rtsp_session155 = rtsp_new_session(g_rtsplive155, "/live155/main_stream");
  rtsp_set_video(g_rtsp_session155, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
  rtsp_sync_video_ts(g_rtsp_session155, rtsp_get_reltime(), rtsp_get_ntptime());

    // init encoder  
    MppEncoder *mpp_encoder155 = new MppEncoder();
    MppEncoderParams enc_params155;
    memset(&enc_params155, 0, sizeof(MppEncoderParams));
    enc_params155.width = 1920;
    enc_params155.height = 1080;
    enc_params155.hor_stride = 1920*3;
    enc_params155.ver_stride = 1088;
    enc_params155.fmt = MPP_FMT_BGR888;//MPP_FMT_BGR888  MPP_FMT_YUV420SP  MPP_FMT_RGB888  
    enc_params155.type = MPP_VIDEO_CodingAVC; //H264
    mpp_encoder155->Init(enc_params155, NULL);
    
    mpp_frame155 = mpp_encoder155->GetInputFrameBuffer();
    mpp_frame_fd155 = mpp_encoder155->GetInputFrameBufferFd(mpp_frame155);
    mpp_frame_addr155 = mpp_encoder155->GetInputFrameBufferAddr(mpp_frame155);
  //  cv::Mat bgr_frame(enc_params.height ,enc_params.width,CV_8UC3,mpp_frame_addr);
    int enc_buf_size155 = mpp_encoder155->GetFrameSize();
    char *enc_data155 = (char *)malloc(enc_buf_size155);
    
    while (1) {
      // 处理第二个摄像头的帧
// 获取开始时间点
    // std::unique_lock<std::mutex> lock(bufferMutex2);
    // bufferCondition2.wait(lock, [&] { return !imageBuffer2.empty(); });
    // auto image2 = std::move(imageBuffer2.front());
    // imageBuffer2.pop_front();
    // lock.unlock();
    // printf("检查帧是否为空");
    
     cap155 >> frame_rtsp155; // 读取帧
    //   auto frame2=std::move(frame5);
    // cv::Mat image(1080, 1920, CV_8UC3, (unsigned char*)map.data);
    std::unique_ptr<cv::Mat> image_ptr155 = std::make_unique<cv::Mat>(frame_rtsp155);
        
        
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
    auto starttime = std::chrono::high_resolution_clock::now();
      if (image_ptr155 != nullptr) {
        // if(image_count2 > 10){
          printf("image_count2:%d \n",image_count2);
        //   yuantu2 = *image2;
        //   std::string save_2 = "/mnt/ssd/yuantu2";
        //   std::string imagesave2=save_2+"/"+std::to_string(image_count2)+".jpg";
        //   cv::imwrite(imagesave2,yuantu2);   
       
            // }
          // cv::cvtColor(*image2, *image2, cv::COLOR_YUV2RGB_NV12);
          rknn_pool2->AddInferenceTask(std::move(image_ptr155), image_process2);
          image_count2++;  
      }
    
      auto image_res2 = rknn_pool2->GetImageResultFromQueue();
       auto endtime = std::chrono::high_resolution_clock::now(); 
    // // 计算持续时间
      std::chrono::duration<double,std::milli> duration = endtime - starttime;
      printf("camera use time: \t%f ms\n", duration.count());
            
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
    // Encode to file1
    // Write header on first frame
    memcpy(mpp_frame_addr155,image_res2->data,1920 * 1080 * 3);
    if (frame_index155 == 1)
    {
        enc_data_size155 = mpp_encoder155->GetHeader(enc_data155, enc_buf_size155);
        if (g_rtsplive155 && g_rtsp_session155) {
            rtsp_tx_video(g_rtsp_session155, (const uint8_t *)enc_data155, enc_data_size155,frame_index155);
            rtsp_do_event(g_rtsplive155);
        }
    }
    memset(enc_data155, 0, enc_buf_size155);
    enc_data_size155 = mpp_encoder155->Encode(mpp_frame155, enc_data155, enc_buf_size155);
    if (g_rtsplive155 && g_rtsp_session155) {
        rtsp_tx_video(g_rtsp_session155, (const uint8_t *)enc_data155, enc_data_size155,frame_index155);
        rtsp_do_event(g_rtsplive155);
    }
    
        // SDL_UpdateTexture(texture, NULL, image_res2->data,image_res2->step);
        //  SDL_RenderClear(renderer);
        // SDL_RenderCopy(renderer, texture, NULL, NULL);
        // SDL_RenderPresent(renderer);
          
      }
    }
  };

  // 创建两个线程并启动
  //std::thread thread3(imageCaptureThread);
 //std::thread thread4(imageCaptureThread1);
std::thread thread1(thread_func1);
  std::thread thread2(thread_func2);

 //std::thread displayThread(displayThreadFunc);
  // 等待两个线程结束
 thread1.join();
  thread2.join();
 // thread3.join();
 //thread4.join();
//displayThread.join();
  rknn_pool1.reset();
  rknn_pool2.reset();
  // 清理资源
// delete[] mpp_frame_addr;
// delete[] mpp_frame;
 
// SDL_DestroyTexture(texture);
// SDL_DestroyRenderer(renderer);
// SDL_DestroyWindow(window);
// SDL_DestroyTexture(texture1);
// SDL_DestroyRenderer(renderer1);
// SDL_DestroyWindow(window1);
// SDL_Quit();
  //cv::destroyAllWindows();
  return 0;
}
