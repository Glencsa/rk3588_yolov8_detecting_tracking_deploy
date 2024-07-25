#include <thread>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <dlfcn.h>
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
#include <SDL2/SDL.h>
#include "mpp_decoder.h"
#include "mpp_encoder.h"
#include "drawing.h"
#include "mk_mediakit.h"
#include "im2d.h"
#include "rga.h"
#include "RgaUtils.h"
//#include "mk_mediakit.h"
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


typedef struct 
{
  //mpp_dec_enc():out_fp(nullptr),decoder(nullptr),encoder(nullptr),window1(nullptr),renderer1(nullptr),texture1(nullptr){}
  FILE *out_fp;
 
  MppDecoder *decoder;
  MppEncoder *encoder;
  SDL_Window* window1 ;
    SDL_Renderer* renderer1 ;
    SDL_Texture* texture1;

std::unique_ptr<RknnPool> rknn_pool1;
 //ImageProcess image_process2;

}mpp_dec_enc;

typedef struct
{
  int width;
  int height;
  int width_stride;
  int height_stride;
  int format;
  char *virt_addr;
  int fd;
} image_frame_t;

struct ProgramOptions {
  std::string model_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/yolov8s.rknn";
  std::string label_path = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/coco_80_labels_list.txt";
  int thread_count = 9;
  int camera_index = 21;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=1 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.163/ latency=1 ! rtph265depay !  h265parse !  avdec_h265 ! videoconvert !  queue ! appsink ";
  int width = 1920;
  int height = 1080;
  double fps = 20.0;
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



void API_CALL on_track_frame_out(void *user_data, mk_frame frame)
{
  mpp_dec_enc *ctx =  (mpp_dec_enc *)user_data;
  printf("on_track_frame_out ctx=%p\n", ctx);
  const char *data = mk_frame_get_data(frame);
  size_t size = mk_frame_get_data_size(frame);
  printf("decoder=%p\n", ctx->decoder);
  ctx->decoder->Decode((uint8_t *)data, size, 0);
// 将解码后的数据放入队列
  // std::vector<uint8_t> ctx->decoder;
  // ctx->frame_queue.Push(decoded_data);
}

void API_CALL on_mk_play_event_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[],
                                    int track_count)
{
  mpp_dec_enc *ctx = (mpp_dec_enc *)user_data;
  if (err_code == 0)
  {
    // success
    printf("play success!");
    int i;
    for (i = 0; i < track_count; ++i)
    {
      if (mk_track_is_video(tracks[i]))
      {
        log_info("got video track: %s", mk_track_codec_name(tracks[i]));
        // 监听track数据回调
        mk_track_add_delegate(tracks[i], on_track_frame_out, user_data);
      }
    }
  }
  else
  {
    printf("play failed: %d %s", err_code, err_msg);
  }
}

void API_CALL on_mk_shutdown_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[], int track_count)
{
  printf("play interrupted: %d %s", err_code, err_msg);
}

int process_video_rtsp(mpp_dec_enc *ctx, const char *url)
{
  mk_config config;
  memset(&config, 0, sizeof(mk_config));
  config.log_mask = LOG_CONSOLE;
  mk_env_init(&config);
  mk_player player = mk_player_create();
  mk_player_set_on_result(player, on_mk_play_event_func, ctx);
  mk_player_set_on_shutdown(player, on_mk_shutdown_func, ctx);
  mk_player_play(player, url);

  printf("enter any key to exit\n");
  getchar();

  if (player)
  {
    mk_player_release(player);
  }
  return 0;
}
void mpp_decoder_frame_callback(void *userdata, int width_stride, \
  int height_stride, int width, int height, int format, int fd, void *data)
{

  mpp_dec_enc *ctx = (mpp_dec_enc *)userdata;

  int ret = 0;
  static int frame_index = 0;
  frame_index++;

  void *mpp_frame = NULL;
  int mpp_frame_fd = 0;
  void *mpp_frame_addr = NULL;
  int enc_data_size;

  rga_buffer_t origin;
  rga_buffer_t src;
   std::shared_ptr<cv::Mat> image_res1;
  // if (ctx->encoder == NULL)
  // {
  //   MppEncoder *mpp_encoder = new MppEncoder();
  //   MppEncoderParams enc_params;
  //   memset(&enc_params, 0, sizeof(MppEncoderParams));
  //   enc_params.width = width;
  //   enc_params.height = height;
  //   enc_params.hor_stride = width_stride;
  //   enc_params.ver_stride = height_stride;
  //   enc_params.fmt = MPP_FMT_YUV420SP;
  //   // enc_params.type = MPP_VIDEO_CodingHEVC;
  //   // Note: rk3562只能支持h264格式的视频流
  //   enc_params.type = MPP_VIDEO_CodingAVC;
  //   mpp_encoder->Init(enc_params, NULL);

  //   ctx->encoder = mpp_encoder;
  // }

  // int enc_buf_size = ctx->encoder->GetFrameSize();
  // char *enc_data = (char *)malloc(enc_buf_size);

  image_frame_t img;
  img.width = width;
  img.height = height;
  img.width_stride = width_stride;
  img.height_stride = height_stride;
  img.fd = fd;
  img.virt_addr = (char *)data;
  img.format = RK_FORMAT_YCbCr_420_SP;  //NV12
  cv::Mat image(height, width, CV_8UC3, (unsigned char*)data);
  std::unique_ptr<cv::Mat> image_ptr = std::make_unique<cv::Mat>(image);
   cv::Mat& managedImage = *image_ptr;
   std::cout << "Image width: " << managedImage.cols << std::endl;
   std::cout << "Image height: " << managedImage.rows << std::endl;
   cv::imshow("Video23", *image_ptr); 
  cv::waitKey(1);
  // detect_result_group_t detect_result;
  // memset(&detect_result, 0, sizeof(detect_result_group_t));

  // ret = inference_model(ctx, &img, &detect_result);
  // if (ret != 0)
  // {
  //   printf("inference model fail\n");
  //   goto RET;
  // }

  // mpp_frame = ctx->encoder->GetInputFrameBuffer();
  // mpp_frame_fd = ctx->encoder->GetInputFrameBufferFd(mpp_frame);
  // mpp_frame_addr = ctx->encoder->GetInputFrameBufferAddr(mpp_frame);

  // // Copy To another buffer avoid to modify mpp decoder buffer
  // origin = wrapbuffer_fd(fd, width, height, RK_FORMAT_YCbCr_420_SP, width_stride, height_stride);
  // src = wrapbuffer_fd(mpp_frame_fd, width, height, RK_FORMAT_YCbCr_420_SP, width_stride, height_stride);
  // imcopy(origin, src);

  // Draw objects
  // for (int i = 0; i < detect_result.count; i++)
  // {
  //   detect_result_t *det_result = &(detect_result.results[i]);
  //   printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
  //          det_result->box.right, det_result->box.bottom, det_result->prop);
  //   int x1 = det_result->box.left;
  //   int y1 = det_result->box.top;
  //   int x2 = det_result->box.right;
  //   int y2 = det_result->box.bottom;
  //   draw_rectangle_yuv420sp((unsigned char *)mpp_frame_addr, width_stride, height_stride, x1, y1, x2 - x1 + 1, y2 - y1 + 1, 0x00FF0000, 4);
  // }
// if (frame_index == 1)
//   {
//     enc_data_size = ctx->encoder->GetHeader(enc_data, enc_buf_size);
//     fwrite(enc_data, 1, enc_data_size, ctx->out_fp);
//   }
//   memset(enc_data, 0, enc_buf_size);
//   enc_data_size = ctx->encoder->Encode(mpp_frame, enc_data, enc_buf_size);
//   fwrite(enc_data, 1, enc_data_size, ctx->out_fp);

//   if (g_rtsplive && g_rtsp_session) {
//     rtsp_tx_video(g_rtsp_session, (const uint8_t *)enc_data, enc_data_size,frame_index);
//     rtsp_do_event(g_rtsplive);
//   }

// RET:
//   if (enc_data != nullptr)
//   {
//     free(enc_data);
//   }
}
int main() {
  ProgramOptions options;
    mpp_dec_enc mpp_dec_enc_;
    int video_type = 264;
  char *video_name="rtsp://admin:admin@192.168.1.155/";
// 初始化SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("无法初始化SDL: %s", SDL_GetError());
        return -1;
    } 


if (mpp_dec_enc_.decoder == NULL)
  {
    MppDecoder *decoder = new MppDecoder();
    decoder->Initt(video_type, options.fps, &mpp_dec_enc_);
     decoder->SetCallback(mpp_decoder_frame_callback);
    mpp_dec_enc_.decoder = decoder;
  }

  
   process_video_rtsp(&mpp_dec_enc_, video_name);



  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count, options.label_path);
  auto rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count, options.label_path);
  auto camera1 = std::make_unique<Camera>(
      options.rtsp_url2, cv::Size(options.width, options.height),
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
//  // 创建窗口和渲染器1
    SDL_Window* window = SDL_CreateWindow("Video2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STATIC, 1920, 1080);
     SDL_Window* window1 = SDL_CreateWindow("Video1", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
    SDL_Renderer* renderer1 = SDL_CreateRenderer(window1, -1, SDL_RENDERER_SOFTWARE);
    SDL_Texture* texture1 = SDL_CreateTexture(renderer1, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STATIC, 1920, 1080);
//   //cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
//  // cv::namedWindow("Video2", cv::WINDOW_AUTOSIZE); 
    
//   static int image_count1 = 0;
//   static int image_res_count1 = 0;
//   static int image_count2 = 0;
//   static int image_res_count2 = 0;
//   TimeDuration time_duration;
//   Timeout timeout(std::chrono::seconds(30));
//   TimeDuration total_time;
// struct timeval time1;
//   gettimeofday(&time1, nullptr);
//   long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
//    struct timeval time2;
//   gettimeofday(&time2, nullptr);
//   long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;

// // 新建一个线程用于获取图像并将其放入缓冲区
// auto imageCaptureThread = [&]() {
//   while (1) {
//     auto image2 = camera2->GetNextFrame();
//     if (image2 != nullptr) {
//       // 加锁以向缓冲区添加图像
//       std::unique_lock<std::mutex> lock(bufferMutex2);
//       imageBuffer2.push_back(std::move(image2));
//       lock.unlock();
//       // 通知等待的线程有新的图像可用
//       bufferCondition2.notify_one();
//     }
//   }
// };
// auto imageCaptureThread1 = [&]() {
//   while (1) {
//     auto image1 = camera1->GetNextFrame();
//     if (image1 != nullptr) {
//       // 加锁以向缓冲区添加图像1
//       std::unique_lock<std::mutex> lock(bufferMutex1);
//       imageBuffer1.push_back(std::move(image1));
//       lock.unlock();
//       // 通知等待的线程有新的图像可用
//       bufferCondition1.notify_one();
//     }
//   }
// };
//   // 线程函数1，处理第一个摄像头
//   auto thread_func1 = [&]() {
//     while ((!timeout.isTimeout()) || (image_count1 != image_res_count1)) {
//       // 处理第一个摄像头的帧
//        std::unique_lock<std::mutex> lock(bufferMutex1);
//     bufferCondition1.wait(lock, [&] { return !imageBuffer1.empty(); });
//     auto image1 = std::move(imageBuffer1.front());
//     imageBuffer1.pop_front();
//     lock.unlock();
//     //  image1 = camera1->GetNextFrame();
//       if (image1 != nullptr) {
//         rknn_pool1->AddInferenceTask(std::move(image1), image_process);
//         image_count1++;
//       }

//       image_res1 = rknn_pool1->GetImageResultFromQueue();
//       if (image_res1 != nullptr) {
//         image_res_count1++;

//         if (image_res_count1 % 60 == 0) {
//           gettimeofday(&time1, nullptr);
//           tmpTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
//           printf("第一个摄像头60帧平均帧率1:\t%f帧\n", 60000.0 / (float)(tmpTime1 - lopTime1));
//           lopTime1 = tmpTime1;
//         }
//         // cv::imshow("Video", *image_res1);
//         // cv::waitKey(1);
//         SDL_UpdateTexture(texture1, NULL, image_res1->data, image_res1->step);
//          SDL_RenderClear(renderer1);
//         SDL_RenderCopy(renderer1, texture1, NULL, NULL);
//         SDL_RenderPresent(renderer1);
//       }
//     }
//   };

//   // 线程函数2，处理第二个摄像头
//   auto thread_func2 = [&]() {
//     while ((!timeout.isTimeout()) || (image_count2 != image_res_count2)) {
//       // 处理第二个摄像头的帧
      

//     std::unique_lock<std::mutex> lock(bufferMutex2);
//     bufferCondition2.wait(lock, [&] { return !imageBuffer2.empty(); });
//     auto image2 = std::move(imageBuffer2.front());
//     imageBuffer2.pop_front();
//     lock.unlock();
//       //auto image2 = camera2->GetNextFrame();
//       if (image2 != nullptr) {
//         rknn_pool2->AddInferenceTask(std::move(image2), image_process2);
//         image_count2++;
//       }

//       auto image_res2 = rknn_pool2->GetImageResultFromQueue();
//       if (image_res2 != nullptr) {
//         image_res_count2++;

//         if (image_res_count2 % 60 == 0) {
//           gettimeofday(&time2, nullptr);
//           tmpTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
//           printf("第二个摄像头60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime2 - lopTime2));
//           lopTime2 = tmpTime2;
//         }
//         // cv::imshow("Video2", *image_res2);
//         // cv::waitKey(1);
//        // 将图像数据复制到SDL纹理
//         SDL_UpdateTexture(texture, NULL, image_res2->data, image_res2->step);
//          SDL_RenderClear(renderer);
//         SDL_RenderCopy(renderer, texture, NULL, NULL);
//         SDL_RenderPresent(renderer);
//       }
//     }
//   };

//   // 创建两个线程并启动
//   std::thread thread1(thread_func1);
//   std::thread thread2(thread_func2);
// std::thread thread3(imageCaptureThread);
//  std::thread thread4(imageCaptureThread1);
//  //std::thread displayThread(displayThreadFunc);
//   // 等待两个线程结束
//   thread1.join();
//   thread2.join();
//   thread3.join();
//   thread4.join();
// //displayThread.join();
  rknn_pool1.reset();
  rknn_pool2.reset();
//   // 清理资源
SDL_DestroyTexture(texture);
SDL_DestroyRenderer(renderer);
SDL_DestroyWindow(window);
SDL_DestroyTexture(texture1);
SDL_DestroyRenderer(renderer1);
SDL_DestroyWindow(window1);
SDL_Quit();
  //cv::destroyAllWindows();
  return 0;
}
