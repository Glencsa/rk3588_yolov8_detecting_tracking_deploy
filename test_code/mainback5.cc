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
#include <SDL2/SDL.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

bool isStreaming1=false;
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
  int thread_count1 = 12;
  int thread_count2 = 9;
  int camera_index = 0;
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
// 声明一个用于存储图像数据的缓冲区a
// RTSP 目标地址
const char* rtsp_url = "rtsp://192.168.1.1:554/live/stream";

// FFmpeg 相关变量
AVFormatContext* fmtCtx = nullptr;
AVCodecContext* codecCtx = nullptr;
AVStream* videoStream = nullptr;

// 初始化 FFmpeg
void initFFmpeg() {
    av_register_all();
    avformat_network_init();

    // 创建输出格式上下文
    avformat_alloc_output_context2(&fmtCtx, nullptr, "rtsp", rtsp_url);
    if (!fmtCtx) {
        std::cerr << "Error creating output context" << std::endl;
        return;
    }

    // 查找视频编码器
    AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "Error finding H.264 encoder" << std::endl;
        return;
    }

    // 创建视频流
    videoStream = avformat_new_stream(fmtCtx, codec);
    if (!videoStream) {
        std::cerr << "Error creating video stream" << std::endl;
        return;
    }

    codecCtx = avcodec_alloc_context3(codec);
    if (!codecCtx) {
        std::cerr << "Error allocating codec context" << std::endl;
        return;
    }

    // 设置编码器参数
    codecCtx->codec_id = codec->id;
    codecCtx->bit_rate = 400000;
    codecCtx->width = 640;
    codecCtx->height = 480;
    codecCtx->time_base = {1, 25};
    codecCtx->gop_size = 10;
    codecCtx->max_b_frames = 1;
    codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;

    // 打开编码器
    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        std::cerr << "Error opening codec" << std::endl;
        return;
    }

    // 复制编码器参数到流
    avcodec_parameters_from_context(videoStream->codecpar, codecCtx);

    // 打开输出 URL
    if (!(fmtCtx->flags & AVFMT_NOFILE)) {
        if (avio_open(&fmtCtx->pb, rtsp_url, AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Error opening output URL" << std::endl;
            return;
        }
    }

    // 写入流头部信息
    if (avformat_write_header(fmtCtx, nullptr) < 0) {
        std::cerr << "Error writing header" << std::endl;
        return;
    }

    // 标记为正在推流
    isStreaming1 = true;
}

// 推送图像到 RTSP 流
void pushImageToRTSP(cv::Mat& image) {
    if (!isStreaming1) {
        initFFmpeg();
    }

    AVFrame* frame = av_frame_alloc();
    frame->format = AV_PIX_FMT_BGR24;
    frame->width = image.cols;
    frame->height = image.rows;
    av_image_fill_arrays(frame->data, frame->linesize, image.data, AV_PIX_FMT_BGR24, image.cols, image.rows, 1);

    // 发送帧到编码器
    if (avcodec_send_frame(codecCtx, frame) == 0) {
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = nullptr;
        pkt.size = 0;

        // 从编码器接收编码后的帧并发送到 RTSP 服务器
        while (avcodec_receive_packet(codecCtx, &pkt) == 0) {
            av_interleaved_write_frame(fmtCtx, &pkt);
            av_packet_unref(&pkt);
        }
    }

    av_frame_free(&frame);
}

std::deque<std::unique_ptr<cv::Mat>> imageBuffer2;
std::mutex bufferMutex2;
std::condition_variable bufferCondition2;
std::deque<std::unique_ptr<cv::Mat>> imageBuffer1;
std::mutex bufferMutex1;
std::condition_variable bufferCondition1;
int main() {
  ProgramOptions options;
// 初始化SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("无法初始化SDL: %s", SDL_GetError());
        return -1;
    } 
  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count1, options.label_path);
  auto rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);
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
 // 创建窗口和渲染器
    SDL_Window* window = SDL_CreateWindow("Video2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
    SDL_Window* window1 = SDL_CreateWindow("Video1", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
    SDL_Renderer* renderer1 = SDL_CreateRenderer(window1, -1, SDL_RENDERER_SOFTWARE);
    SDL_Texture* texture1 = SDL_CreateTexture(renderer1, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
  //cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
 // cv::namedWindow("Video2", cv::WINDOW_AUTOSIZE); 
    
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
	// 推送图像到 RTSP 流
            //pushImageToRTSP(image_res1);
        if (image_res_count1 % 60 == 0) {
          gettimeofday(&time1, nullptr);
          tmpTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
          printf("第一个摄像头60帧平均帧率1:\t%f帧\n", 60000.0 / (float)(tmpTime1 - lopTime1));
          lopTime1 = tmpTime1;
        }
       //  cv::imshow("Video", *image_res1);
        // cv::waitKey(1);
        // SDL_UpdateTexture(texture1, NULL, image_res1->data, image_res1->step);
        //  SDL_RenderClear(renderer1);
        // SDL_RenderCopy(renderer1, texture1, NULL, NULL);
        // SDL_RenderPresent(renderer1);
         printf("rtsp is running \n");
          AVFrame *frame = av_frame_alloc();
            frame->width = image_res1->cols;
            frame->height = image_res1->rows;
            frame->format = AV_PIX_FMT_BGR24; // OpenCV的Mat对象格式为BGR24
            av_frame_get_buffer(frame,32);
            //av_image_alloc(frame->data, frame->linesize, frame->width, frame->height, AV_PIX_FMT_BGR24, 32);
            memcpy(frame->data[0], image_res1->data, image_res1->step * image_res1->rows);

            // 初始化AVFormatContext和AVOutputFormat
            AVFormatContext *formatContext = nullptr;
            AVOutputFormat *outputFormat = nullptr;
            avformat_alloc_output_context2(&formatContext, NULL, "rtsp", NULL); // 创建RTSP格式的输出上下文
            avio_open(&formatContext->pb, "rtsp://192.168.1.1:554/live/stream", AVIO_FLAG_WRITE); // 打开RTSP连接
            outputFormat = formatContext->oformat;

            // 创建新的视频流
            if (avformat_new_stream(formatContext, NULL) == NULL) {
                fprintf(stderr, "Error creating new stream.\n");
                return;
            }

            // 获取视频流的编码器
            AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
            if (!codec) {
                printf("Codec not found\n");
                return;
            }
            AVCodecContext *codecContext = avcodec_alloc_context3(codec);
            codecContext->width = frame->width;
            codecContext->height = frame->height;
            codecContext->time_base = {1, 20}; // 设置帧率为30帧/秒
            codecContext->pix_fmt = AV_PIX_FMT_YUV420P;

            // 打开编码器
            avcodec_open2(codecContext, codec, NULL);

            // 编码并发送视频帧到RTSP流
            AVPacket packet;
            av_init_packet(&packet);
            packet.data = NULL;
            packet.size = 0;

            int ret = avcodec_send_frame(codecContext, frame);
            if (ret < 0) {
              printf("Error sending frame for encodingd\n");
                //fprintf(stderr, "Error sending frame for encoding: %s\n", av_err2str(ret));
                return;
            }

            while (ret >= 0) {
                ret = avcodec_receive_packet(codecContext, &packet);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                else if (ret < 0) {
                  printf("Error during encoding\n");
                   // fprintf(stderr, "Error during encoding: %s\n", av_err2str(ret));
                    return;
                }

                // 将编码后的视频帧写入到RTSP流中
                ret = av_write_frame(formatContext, &packet);
                if (ret < 0) {
                   printf("Error d av_write_frame\n");
                    return;
                }
            }

            // 释放资源
            // av_packet_unref(&packet);
            // avformat_close_input(&formatContext);
            // avcodec_free_context(&codecContext);
            // av_frame_free(&frame);
      }
    }
  };

  // 线程函数2，处理第二个摄像头
  auto thread_func2 = [&]() {
    while ((!timeout.isTimeout()) || (image_count2 != image_res_count2)) {
      // 处理第二个摄像头的帧
      

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
        // cv::imshow("Video2", *image_res2);
        // cv::waitKey(1);
       // 将图像数据复制到SDL纹理
        SDL_UpdateTexture(texture, NULL, image_res2->data, image_res2->step);
         SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
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
  // 清理资源
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
