#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include "yolov8.h"
#include <thread>
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
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
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
#include "ffstream.h"
#include "ffmpeg.h"
#include "location.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "mpprtspdecoder.h"
// #include <QCoreApplication>

#define OUT_VIDEO_PATH "out.h264"
#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
// init rtsp class
// rtsp_demo_handle g_rtsplive155 = NULL;
// static rtsp_session_handle g_rtsp_session155;

// rtsp_demo_handle g_rtsplive163 = NULL;
// static rtsp_session_handle g_rtsp_session163;
rtsp_demo_handle g_rtsplive177 = NULL;
static rtsp_session_handle g_rtsp_session177;

void setThreadAffinity(std::thread &thread, int core1, int core2)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core1, &cpuset);
  CPU_SET(core2, &cpuset);

  pid_t tid = syscall(SYS_gettid); // 获取线程的本地ID

  if (sched_setaffinity(tid, sizeof(cpu_set_t), &cpuset) == -1)
  {
    std::cerr << "Failed to set CPU affinity for thread." << std::endl;
    // 可以添加错误处理代码
  }
}
void setThreadAffinity2(std::thread &thread, int core3, int core4)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core3, &cpuset);
  CPU_SET(core4, &cpuset);

  pid_t tid = syscall(SYS_gettid); // 获取线程的本地ID

  if (sched_setaffinity(tid, sizeof(cpu_set_t), &cpuset) == -1)
  {
    std::cerr << "Failed to set CPU affinity for thread." << std::endl;
    // 可以添加错误处理代码11
  }
}

typedef struct
{
  FILE *out_fp;

  MppDecoder *decoder;
  // MppEncoder *encoder;
} mpp_dec_enc;
struct ProgramOptions
{
  std::string model_path = "/home/linaro/rknn_model_zoo-main/examples/yolov8/model/next-n.rknn";
  std::string label_path = "/home/linaro/rknn_model_zoo-main/examples/yolov8/model/dyt.txt";
  int thread_count1 = 6;
  int thread_count2 = 12;
  int camera_index = 0;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert  !  queue ! appsink";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.163/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink  ";
  std::string rtsp_url3 = "rtspsrc location=rtsp://admin:admin@192.168.1.13/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 max-threads=3 ! videoconvert n-threads=3 !  queue ! appsink ";
  std::string rtsp_url4 = "rtspsrc location=rtsp://192.168.2.119/554 latency=200 drop-on-latency=true ! rtph264depay !   h264parse !   avdec_h264   ! videoconvert  n-threads=2  ! appsink   ";
   std::string rtsp_url5 = "filesrc location=/home/linaro/test/DJI_0354.mp4 ! qtdemux !   h264parse !   avdec_h264  max-threads=2 ! videoconvert  n-threads=2  ! appsink   ";
  int width = 1920;
  int height = 1080;
  double fps = 25.0;
  int width2 = 1920;
  int height2 = 1080;
  double fps2 = 25.0;
};
// 声明一个用于存储图像数据的缓冲区abmax-threads=3
// double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }
std::deque<std::unique_ptr<cv::Mat>> imageBuffer2;
std::mutex bufferMutex2;
std::condition_variable bufferCondition2;
std::deque<std::unique_ptr<cv::Mat>> imageBuffer1;
std::mutex bufferMutex1;
std::condition_variable bufferCondition1;

int open_serial_port(const char *port)
{
  int fd;
  struct termios options;

  // 打开串口设备
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    perror("open_port: Unable to open port");
    return -1;
  }

  // 获取当前串口参数
  tcgetattr(fd, &options);

  // 设置波特率为115200
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // 设置数据位、停止位、奇偶校验位等
  options.c_cflag &= ~PARENB;        // 禁用奇偶校验
  options.c_cflag &= ~CSTOPB;        // 设置为一位停止位
  options.c_cflag &= ~CSIZE;         // 清除数据位设置
  options.c_cflag |= CS8;            // 设置数据位为8位
  options.c_cflag |= CREAD | CLOCAL; // 使能接收和本地连接模式

  // 设置输入模式（非规范模式）
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // 清空输出队列，然后应用新的设置1
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);

  return fd;
}
void print_hex(const char *prefix, const unsigned char *data, int len)
{
  int i;
  printf("%s", prefix);
  for (i = 0; i < len; ++i)
  {
    printf("%02X ", data[i]);
  }
  printf("\n");
}
Vector3d obj_locate(vector<one_info> vec)
{
  int n = vec.size();
  MultiView mv;
  vector<Vector2d> uv1;
  for (int i = 0; i < n; i++)
  {
    SingleView sv(vec[i].cam.K);
    // 世界->机体
    sv.addTranslation(vec[i].uav.xyz_world);
    sv.addRotationXYZ(vec[i].uav.roll, vec[i].uav.pitch, vec[i].uav.yaw, true);
    // 机体->相机
    sv.addTranslation(vec[i].cam.xyz_uav);
    sv.addRotationXYZ(vec[i].cam.roll, vec[i].cam.pitch, vec[i].cam.yaw, true);
    mv.addView(sv);
    uv1.push_back(vec[i].uv);
  }
  return mv.multiViewLocate(uv1);
}
one_info get_position(const unsigned char *data, int len)
{
  uav_state u = {};
  memcpy(&u, &data[6], sizeof(u));
  uav_info uav;
  uav.xyz_world << u.lng / 1e7, u.lat / 1e7, u.gps_h / 1e2;
  uav.roll = (double)u.roll_uav / 1e2;
  uav.pitch = (double)u.pitch_uav / 1e2;
  uav.yaw = (double)u.yaw_uav / 1e2;
  cam_info cam;
  cam.xyz_uav << 5.0, 0, 0;
  cam.roll = u.roll_dyt / 1e2;
  cam.pitch = u.pitch_dyt / 1e2;
  cam.yaw = u.yaw_dyt / 1e2;
  cam.K << 1, 0, 960,
      0, 2, 540,
      0, 0, 1;
  one_info res;
  res.cam = cam;
  res.uav = uav;
  return res;
}
ProgramOptions options;
auto rknn_pool3 = std::make_unique<RknnPool>(
    options.model_path, options.thread_count2, options.label_path);

ImageProcess image_process3(options.width2, options.height2, 640);
std::unique_ptr<cv::Mat> image3;
std::shared_ptr<cv::Mat> image_res3;
static int image_count3 = 0;
static int image_res_count3 = 0;
void *mpp_frame177 = NULL;
int mpp_frame_fd177 = 0;
void *mpp_frame_addr177 = NULL;
int enc_data_size177;
static int frame_index177 = 0;

int main()
{
  ProgramOptions options;
  mpp_dec_enc mpp_dec_enc_;
  int video_type = 264;

  auto camera3 = std::make_unique<Camera>(
      options.rtsp_url5, cv::Size(options.width, options.height),
      options.fps);
  auto rknn_pool3 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);

  ImageProcess image_process3(options.width2, options.height2, 640);
  std::unique_ptr<cv::Mat> image1;
  std::shared_ptr<cv::Mat> image_res1;
  std::unique_ptr<cv::Mat> image2;
  std::shared_ptr<cv::Mat> image_res2;
  std::unique_ptr<cv::Mat> image3;
  std::shared_ptr<cv::Mat> image_res3;
  cv::Mat result1, result2, yuantu1, yuantu2;

  static int image_count3 = 0;
  static int image_res_count3 = 0;
  static int camera_count = 0;
  static int inference_count = 0;

  struct timeval time1;
  gettimeofday(&time1, nullptr);
  long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
  struct timeval time2;
  gettimeofday(&time2, nullptr);
  long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
  struct timeval start_time, stop_time;
  // cv::VideoCapture cap177(options.rtsp_url4, cv::CAP_GSTREAMER);
  // // 检查摄像头是否成功打开1
  // if (!cap177.isOpened())
  // {
  //   std::cerr << "Error: Cannot open RTSP177 stream" << std::endl;
  //   return -1;
  // }
  // cv::Mat frame_rtsp177;


  auto imageCaptureThread1 = [&]()
  {
    while (1)
    {
      auto image1 = camera3->GetNextFrame();
      if (image1 != nullptr)
      {
        // 加锁以向缓冲区添加图像1
        std::unique_lock<std::mutex> lock(bufferMutex1);
        imageBuffer1.push_back(std::move(image1));
        lock.unlock();
        // 通知等待的线程有新的图像可用
        bufferCondition1.notify_one();
      }
    }
  };



  // 线程函数3，处理第3个摄像头
  auto thread_func3 = [&]()
  {
    // init value
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
    enc_params177.hor_stride = 1920 * 3;
    enc_params177.ver_stride = 1088;
    enc_params177.fmt = MPP_FMT_BGR888;       // MPP_FMT_BGR888  MPP_FMT_YUV420SP  MPP_FMT_RGB888
    enc_params177.type = MPP_VIDEO_CodingAVC; // H264MPP_VIDEO_CodingHEVC  MPP_VIDEO_CodingAVC
    mpp_encoder177->Init(enc_params177, NULL);

    mpp_frame177 = mpp_encoder177->GetInputFrameBuffer();
    mpp_frame_fd177 = mpp_encoder177->GetInputFrameBufferFd(mpp_frame177);
    mpp_frame_addr177 = mpp_encoder177->GetInputFrameBufferAddr(mpp_frame177);
    //  cv::Mat bgr_frame(enc_params.height ,enc_params.width,CV_8UC3,mpp_frame_addr);
    int enc_buf_size177 = mpp_encoder177->GetFrameSize();
    char *enc_data177 = (char *)malloc(enc_buf_size177);

    while (1)
    {

      //cap177 >> frame_rtsp177; // 读取帧
      //std::unique_ptr<cv::Mat> image_ptr177 = std::make_unique<cv::Mat>(frame_rtsp177);

      // 计算持续时间

      // auto starttime = std::chrono::high_resolution_clock::now();
      std::unique_lock<std::mutex> lock(bufferMutex1);
      bufferCondition1.wait(lock, [&]
                            { return !imageBuffer1.empty(); });
      auto image_ptr177 = std::move(imageBuffer1.front());
      imageBuffer1.pop_front();
      lock.unlock();
      // auto starttime = std::chrono::high_resolution_clock::now();
      if (image_ptr177 != nullptr)
      {
        // printf("image_count2:%d \n", image_count3);

        rknn_pool3->AddInferenceTask(std::move(image_ptr177), image_process3);
        image_count3++;
      }

      auto image_res3 = rknn_pool3->GetImageResultFromQueue();
      //  auto endtime = std::chrono::high_resolution_clock::now();
      // // 计算持续时间宽度: 2560  高度: 1440 4M
      // std::chrono::duration<double,std::milli> duration = endtime - starttime;
      // printf("camera use time: \t%f ms\n", duration.count());
      //   auto endtime = std::chrono::high_resolution_clock::now();
      //  std::chrono::duration<double,std::milli> duration = endtime - starttime;
      // printf("循环里面取推理结果用时: \t%f ms\n", duration.count());
      if (image_res3 != nullptr)
      {
        image_res_count3++;
        auto result3 = *image_res3;
        // std::string save_dir2 = "/home/linaro/datasets";
        // std::string imagesavepath2=save_dir2+"/"+std::to_string(image_res_count3)+".jpg";
        // cv::imwrite(imagesavepath2,result3);
        //  printf("image_res_count2:%d \n", image_res_count3);
        //    cv::imshow("Video3", *image_res3);
        //  cv::waitKey(1);
        memcpy(mpp_frame_addr177, image_res3->data, 1920 * 1080 * 3);
        if (frame_index177 == 1)
        {
          enc_data_size177 = mpp_encoder177->GetHeader(enc_data177, enc_buf_size177);
          if (g_rtsplive177 && g_rtsp_session177)
          {
            rtsp_tx_video(g_rtsp_session177, (const uint8_t *)enc_data177, enc_data_size177, frame_index177);
            rtsp_do_event(g_rtsplive177);
          }
        }
        memset(enc_data177, 0, enc_buf_size177);
        enc_data_size177 = mpp_encoder177->Encode(mpp_frame177, enc_data177, enc_buf_size177);
        if (g_rtsplive177 && g_rtsp_session177)
        {
          rtsp_tx_video(g_rtsp_session177, (const uint8_t *)enc_data177, enc_data_size177, frame_index177);
          rtsp_do_event(g_rtsplive177);
        }
      }
    }
  };

  auto thread_location = [&]()
  {
    int fd;
    unsigned char buffer[512];
    int buffer_len = 0;
    // 打开串口设备 /dev/ttyS3
    fd = open_serial_port("/dev/ttyS3");
    if (fd == -1)
    {
      fprintf(stderr, "Failed to open serial port.\n");
      return 1;
    }
    // 读取和发送串口数据
    printf("Waiting for data...\n");
    while (1)
    {
      // 尝试从串口读取数据
      int bytes_read = read(fd, buffer + buffer_len, sizeof(buffer) - buffer_len - 1);
      one_info o1 = {};
      if (bytes_read > 0)
      {
        buffer_len += bytes_read;
        // 寻找起始字节 0x90 和 0xEB
        int start_index = -1;
        for (int i = 0; i <= buffer_len - 2; ++i)
        {
          if (buffer[i] == 0x90 && buffer[i + 1] == 0xEB)
          {
            start_index = i;
            break;
          }
        }

        if (start_index != -1)
        { // 找到了起始字节
          if (buffer_len - start_index >= 63)
          {
            //  printf("Received data (63 bytes):\n");
            //  print_hex("", buffer + start_index, 63);
            o1 = get_position(buffer + start_index, 63);
            cout << "无人机坐标：" << o1.uav.xyz_world << endl
                 << "roll:" << o1.uav.roll << "pitch:" << o1.uav.pitch << "yaw:" << o1.uav.yaw << endl;
            // uav_state u = {};
            // memcpy(&u, &buffer[start_index + 6], sizeof(u));
            // printf("经度：%f,纬度：%f,高度：%f \n", u.lat / 1e7, u.lng / 1e7, u.gps_h / 1e2);
            // printf("roll:%f,pitch:%f,yaw: %f \n", u.roll_uav / 1e2, u.picth_uav / 1e2, u.yaw_uav / 1e2);
            memmove(buffer, buffer + start_index + 63, buffer_len - start_index - 63);
            buffer_len -= (start_index + 63);
          }
        }
        else
        {
          // 如果没有找到起始字节，则丢弃缓冲区中的数据
          buffer_len = 0;
        }
        //  buffer[bytes_read] = '\0';
        // if (buffer_len >= 63)
        // {
        //   printf("Received data (63 bytes):\n");
        //   print_hex("", buffer, 63);

        //   o1 = get_position(buffer, 63);
        //   //uav_state u = {};
        //   //memcpy(&u, &buffer[6], sizeof(u));
        //   //printf("经度：%d,纬度：%d,高度：%d \n", u.lat / 1e7, u.lng / 1e7, u.gps_h / 1e2);
        //   // cout<<"无人机坐标："<<endl<<o1.uav.xyz_world<<endl
        //   // <<"roll:"<<o1.uav.roll<<"pitch:"<<o1.uav.pitch<<"yaw:"<<o1.uav.yaw<<endl;
        //   memmove(buffer, buffer + 63, buffer_len - 63);
        //   buffer_len -= 63;
        // }
      }
      // 发送数据到串口
      //  printf("Enter message to send (Ctrl+C to exit): ");
      // fgets(buffer, sizeof(buffer), stdin);
      // int bytes_written = write(fd, buffer, strlen(buffer));
      // if (bytes_written < 0) {
      //    fprintf(stderr, "Error writing to serial port.\n");
      //    break;
      // }

      auto ob1 = rknn_pool3->GetObjResultFromQueue();
      if (ob1)
      {

        for (int i = 0; i < ob1->count; ++i)
        {
          int u1 = (ob1->results[i].box.left + ob1->results[i].box.right) / 2;
          int v1 = (ob1->results[i].box.top + ob1->results[i].box.bottom) / 2;
          o1.uv << u1, v1;
          std::vector<one_info> O = {o1};
          Vector3d result_location = obj_locate(O);
          // std::cout << "position:" << std::endl
          //          << result_location << std::endl;
          // std::cout << std::endl;
        }
      }
    };

    // 关闭串口设备
    close(fd);
  };
  // 创建两个线程并启动1
  std::thread thread1(imageCaptureThread1);
  std::thread thread2(thread_func3);
  std::thread thread3(thread_location);
  // 等待两个线程结束

  // setThreadAffinity(thread1, 1, 2);
  setThreadAffinity2(thread2, 3, 4);
  thread1.join();
  thread2.join();
  thread3.join();
  // displayThread.join();
  // rknn_pool1.reset();
  // rknn_pool2.reset();
  rknn_pool3.reset();
  // 清理资源1

  return 0;
}
