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
#define OUT_VIDEO_PATH "out.h264"
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
//init rtsp class
rtsp_demo_handle g_rtsplive155 = NULL;
static rtsp_session_handle g_rtsp_session155;

rtsp_demo_handle g_rtsplive163 = NULL;
static rtsp_session_handle g_rtsp_session163;
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

  pid_t tid = syscall(SYS_gettid); // 获取线程的本地ID11

  if (sched_setaffinity(tid, sizeof(cpu_set_t), &cpuset) == -1)
  {
    std::cerr << "Failed to set CPU affinity for thread." << std::endl;
    // 可以添加错误处理代码1
  }
}

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

  // 清空输出队列，然后应用新的设置
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
  uav.xyz_world << u.lng / 1e7,u.lat / 1e7 , u.gps_h / 1e2;
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
typedef struct 
{
  FILE *out_fp;
 
  MppDecoder *decoder;
  //MppEncoder *encoder;
}mpp_dec_enc;
struct ProgramOptions {
  std::string model_path = "/home/linaro/rknn_model_zoo-main/examples/yolov8/model/yolov8.rknn";
  std::string label_path = "/home/linaro/rknn_model_zoo-main/examples/yolov8/model/coco_80_labels_list.txt";
  int thread_count1 = 6;
  int thread_count2 =18;
  int camera_index = 0;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert  !  queue ! appsink";
  std::string rtsp_url2 = "rtspsrc location=rtsp://admin:admin@192.168.1.163/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink  ";
   std::string rtsp_url3 = "rtspsrc location=rtsp://admin:admin@192.168.1.177/ latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
   std::string rtsp_url4 = "rtspsrc location=rtsp://192.168.2.119/554 latency=0 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue !  appsink ";
   std::string rtsp_url5 = "filesrc location=/home/linaro/test/testDJI.mp4 ! qtdemux !   h264parse !   avdec_h264   ! videoconvert  n-threads=2  ! appsink   ";
  int width = 1280;
  int height = 720;
  double fps = 24.0;
  int width2 = 1280;
  int height2 = 720;
  double fps2 = 24.0;
};

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
   mpp_dec_enc mpp_dec_enc_;
   int video_type = 264;
  //  if (mpp_dec_enc_.decoder == NULL)
  // {
  //   MppDecoder *decoder = new MppDecoder();
  //   decoder->Initt(video_type, options.fps, &mpp_dec_enc_);
  //    decoder->SetCallback(mpp_decoder_frame_callback);
  //   mpp_dec_enc_.decoder = decoder;
  // }
  auto rknn_pool1 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count1, options.label_path);
  auto rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);
  auto rknn_pool3 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count2, options.label_path);

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
  static int image_count3 = 0;
  static int image_res_count3 = 0;
  static int camera_count = 0;
  static int inference_count = 0;
//   TimeDuration time_duration;
//   Timeout timeout(std::chrono::seconds(30));
//   TimeDuration total_time;
struct timeval time1;
  gettimeofday(&time1, nullptr);
  long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
   struct timeval time2;
  gettimeofday(&time2, nullptr);
  long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
 struct timeval start_time, stop_time;
  //  std::string path155 = "rtsp://admin:admin@192.168.1.155/";
  //   MppRtspDecoder *mppdec = new MppRtspDecoder{path155};
  //   mppdec->init();
  //   cv::Mat imgshow;
  //   int count = 0;
  //   double cost_all = 0;
  //   cv::Mat img155;
  //   img155.create(cv::Size(1920,1080), CV_8UC3);
// 打开RTSP摄像头  155 camera
  //   cv::VideoCapture cap155(options.rtsp_url, cv::CAP_GSTREAMER);
  //   // 检查摄像头是否成功打开
  //   if (!cap155.isOpened()) {
  //       std::cerr << "Error: Cannot open RTSP155 stream" << std::endl;
  //       return -1;
  //   }
  //   // 获取RTSP摄像头的帧率
  //   double fps = cap155.get(cv::CAP_PROP_FPS);
  //   std::cout << "帧率: " << fps << std::endl;
  //   // 获取RTSP摄像头的宽度和高度
  //   int frame_width = cap155.get(cv::CAP_PROP_FRAME_WIDTH);
  //   int frame_height = cap155.get(cv::CAP_PROP_FRAME_HEIGHT);
  //   std::cout << "宽度: " << frame_width << "  高度: " << frame_height << std::endl;
  //  cv::Mat frame_rtsp155;
   

// 打开RTSP摄像头  163 camera
  //   cv::VideoCapture cap163(options.rtsp_url2, cv::CAP_GSTREAMER);
  //   // 检查摄像头是否成功打开
  //   if (!cap163.isOpened()) {
  //       std::cerr << "Error: Cannot open RTSP163 stream" << std::endl;
  //       return -1;
  //   }
  //  cv::Mat frame_rtsp163;

// 打开RTSP摄像头  177 camera
    cv::VideoCapture cap177(options.rtsp_url5, cv::CAP_GSTREAMER);
    // 检查摄像头是否成功打开
    if (!cap177.isOpened()) {
        std::cerr << "Error: Cannot open RTSP177 stream" << std::endl;
        return -1;
    }
   cv::Mat frame_rtsp177;
#if 0
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
    enc_params163.width = 1280;
    enc_params163.height = 720;
    enc_params163.hor_stride = 1280*3;
    enc_params163.ver_stride = 728;
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
      
     cap163 >> frame_rtsp163; // 读取帧
    
    std::unique_ptr<cv::Mat> image_ptr163 = std::make_unique<cv::Mat>(frame_rtsp163);
      if (image_ptr163 != nullptr) {
        
        rknn_pool1->AddInferenceTask(std::move(image_ptr163), image_process);
        image_count1++;
      }
      image_res1 = rknn_pool1->GetImageResultFromQueue();
      if (image_res1 != nullptr) {
        image_res_count1++;
         memcpy(mpp_frame_addr163,image_res1->data,1280 * 720 * 3);
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
    enc_params155.width = 1280;
    enc_params155.height = 720;
    enc_params155.hor_stride = 1280*3;
    enc_params155.ver_stride = 728;
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
//auto start = std::chrono::system_clock::now();
        // mppdec->decode_simple(img155);
        // count++;

        // auto finish = std::chrono::system_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
        // cost_all += (double)duration.count();
        // printf("frame: %d, cost: %f, avg_cost: %f\n", count, (double)duration.count(), cost_all/count);
        // printf("cols:%d\n", img155.rows);
        // if ( img155.empty() )
        // {
        //     printf("img is empty\n");
        //     continue;
        // }
        
    cap155 >> frame_rtsp155; // 读取帧
    std::unique_ptr<cv::Mat> image_ptr155 = std::make_unique<cv::Mat>(frame_rtsp155);
    // auto starttime = std::chrono::high_resolution_clock::now();
      if (image_ptr155 != nullptr) {
       if (image_count2%2==0){
  //       printf("image_count2:%d \n",image_count2);
         rknn_pool2->AddInferenceTask(std::move(image_ptr155), image_process2);
        }
          image_count2++;  
      }
      auto image_res2 = rknn_pool2->GetImageResultFromQueue();
      //  auto endtime = std::chrono::high_resolution_clock::now(); 
    // // 计算持续时间
      // std::chrono::duration<double,std::milli> duration = endtime - starttime;
      // printf("camera use time: \t%f ms\n", duration.count());
          
      if (image_res2 != nullptr) {
        image_res_count2++;
   //       printf("image_res_count2:%d \n",image_res_count2);
    // Encode to file1
    // Write header on first frame 2304  高度: 1296
    memcpy(mpp_frame_addr155,image_res2->data,1280 * 720 * 3);
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
    //  cv::imshow("Video2", *image_res2); 
        //       cv::waitKey(1);


      }
      // else{

      //   usleep(100*1000);
      // }

       

    }
  };
  #endif
// 线程函数3，处理第3个摄像头
  auto thread_func3 = [&]() {
    //init value
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
    enc_params177.width = 1280;
    enc_params177.height = 720;
    enc_params177.hor_stride = 1280*3;
    enc_params177.ver_stride = 728;
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
     cap177 >> frame_rtsp177; // 读取帧
    std::unique_ptr<cv::Mat> image_ptr177 = std::make_unique<cv::Mat>(frame_rtsp177);
    auto starttime = std::chrono::high_resolution_clock::now();
      if (image_ptr177 != nullptr) {
       //   printf("image_count2:%d \n",image_count3);
  
          rknn_pool3->AddInferenceTask(std::move(image_ptr177), image_process2);
          image_count3++;  
      }
    
      auto image_res3 = rknn_pool3->GetImageResultFromQueue();
       auto endtime = std::chrono::high_resolution_clock::now(); 
    // // 计算持续时间
      // std::chrono::duration<double,std::milli> duration = endtime - starttime;
      // printf("camera use time: \t%f ms\n", duration.count());
            
      if (image_res3 != nullptr) {
        image_res_count3++;
          //printf("image_res_count2:%d \n",image_res_count3);
    memcpy(mpp_frame_addr177,image_res3->data,1280 * 720 * 3);
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
            for (int i = 0; i <= buffer_len - 2; ++i) {
                if (buffer[i] == 0x90 && buffer[i + 1] == 0xEB) {
                    start_index = i;
                    break;
                }
            }

            if (start_index != -1) {  // 找到了起始字节
                if (buffer_len - start_index >= 63) { 
                  //  printf("Received data (63 bytes):\n");
                  //  print_hex("", buffer + start_index, 63);
                    o1=get_position(buffer + start_index, 63);
                    cout<<"无人机坐标："<<o1.uav.xyz_world<<endl
                      <<"roll:"<<o1.uav.roll<<"pitch:"<<o1.uav.pitch<<"yaw:"<<o1.uav.yaw<<endl;
                    // uav_state u = {};
                    // memcpy(&u, &buffer[start_index + 6], sizeof(u));
                    // printf("经度：%f,纬度：%f,高度：%f \n", u.lat / 1e7, u.lng / 1e7, u.gps_h / 1e2);
                    // printf("roll:%f,pitch:%f,yaw: %f \n", u.roll_uav / 1e2, u.picth_uav / 1e2, u.yaw_uav / 1e2);
                    memmove(buffer, buffer + start_index + 63, buffer_len - start_index - 63);
                    buffer_len -= (start_index + 63);
                }
            } else {
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
      // auto ob2 = rknn_pool2->GetObjResultFromQueue();
      // 多机目标定位
      //    if(ob1&&ob2)
      //    {
      //      for(int i =0;i<ob1->count;++i)
      //      {
      //        for(int j = 0;j<ob2->count;++j)
      //        {
      //          if(ob1->results[i].cls_id == ob2->results[j].cls_id)
      //                {
      //                    int u1 = (ob1->results[i].box.left+ob1->results[i].box.right)/2;
      //                    int v1 = (ob1->results[i].box.top+ob1->results[i].box.bottom)/2;
      //                    int u2 = (ob2->results[j].box.left+ob2->results[j].box.right)/2;
      //                    int v2 = (ob2->results[j].box.top+ob2->results[j].box.bottom)/2;
      //                    uav_info uva1,uva2;
      //                    uva1.xyz_world <<48,71,0;
      //                    uva1.yaw = 90;
      //                    uva1.pitch = 0;
      //                    uva1.roll = -45;
      //                    uva2.xyz_world<<18.5,82,0;
      //                    uva2.yaw = 90;
      //                    uva2.pitch = 0;
      //                    uva2.roll = 0;
      //                    cam_info cam1,cam2;
      //                    cam1.xyz_uav <<0,0,0;
      //                    cam1.pitch = 0;
      //                    cam1.roll = 0;
      //                    cam1.yaw = 0;
      //                    cam1.K<<1,0,960,
      //                          0,2,540,
      //                          0,0,1;
      //                    cam2.xyz_uav <<0,0,0;
      //                    cam2.pitch = 0;
      //                    cam2.roll = 0;
      //                    cam2.yaw = 0;
      //                    cam2.K<<500,0,960,
      //                          0,500,540,
      //                          0,0,1;
      //                    one_info o1,o2;
      //                    o1.cam = cam1;
      //                    o1.uav = uva1;
      //                    o1.uv <<u1,v1;
      //                    o2.cam = cam2;
      //                    o2.uav = uva2;
      //                    o2.uv <<u2,v2;
      //                    std::vector<one_info> O = {o1,o2};
      //                    Vector3d result_location = obj_locate(O);
      //                    std::cout<<"position:"<<endl<<result_location<<endl;
      //                    cout<<endl;
      //                }
      //        }
      //      }
      //    }
      //  }

      if (ob1)
      {

        for (int i = 0; i < ob1->count; ++i)
        {
          int u1 = (ob1->results[i].box.left + ob1->results[i].box.right) / 2;
          int v1 = (ob1->results[i].box.top + ob1->results[i].box.bottom) / 2;
          o1.uv << u1, v1;
          std::vector<one_info> O = {o1};
          Vector3d result_location = obj_locate(O);
          std::cout << "position:" << std::endl
                    << result_location << std::endl;
          std::cout << std::endl;
        }
      }
    };

    // 关闭串口设备
    close(fd);
  };
  // 创建两个线程并启动
  //std::thread thread3(imageCaptureThread);
 //std::thread thread4(imageCaptureThread1);
//std::thread thread1(thread_func1);
 // std::thread thread2(thread_func2);
   std::thread thread2(thread_func3);
 std::thread thread3(thread_location);
   setThreadAffinity2(thread2, 3, 4);
 //std::thread displayThread(displayThreadFunc);
  // 等待两个线程结束
 //thread1.join();
  thread2.join();
  thread3.join();
 // thread3.join();
 //thread4.join();
//displayThread.join();
  rknn_pool1.reset();
  rknn_pool2.reset();
  rknn_pool3.reset();
  // 清理资源

  return 0;
}
