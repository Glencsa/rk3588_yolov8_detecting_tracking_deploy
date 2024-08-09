#include <thread>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include "gst/gst.h"
#include <gst/app/gstappsink.h>
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
#include <queue>
#include <mutex>
#include <condition_variable>

#include <SDL2/SDL.h>
// 全局变量存储图像数据
GstBuffer *global_buffer = nullptr;
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
  int thread_count = 6;
  int camera_index = 0;
  std::string rtsp_url = "rtspsrc location=rtsp://admin:admin@192.168.1.155/ latency=1 ! rtph264depay !  h264parse !  avdec_h264 ! videoconvert !  queue ! appsink ";
  std::string rtsp_url2 = "v4l2src device=/dev/video21 ! image/jpeg, format=NV12,width=640, height=480 ! queue! jpegparse ! queue ! jpegdec ! queue ! videoconvert ! queue ! appsink";
  int width = 640;
  int height = 480;
  double fps = 25.0;
  int width2 = 1920;
  int height2 = 1080;
  double fps2 = 20.0;
};
TimeDuration time_duration;
  Timeout timeout(std::chrono::seconds(30));
  TimeDuration total_time;
struct timeval time1;
  
   struct timeval time2;
  
  static int image_count1 = 0;
  static int image_res_count1 = 0;
  static int image_count2 = 0;
  static int image_res_count2 = 0;
   std::unique_ptr<cv::Mat> image1;
  std::shared_ptr<cv::Mat> image_res1;
  std::unique_ptr<cv::Mat> image2;
  std::shared_ptr<cv::Mat> image_res2;


struct _GSTHANDLE {
    GMainLoop *loop;
    GstBus *bus;
    GstElement *pipeline;
    GstMessage *msg;
};

struct _GSTELES {
    GstElement *source,
               *demuxer,
               *parser,
               *decoder,
               *sink,
               *capabilities,
               *filter,
               *filesink,
               *appsink,
               *videoconvert,
                *rtph264depay,
               *videorate;
               cv::Mat* images2;
        int widths2=1920; 
        int heights2=1080; 
        std::unique_ptr<RknnPool> rknn_pool2;
        ImageProcess image_process2;
        SDL_Window* window ;
    SDL_Renderer* renderer ;
    SDL_Texture* texture ;
     SDL_Window* window1 ;
    SDL_Renderer* renderer1 ;
    SDL_Texture* texture1;
        std::string model_path2 = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/yolov8s.rknn";
        std::string label_path2 = "/home/firefly/cy/yolov8/rk3588-yolo-demo-master/src/yolov8/model/coco_80_labels_list.txt";
        int thread_count = 3;
          FILE *outFile;
};

struct GSTOBJ {
    struct _GSTHANDLE handle;
    struct _GSTELES elements;
};

static struct GSTOBJ *gstobj = NULL;
static bool g_quit = false;


static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
    GMainLoop *loop = (GMainLoop *)data;
    gchar  *debug;
    GError *error;

    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;

    case GST_MESSAGE_ERROR:
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        printf("Error: %s\n", error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
        break;

    default:
        break;
    }
    return true;
}
// 全局变量或者类成员变量
std::mutex displayMutex;
std::condition_variable imageAvailableCond;
bool isImageAvailable = false;
cv::Mat currentImage;

void displayThreadFunc() {
    while (true) {
        // 等待图像可用的通知
        {
            std::unique_lock<std::mutex> lock(displayMutex);
            imageAvailableCond.wait(lock, []{ return isImageAvailable; });
        }

        // 显示图像
        cv::imshow("Video2", currentImage);
        cv::waitKey(1);

        // 重置图像可用标志
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            isImageAvailable = false;
        }
    }
}

static void on_pad_added(GstElement *element, GstPad *pad, gpointer data) {
    GstPad *sinkpad;
    GstElement *parser = (GstElement*)data;
    GstCaps *caps;
    gchar *padname;

    padname = gst_pad_get_name(pad);
    g_print("New pad '%s' detected.\n", padname);
    g_free(padname);

    caps = gst_pad_query_caps(pad, NULL);
    if (!gst_caps_is_fixed(caps)) {
        printf("Pad caps are not fixed.\n");
        return;
    }

    /* Create an element to connect the src pad of the demuxer to the sink pad of the parser */
    sinkpad = gst_element_get_static_pad(parser, "sink");
    if (GST_PAD_IS_LINKED(sinkpad)) {
        printf("Parser sink pad is already linked.\n");
        return;
    }

    if (gst_pad_link(pad, sinkpad) != GST_PAD_LINK_OK) {
        printf("Failed to link qtdemuxer and h264parser.\n");
    }

    gst_object_unref(sinkpad);
    gst_caps_unref(caps);
}

/**
 * signal handler function
 */
static void sig_handle(int signal) {
    printf("\n\n\r\033[k");
    if (gstobj != NULL)
    {
         g_main_loop_quit(gstobj->handle.loop);
    }
    g_quit = true;
    sleep(1);
}

/**
 * initialize gst object
 */
bool initialize_gst(int argc, char **argv, struct _GSTHANDLE *handle) {
    
    if (handle == NULL)
        return false;

    /* Initialization of gstreamer */
    gst_init(&argc, &argv);
    handle->loop = g_main_loop_new(NULL, FALSE);

    /* Create a new pipeline */
    handle->pipeline = gst_pipeline_new("_pipeline");
    if (!handle->pipeline)
        return false;

    return true;
}
void thread_func1(ProgramOptions options, std::unique_ptr<RknnPool> rknn_pool1, std::unique_ptr<Camera> camera1, ImageProcess image_process) {
  gettimeofday(&time1, nullptr);
  long tmpTime1, lopTime1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
  while ((!timeout.isTimeout()) || (image_count1 != image_res_count1)) {
      image1 = camera1->GetNextFrame();
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
}
// 回调函数，用于处理从appsink接收到的数据
static GstFlowReturn new_sample_callback(GstElement *sink, gpointer data) {
    // 获取sample
    GstSample *sample;
    int width,height;
    struct _GSTELES* elements = (struct _GSTELES*) data;
//      gettimeofday(&time2, nullptr);
     
//   long tmpTime2, lopTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
  
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample) {
        // 获取buffer
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (buffer) {
            // 将buffer映射到内存
            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {   
                // FILE *file = (FILE *)data;
                // fwrite(map.data, 1, map.size, file);
                //  printf("image write begin");
            //    cv::Mat mat=cv::Mat (1080,1920,CV_8U, (char*)map.data);
            //    cv::imshow("Video3", mat); 
            

// // // 获取图像类型
// //                 int type = managedImage.type();
//                 //std::cout << "Image type: " << type << std::endl;

// //                 // 获取图像宽高...

// //                 //cv::Mat *imagess2 = new cv::Mat(elements->heights2, elements->widths2, CV_8UC1, map.data);
                cv::Mat image(1080, 1920, CV_8UC3, (unsigned char*)map.data);
                 std::unique_ptr<cv::Mat> image_ptr = std::make_unique<cv::Mat>(image);
                //  cv::Mat& managedImage = *image_ptr;
                // std::cout << "Image width: " << managedImage.cols << std::endl;
                // std::cout << "Image height: " << managedImage.rows << std::endl;
                // cv::imshow("Video23", image); 
                // cv::waitKey(1);
               
                
                 if (image_ptr != nullptr) {
                    elements->rknn_pool2->AddInferenceTask(std::move(image_ptr), elements->image_process2);
                    image_count2++;
                
                }
       
            
                auto image_res2 = elements->rknn_pool2->GetImageResultFromQueue();
               // std::cout << "Image width: " << image_res2->cols << std::endl;
              if (image_res2 != nullptr) {
                image_res_count2++;
                printf("tramsit image");
               {
                std::lock_guard<std::mutex> lock(displayMutex);
                currentImage = *image_res2;
                isImageAvailable = true;
            }
            imageAvailableCond.notify_one();
              SDL_UpdateTexture(elements->texture1, NULL, image_res2->data, image_res2->step);
            SDL_RenderClear(elements->renderer1);
            SDL_RenderCopy(elements->renderer1, elements->texture1, NULL, NULL);
            SDL_RenderPresent(elements->renderer1);
           
                // if (image_res_count2 % 60 == 0) {
                //   gettimeofday(&time2, nullptr);
                //   tmpTime2 = time2.tv_sec * 1000 + time2.tv_usec / 1000;
                //   printf("第二个摄像头60帧平均帧率:\t%f帧\n", 60000.0 / (float)(tmpTime2 - lopTime2));
                //   lopTime2 = tmpTime2;
                // }
                //cv::imshow("Video2", *image_res2); 
               // cv::waitKey(1);
              }
            // delete elements->image2; // 使用完毕后释放内存d
            // elements->image = nullptr;
                //delete image; 
                gst_buffer_unmap(buffer, &map);
            }
        }
        // 释放sample
        gst_sample_unref(sample);
    }

    return GST_FLOW_OK;
}

bool transform(struct _GSTELES *elements){
//    elements->rknn_pool2 = std::make_unique<RknnPool>(elements->model_path2, elements->thread_count,elements->label_path2);
//       elements->image_process2 = ImageProcess(elements->widths2, elements->heights2, 640);
  g_object_set(G_OBJECT(elements->appsink), "emit-signals", TRUE, NULL);
g_signal_connect(G_OBJECT(elements->appsink), "new-sample", G_CALLBACK(new_sample_callback), elements);
//printf("begin transformer\n");
//cv::namedWindow("Video2", cv::WINDOW_AUTOSIZE);
//    elements->outFile = fopen("output1.yuv", "wb");
//   if (elements->outFile == NULL) {
//     perror("Error opening file");
//     return 1;
//   }
// printf("begin transformer\n");
// g_object_set(G_OBJECT(elements->appsink), "emit-signals", TRUE, NULL);
// g_signal_connect(G_OBJECT(elements->appsink), "new-sample", G_CALLBACK(new_sample_callback), elements->outFile);
return true;
}


bool make_gst_elements(struct _GSTELES *elements ) {
    
    // if (params == NULL || elements == NULL)
    //     return false;
 GstCaps *caps;
    /* Initialization of elements */
    elements->source        = gst_element_factory_make("rtspsrc", "_filesrc");
     g_object_set (G_OBJECT(elements->source), "latency", 0, NULL);
     elements->rtph264depay = gst_element_factory_make ("rtph264depay", "depay");
    elements->demuxer       = gst_element_factory_make("matroskademux", "_qtdemux");
    elements->decoder       = gst_element_factory_make("avdec_h264", "_mppvideodec");
    elements->sink          = gst_element_factory_make("filesink", "_xvimagesink");
    elements->videorate     = gst_element_factory_make("videorate", "_videorate");
    elements->appsink       = gst_element_factory_make("appsink", "_appsink");
    elements->videoconvert      = gst_element_factory_make("videoconvert", "_videoconvert");
    //elements->filesink     = gst_element_factory_make("filesink", "_filesink");
    elements->capabilities  = gst_element_factory_make("capsfilter", "_capsfilter");
    elements->parser = gst_element_factory_make("h264parse", "_h264parse");
    g_object_set(G_OBJECT(elements->sink), "location", "output.yuv", NULL);
   // g_object_set(G_OBJECT(elements->source), "location", "/home/firefly/h265output.mkv", NULL);
    g_object_set (elements->source, "location", "rtsp://admin:admin@192.168.1.155", NULL);
    /* 设置需要保存的yuv形式 */
    caps = gst_caps_new_simple("video/x-raw",
                              "format", G_TYPE_STRING, "BGR",
                               "width", G_TYPE_INT, 1920,
                               "height", G_TYPE_INT, 1080,
                              // "framerate", GST_TYPE_FRACTION, 30, 1,
                               NULL);
    g_object_set(G_OBJECT(elements->capabilities), "caps", caps, NULL);
    gst_caps_unref(caps);
    return true;
}

/**
 * check if element was created successfully
 */
bool check_gst_elements(struct _GSTELES *elements) {
        
    if(!elements->source || !elements->demuxer || 
        !elements->decoder || !elements->sink ||
        !elements->parser || !elements->videorate ||  !elements->videoconvert || 
        !elements->capabilities)
    {
        printf("Failed to create elements. Exiting.\n");
        return false;
    }
 
    return TRUE ;
}
/**
 * link elements
 */
bool link_gst_elements(struct _GSTELES *elements, GstElement *pipeline) {
    
    if (elements == NULL || pipeline == NULL)
        return false;

   
gst_bin_add_many (GST_BIN(pipeline),  elements->source,  elements->rtph264depay, elements->parser,  elements->decoder,  elements->videoconvert,  elements->capabilities,elements->appsink, NULL);
    if (!gst_element_link_many (elements->rtph264depay, elements->parser,  elements->decoder,  elements->videoconvert, elements->capabilities , elements->appsink, NULL)) {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (pipeline);
        return -1;
    }
    /* Set the callback function for the "qtdemuxer" to receive stream data */
    g_signal_connect(elements->source, "pad-added", G_CALLBACK(on_pad_added), elements->rtph264depay);
    
    return true;
}

bool play_gst_pipeline(struct _GSTHANDLE *handle) {
    pthread_t id;
    gboolean ret;

    if (handle == NULL)
        return false;

    /* Get pipeline message bus and monitoring messages */
    handle->bus = gst_pipeline_get_bus(GST_PIPELINE(handle->pipeline));
    gst_bus_add_watch(handle->bus, bus_call, handle->loop);
    gst_object_unref(handle->bus);

    /* Start the pipeline */
    ret = gst_element_set_state(handle->pipeline, GST_STATE_PLAYING);
   
    return true;
}

/**
 * Release gstreamer pipeline
 */
void release_gst(struct _GSTHANDLE *handle) {
    /* clean up */
    gst_element_set_state(handle->pipeline, GST_STATE_NULL);

    /* Release gst pipeline*/
    gst_object_unref(GST_OBJECT(handle->pipeline));

    /* Quit loop */
    g_main_loop_unref(handle->loop);
}



int main(int argc, char *argv[]) {
   
   gboolean ret; 
    GstCaps *caps;
    ProgramOptions options;
    /* Ctrl+c handler */
    signal(SIGINT, sig_handle);
    /*gstobj Apply for heap space*/
    gstobj = (struct GSTOBJ *)malloc(sizeof(struct GSTOBJ));
    memset(gstobj, 0, sizeof(struct GSTOBJ));
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("无法初始化SDL: %s", SDL_GetError());
        return -1;
    } 
   
    /* Initialization of gstreamer */
//       auto rknn_pool1 = std::make_unique<RknnPool>(
//       options.model_path, options.thread_count, options.label_path);
  
//   auto camera1 = std::make_unique<Camera>(
//       options.rtsp_url2, cv::Size(options.width, options.height),
//       options.fps);
  

//   ImageProcess image_process(options.width, options.height, 640);
 

//   cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
  
//     std::thread thread1(thread_func1, options, std::move(rknn_pool1), std::move(camera1), image_process);
//     thread1.detach();

    ret = initialize_gst(argc, argv, &gstobj->handle);
    if (! ret) {
        printf("\n Initialization failed \n");
       // goto err_release;
    }
    /* Create elementss */
    ret = make_gst_elements(&gstobj->elements);
    if (! ret) {
        printf("\n Make elements failed\n\n");
       // goto err_release;
    }
     // 创建窗口和渲染器
    gstobj->elements.window = SDL_CreateWindow("Video2", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
    gstobj->elements.renderer = SDL_CreateRenderer(gstobj->elements.window, -1, SDL_RENDERER_SOFTWARE);
    gstobj->elements.texture = SDL_CreateTexture(gstobj->elements.renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STATIC, 1920, 1080);
     gstobj->elements.window1 = SDL_CreateWindow("Video1", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1920, 1080, 0);
   gstobj->elements.renderer1 = SDL_CreateRenderer(gstobj->elements.window1, -1, SDL_RENDERER_SOFTWARE);
    gstobj->elements.texture1 = SDL_CreateTexture(gstobj->elements.renderer1, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STATIC, 1920, 1080);
    // gstobj->elements.rknn_pool2 = std::make_unique<RknnPool>(
    //   gstobj->elements.model_path2, gstobj->elements.thread_count, gstobj->elements.label_path2);
    //    gstobj->elements.image_process2=ImageProcess ( gstobj->elements.widths2,  gstobj->elements.heights2, 640);
    gstobj->elements.rknn_pool2 = std::make_unique<RknnPool>(
      options.model_path, options.thread_count, options.label_path);
    gstobj->elements.image_process2=ImageProcess (options.width2, options.height2, 640);
    //  g_object_set(G_OBJECT(elements->appsink), "emit-signals", TRUE, NULL);
    // g_signal_connect(G_OBJECT(elements->appsink), "new-sample", G_CALLBACK(new_sample_callback), elements);
    ret=transform(&gstobj->elements);
    if (! ret) {
        printf("\n Make transform failed\n\n");
       // goto err_release;
    }
    /* Check creation of elements */
    ret = check_gst_elements(&gstobj->elements);
    if (! ret) {
        printf("\n Elements error in check \n");
       // goto err_release;
    }
    /* Link elements into pipeline */
    ret = link_gst_elements( &gstobj->elements,gstobj->handle.pipeline);
    if (! ret) {
        printf("\n Link elements failed \n");
      //  goto err_release;
    }
    /* Play the pipeline */
    ret = play_gst_pipeline(&gstobj->handle);
    if (! ret) {
        printf(" \n Play the pipeline failed \n");
       // goto err_release;
    }
    /* Create loop, keep listen for pipeline event */
    g_main_loop_run(gstobj->handle.loop);
  std::thread displayThread(displayThreadFunc);
displayThread.join();
    printf(" \n Play the pipeline..  \n");
  // 等待线程结束
 // thread1.join();
  //thread2.join();

 
//err_release:
    /* Release gst pipeline*/
    release_gst(&gstobj->handle);
    fclose(gstobj->elements.outFile);
    free(gstobj);
    //  rknn_pool1.reset();
   // rknn_pool2.reset();
   cv::destroyAllWindows();
    // if (! ret)
    //     return -1;
    // printf("Exit\n");
    return 0;
}
