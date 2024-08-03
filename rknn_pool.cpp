
#include "rknn_pool.h"
#include <chrono>
#include "postprocess.h"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include<iostream>
using namespace std;
RknnPool::RknnPool(const std::string model_path, const int thread_num, const std::string lable_path) {
  this->thread_num_ = thread_num;
  this->model_path_ = model_path;
  this->label_path_ = lable_path;
  this->Init();
}

RknnPool::~RknnPool() { this->DeInit(); }

void RknnPool::Init() {
  init_post_process(this->label_path_);
  try {
    // 配置线程池
    this->pool_ = std::make_unique<ThreadPool>(this->thread_num_);
    // 这里每一个线程需要加载一个模型
    for (int i = 0; i < this->thread_num_; ++i) {
      models_.push_back(std::make_shared<Yolov8>(
          std::forward<std::string>(this->model_path_)));
    }
  } catch (const std::bad_alloc &e) {
    printf("Out of memory: {}", e.what());
    exit(EXIT_FAILURE);
  }
  for (int i = 0; i < this->thread_num_; ++i) {
    auto ret = models_[i]->Init(models_[0]->get_rknn_context(), i != 0);
    if (ret != 0) {
      printf("Init rknn model failed!");
      exit(EXIT_FAILURE);
    }
  }
}

void RknnPool::DeInit() { deinit_post_process(); }

void RknnPool::AddInferenceTask(std::shared_ptr<cv::Mat> src,
                                ImageProcess &image_process) {
                                  // auto starttime = std::chrono::high_resolution_clock::now();
    // 清空队列中的旧帧，只保留最新的帧
    // {
    //     std::lock_guard<std::mutex> lock_guard(this->image_results_mutex_);
    //     while (!this->image_results_.empty()) {
    //         this->image_results_.pop();
    //     }
    // }
  pool_->enqueue(
      [&](std::shared_ptr<cv::Mat> original_img) {
        auto convert_img = image_process.Convert(*original_img);
        auto mode_id = get_model_id();
        cv::Mat rgb_img = cv::Mat::zeros(
            this->models_[mode_id]->get_model_width(),
            this->models_[mode_id]->get_model_height(), convert_img->type());
        cv::cvtColor(*convert_img, rgb_img, cv::COLOR_RGB2BGR);
        object_detect_result_list od_results;
        this->models_[mode_id]->Inference(rgb_img.ptr(), &od_results,
                                          image_process.get_letter_box());
        //记录检测框数据
        int count = od_results.count;
        vector<int>des_u;
        vector<int> des_v;
        double hight = 100.0;
        cv::Mat K = (cv::Mat_<double>(3,3)<<1000,0,960,0,1000,540,0,0,1);
        cv::Mat K_inv;
        cv::invert(K,K_inv);


        for(int i =0;i<count;++i)
        {
          int u = (od_results.results[i].box.left+od_results.results[i].box.right)/2;
          int v = ((od_results.results[i].box.top+od_results.results[i].box.bottom)/2);
          des_u.push_back(u);
          des_v.push_back(v);
          // cout<<"类别："<<od_results.results[i].cls_id<<endl
          //     <<"置信度："<<od_results.results[i].prop<<endl
          //     <<"左："<<od_results.results[i].box.left<<endl
          //     <<"右："<<od_results.results[i].box.right<<endl
          //     <<"上："<<od_results.results[i].box.top<<endl
          //     <<"下："<<od_results.results[i].box.bottom<<endl;
          //   cv::Mat tx = (cv::Mat_<double>(3,1)<<u,v,1);
          //   cv::Mat o_res = K_inv*tx;
          //   double w = o_res.at<double>(2,0);
          //   cv::Mat no_res = o_res/w*hight;
          //   cout<<"mubiao_position:: (x,y,z):"<<no_res<<endl;
        }

        //bytetrack检测
        this->mytrack.Add_frame(od_results);
        std::cout<<"strack number "<<this->mytrack.m_stracks.size()<<" of "<<od_results.count<<endl;
        image_process.ImagePostProcess(*original_img, this->mytrack.m_stracks,this->mytrack);


        // image_process.ImagePostProcess(*original_img, od_results);
        std::lock_guard<std::mutex> lock_guard(this->image_results_mutex_);
        this->image_results_.push(std::move(original_img));
        std::lock_guard<std::mutex> lock_guard1(this->obj_results_mutex_);
        std::shared_ptr<object_detect_result_list> od = std::make_shared<object_detect_result_list>(od_results);
        this->obj_results.push(std::move(od));
      },
      std::move(src));
    //    auto endtime = std::chrono::high_resolution_clock::now();
    // // // 计算持续时间
    //   std::chrono::duration<double,std::milli> duration = endtime - starttime;
    //   printf("inference use time: \t%f ms\n", duration.count());
}

int RknnPool::get_model_id() {
  std::lock_guard<std::mutex> lock(id_mutex_);
  int mode_id = id % thread_num_;
  id++;

  return mode_id;
}

std::shared_ptr<cv::Mat> RknnPool::GetImageResultFromQueue() {
   
  std::lock_guard<std::mutex> lock_guard(this->image_results_mutex_);
  if (this->image_results_.empty()) {
    return nullptr;
  }
   else {
    auto res = this->image_results_.front();
    this->image_results_.pop();
    return std::move(res);
  }
   
}
std::shared_ptr<object_detect_result_list> RknnPool::GetObjResultFromQueue() {
   
  std::lock_guard<std::mutex> lock_guard1(this->obj_results_mutex_);
  if (this->obj_results.empty()) {
    return nullptr;
  }
   else {
    auto res = this->obj_results.front();
    this->obj_results.pop();
    return std::move(res);
  }
   
}

int RknnPool::GetTasksSize() { return pool_->TasksSize(); }