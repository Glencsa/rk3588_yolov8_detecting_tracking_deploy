
#include "yolov8.h"
#include "postprocess.h"
#include "im2d.h"
#include "rga.h"
#include "RgaUtils.h"
#include <cstring>
#include <sys/time.h>
//#include "mpp_decoder.h"
//#include "mpp_encoder.h"
//#include "drawing.h"
#include "mk_mediakit.h"
const int RK3588 = 3;
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
// 设置模型需要绑定的核心
// Set the core of the model that needs to be bound
int get_core_num() {
  static int core_num = 0;
  static std::mutex mtx;
  std::lock_guard<std::mutex> lock(mtx);
  int temp = core_num % RK3588;
  core_num++;
  return temp;
}




double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

int read_data_from_file(const char *path, char **out_data) {
  FILE *fp = fopen(path, "rb");
  if (fp == NULL) {
    printf("fopen %s failed!", path);
    return -1;
  }
  fseek(fp, 0, SEEK_END);
  int file_size = ftell(fp);
  char *data = (char *)malloc(file_size + 1);
  data[file_size] = 0;
  fseek(fp, 0, SEEK_SET);
  if (file_size != fread(data, 1, file_size, fp)) {
    printf("fread %s failed!", path);
    free(data);
    fclose(fp);
    return -1;
  }
  if (fp) {
    fclose(fp);
  }
  *out_data = data;
  return file_size;
}
static void dump_tensor_attr(rknn_tensor_attr *attr) {
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

Yolov8::Yolov8(std::string &&model_path) : model_path_(model_path) {}

int Yolov8::Init(rknn_context *ctx_in, bool copy_weight) {
  int model_len = 0;
  char *model;
  int ret = 0;
  model_len = read_data_from_file(model_path_.c_str(), &model);
  if (model == nullptr) {
    printf("Load model failed");
    return -1;
  }
  if (copy_weight) {
    printf("rknn_dup_context() is called");
    // 复用模型参数
    ret = rknn_dup_context(ctx_in, &ctx_);
    if (ret != RKNN_SUCC) {
      printf("rknn_dup_context failed! error code = %d", ret);
      return -1;
    }
  } else {
    printf("rknn_init() is called");
    ret = rknn_init(&ctx_, model, model_len, 0, NULL);
    free(model);
    if (ret != RKNN_SUCC) {
      printf("rknn_init failed! error code = %d", ret);
      return -1;
    }
  }
  rknn_core_mask core_mask;
  switch (get_core_num()) {
    case 0:
      core_mask = RKNN_NPU_CORE_0;
      break;
    case 1:
      core_mask = RKNN_NPU_CORE_1;
      break;
    case 2:
      core_mask = RKNN_NPU_CORE_2;
      break;
  }
  ret = rknn_set_core_mask(ctx_, core_mask);
  if (ret < 0) {
    printf("rknn_set_core_mask failed! error code = %d", ret);
    return -1;
  }

  rknn_sdk_version version;
  ret = rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &version,
                   sizeof(rknn_sdk_version));
  if (ret < 0) {
    return -1;
  }
  printf("sdk version: %s driver version: %s", version.api_version,
                     version.drv_version);

  // Get Model Input Output Number
  rknn_input_output_num io_num;
  ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if (ret != RKNN_SUCC) {
    printf("rknn_query failed! error code = %d", ret);
    return -1;
  }
  printf("model input num: %d, and output num: %d", io_num.n_input,
                     io_num.n_output);
  // Get Model Input Info
  printf("input tensors:");
  rknn_tensor_attr
      input_attrs[io_num.n_input];  //这里使用的是变长数组，不建议这么使用
  memset(input_attrs, 0, sizeof(input_attrs));
  for (int i = 0; i < io_num.n_input; i++) {
    input_attrs[i].index = i;
    ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]),
                     sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      printf("input rknn_query failed! error code = %d", ret);
      return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
  }

  // Get Model Output Info
  printf("output tensors:");
  rknn_tensor_attr output_attrs[io_num.n_output];
  memset(output_attrs, 0, sizeof(output_attrs));
  for (int i = 0; i < io_num.n_output; i++) {
    output_attrs[i].index = i;
    ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]),
                     sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      printf("output rknn_query fail! error code = %d", ret);
      return -1;
    }
    dump_tensor_attr(&(output_attrs[i]));
  }
  // Set to context
  app_ctx_.rknn_ctx = ctx_;
  if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
      output_attrs[0].type == RKNN_TENSOR_INT8) {
    app_ctx_.is_quant = true;
  } else {
    app_ctx_.is_quant = false;
  }
  app_ctx_.io_num = io_num;
  app_ctx_.input_attrs =
      (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
  memcpy(app_ctx_.input_attrs, input_attrs,
         io_num.n_input * sizeof(rknn_tensor_attr));
  app_ctx_.output_attrs =
      (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
  memcpy(app_ctx_.output_attrs, output_attrs,
         io_num.n_output * sizeof(rknn_tensor_attr));

  if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
    printf("model is NCHW input fmt");
    app_ctx_.model_channel = input_attrs[0].dims[1];
    app_ctx_.model_height = input_attrs[0].dims[2];
    app_ctx_.model_width = input_attrs[0].dims[3];
  } else {
    printf("model is NHWC input fmt");
    app_ctx_.model_height = input_attrs[0].dims[1];
    app_ctx_.model_width = input_attrs[0].dims[2];
    app_ctx_.model_channel = input_attrs[0].dims[3];
  }
  printf("model input height=%d, width=%d, channel=%d",
                     app_ctx_.model_height, app_ctx_.model_width,
                     app_ctx_.model_channel);
 


  inputs_ = std::make_unique<rknn_input[]>(app_ctx_.io_num.n_input);
  outputs_ = std::make_unique<rknn_output[]>(app_ctx_.io_num.n_output);
  inputs_[0].index = 0;
  inputs_[0].type = RKNN_TENSOR_UINT8;
  inputs_[0].fmt = RKNN_TENSOR_NHWC;
  inputs_[0].size =
      app_ctx_.model_width * app_ctx_.model_height * app_ctx_.model_channel;
  inputs_[0].buf = nullptr;
  return 0;
}




Yolov8::~Yolov8() { DeInit(); }

int Yolov8::DeInit() {
  if (app_ctx_.rknn_ctx != 0) {
    printf("rknn_destroy");
    rknn_destroy(app_ctx_.rknn_ctx);
    app_ctx_.rknn_ctx = 0;
  }
  if (app_ctx_.input_attrs != nullptr) {
    printf("free input_attrs");
    free(app_ctx_.input_attrs);
  }
  if (app_ctx_.output_attrs != nullptr) {
    printf("free output_attrs");
    free(app_ctx_.output_attrs);
  }
  return 0;
}

rknn_context *Yolov8::get_rknn_context() { return &(this->ctx_); }

int Yolov8::Inference(void *image_buf, object_detect_result_list *od_results,
                      letterbox_t letter_box) {
    // struct timeval start_time, stop_time;
    //  gettimeofday(&start_time, NULL);   
   //auto starttime = std::chrono::high_resolution_clock::now();                 
  inputs_[0].buf = image_buf;
  int ret = rknn_inputs_set(app_ctx_.rknn_ctx, app_ctx_.io_num.n_input,
                            inputs_.get());
  if (ret < 0) {
    printf("rknn_input_set failed! error code = %d", ret);
    return -1;
  }
 
  ret = rknn_run(app_ctx_.rknn_ctx, nullptr); 
  
  if (ret != RKNN_SUCC) {
    printf("rknn_run failed, error code = %d", ret);
    return -1;
  }
  for (int i = 0; i < app_ctx_.io_num.n_output; ++i) {
    outputs_[i].index = i;
    outputs_[i].want_float = (!app_ctx_.is_quant);
  }
  outputs_lock_.lock();
  ret = rknn_outputs_get(app_ctx_.rknn_ctx, app_ctx_.io_num.n_output,
                         outputs_.get(), nullptr);
  if (ret != RKNN_SUCC) {
    printf("rknn_outputs_get failed, error code = %d", ret);
    return -1;
  }
  // gettimeofday(&stop_time, NULL);
  // printf("一次推理模型所花的时间 %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    //  auto endtime = std::chrono::high_resolution_clock::now(); 
    //    std::chrono::duration<double,std::milli> duration = endtime - starttime;
    //   printf("一次推理模型所花的时间: \t%f ms\n", duration.count());
  const float nms_threshold = NMS_THRESH;       // 默认的NMS阈值
  const float box_conf_threshold = BOX_THRESH;  // 默认的置信度阈值
  // Post Process
  // 分割的模型输出有13层
//   if (app_ctx_.io_num.n_output == 13) {
//     post_process_seg(&app_ctx_, outputs_.get(), &letter_box, box_conf_threshold,
//                      nms_threshold, od_results);
//   } else {
    post_process(&app_ctx_, outputs_.get(), &letter_box, box_conf_threshold,
                 nms_threshold, od_results);
  //}


  // Remeber to release rknn outputs_
  rknn_outputs_release(app_ctx_.rknn_ctx, app_ctx_.io_num.n_output,
                       outputs_.get());
  outputs_lock_.unlock();

 // printf("Inference time is");
  return 0;
}

int Yolov8::get_model_width() { return app_ctx_.model_width; }

int Yolov8::get_model_height() { return app_ctx_.model_height; }
