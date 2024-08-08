
#include "image_process.h"
#include "im2d.h"
#include "rga.h"
#include "RgaUtils.h"
#include "drawing.h"
#define N_CLASS_COLORS (20)
unsigned char class_colors[][3] = {
    {255, 56, 56},    // 'FF3838'
    {255, 157, 151},  // 'FF9D97'
    {255, 112, 31},   // 'FF701F'
    {255, 178, 29},   // 'FFB21D'
    {207, 210, 49},   // 'CFD231'
    {72, 249, 10},    // '48F90A'
    {146, 204, 23},   // '92CC17'
    {61, 219, 134},   // '3DDB86'
    {26, 147, 52},    // '1A9334'
    {0, 212, 187},    // '00D4BB'
    {44, 153, 168},   // '2C99A8'
    {0, 194, 255},    // '00C2FF'
    {52, 69, 147},    // '344593'
    {100, 115, 255},  // '6473FF'
    {0, 24, 236},     // '0018EC'
    {132, 56, 255},   // '8438FF'
    {82, 0, 133},     // '520085'
    {203, 56, 255},   // 'CB38FF'
    {255, 149, 200},  // 'FF95C8'
    {255, 55, 199}    // 'FF37C7'
};
inline int clamp(float val, int min, int max) {
  return val > min ? (val < max ? val : max) : min;
}
static int ret=0;
int resize_rga(rga_buffer_t &src, rga_buffer_t &dst, const cv::Mat &image, cv::Mat &resized_image, const cv::Size &target_size)
{
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    size_t img_width = image.cols;
    size_t img_height = image.rows;
    if (image.type() != CV_8UC3)
    {
        printf("source image type is %d!\n", image.type());
        return -1;
    }
    size_t target_width = target_size.width;
    size_t target_height = target_size.height;
    src = wrapbuffer_virtualaddr((void *)image.data, img_width, img_height, RK_FORMAT_RGB_888);
    dst = wrapbuffer_virtualaddr((void *)resized_image.data, target_width, target_height, RK_FORMAT_RGB_888);
    int ret = imcheck(src, dst, src_rect, dst_rect);
    if (IM_STATUS_NOERROR != ret)
    {
        fprintf(stderr, "rga check error! %s", imStrError((IM_STATUS)ret));
        return -1;
    }
    IM_STATUS STATUS = imresize(src, dst);
    return 0;
}


ImageProcess::ImageProcess(int width, int height, int target_size) {
  scale_ = static_cast<double>(target_size) / std::max(height, width);
  padding_x_ = target_size - static_cast<int>(width * scale_);
  padding_y_ = target_size - static_cast<int>(height * scale_);
  new_size_ = cv::Size(static_cast<int>(width * scale_),
                       static_cast<int>(height * scale_));
  target_size_ = target_size;
  letterbox_.scale = scale_;
  letterbox_.x_pad = padding_x_ / 2;
  letterbox_.y_pad = padding_y_ / 2;
}

std::unique_ptr<cv::Mat> ImageProcess::Convert(const cv::Mat &src) {
  if (&src == nullptr) {
    return nullptr;
  }
  
#if 1
  int img_width = src.cols;
  int img_height = src.rows;
  
  rga_buffer_t srcimage;
  rga_buffer_t dstimage;
  memset(&srcimage, 0, sizeof(srcimage));
  memset(&dstimage, 0, sizeof(dstimage));
  cv::Size target_sizes(640, 640);
  cv::Mat resized_imging(target_sizes.height, target_sizes.width, CV_8UC3);

  ret  = resize_rga(srcimage, dstimage, src, resized_imging, target_sizes);
  if (ret != 0)
  {
  printf("resize with rga error\n");
  return 0;
  }

std::unique_ptr<cv::Mat> square_img = std::make_unique<cv::Mat>(resized_imging);
#endif
  // opencv 1
#if 0
   cv::Mat resize_img;
   cv::resize(src, resize_img, new_size_);
   auto square_img = std::make_unique<cv::Mat>(target_size_, target_size_,
                                               src.type(), cv::Scalar(114, 114, 114));
   cv::Point position(padding_x_ / 2, padding_y_ / 2);
   resize_img.copyTo((*square_img)(
     cv::Rect(position.x, position.y, resize_img.cols, resize_img.rows)));
#endif
  return std::move(square_img);
}

const letterbox_t &ImageProcess::get_letter_box() { return letterbox_; }

void ImageProcess::ImagePostProcess(cv::Mat &image,
                                    object_detect_result_list &od_results) {
  char text[256];
  //  static int img_width = image.cols;
  //   static  int img_height = image.rows;
  for (int i = 0; i < od_results.count; ++i) {
    object_detect_result *detect_result = &(od_results.results[i]);
  //   //detect_result_t *det_result = &(detect_result_group.results[i]);
  //    sprintf(text, "%s %.1f%%", coco_cls_to_name(detect_result->cls_id),
  //            detect_result->prop * 100);
  //   int x1 = detect_result->box.left;
  //       int y1 = detect_result->box.top;

  //       int x2 = detect_result->box.right;
  //       int y2 = detect_result->box.bottom;
  //       draw_rectangle_yuv420sp((unsigned char *)image.data, img_width, img_height, x1, y1, x2 - x1 + 1, y2 - y1 + 1, 0x00FF0000, 4);
  //       putText(image, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    //printf("%s @ (%d %d %d %d) %f\n", coco_cls_to_name(detect_result->cls_id), detect_result->box.left, detect_result->box.top,
    //       detect_result->box.right, detect_result->box.bottom, detect_result->prop);
    cv::rectangle(
        image, cv::Point(detect_result->box.left, detect_result->box.top),
        cv::Point(detect_result->box.right, detect_result->box.bottom),
        cv::Scalar(0, 0, 255), 2);
    
    sprintf(text, "%s %.1f%%", coco_cls_to_name(detect_result->cls_id),
            detect_result->prop * 100);
    cv::putText(image, text,
                cv::Point(detect_result->box.left, detect_result->box.top + 20),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2,
                cv::LINE_8);
  }
}

void DrawTrackLines(cv::Mat &image, const  MyTrack &mytrack) {
 // int strack_num=mytrack.m_stracks.size();
 // for (int i = 0; i < strack_num; i++)
 // {
 //   const std::vector<cv::Point> point_vector=mytrack.m_history_trackpoints.at(mytrack.m_stracks[i].track_id);
 //   cv::polylines(image, point_vector, false, Scalar(0, 0, 255), 4, cv::LINE_8); 
 //   for (int j = 0; j < point_vector.size(); j++)
  //  {
  //    std::cout<<'('<<point_vector[j].x<<" "<<point_vector[j].y<<')';
  //  }
  //  std::cout<<endl;
 // }
  
}


void ImageProcess::ImagePostProcess(cv::Mat &image,
                                    const std::vector<STrack> &track_results,const MyTrack &mytrack) {
  char text[256];
  for (int i = 0; i < track_results.size(); ++i) {
    const STrack &one_track = track_results[i];
 
    cv::rectangle(
        image, cv::Point(one_track.tlbr[0], one_track.tlbr[1]),
        cv::Point(one_track.tlbr[2], one_track.tlbr[3]),
        cv::Scalar(0, 0, 255), 4);
    
    sprintf(text, "id: %d %.3f%%", one_track.track_id,
            one_track.score);
    
    cv::putText(image, text,
                cv::Point(one_track.tlbr[0], one_track.tlbr[1] - 20),
                cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 0, 0), 3,
                cv::LINE_8);
std::cout<<one_track.tlbr[0]<<" "<<one_track.tlbr[1]<<" "<<one_track.tlbr[2]<<" "<<one_track.tlbr[3]<<endl;
    //DrawTrackLines(image,mytrack);
  }
}


