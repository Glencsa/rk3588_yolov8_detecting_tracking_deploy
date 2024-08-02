//
// Created by <Zifeng Deng>binape@126.com on 2024/7/17
//

#include "ffstream.h"

#include "ffmpeg.h"

#include <QCoreApplication>
#include <iostream>

class MyStreamFilter : public ffmpeg::AVFrameFilter {

public:
  void doFilter(AVFrame *frame) override
  {
    // std::cout << "FRAME:" << frame << std::endl;
  }
};

static int ff_remuxing_stream(int argc, char *argv[]);

int main(int argc, char *argv[])
{
  ff_remuxing_stream(argc, argv);
}

int ff_remuxing_stream(int argc, char *argv[])
{
  QCoreApplication app(argc ,argv);

  avcodec_register_all();
  av_register_all();
  avformat_network_init();

  MyStreamFilter filter;

  ffmpeg::StreamMedia media("rtsp://192.168.2.120/554");
  media.addOutput("rtsp://192.168.2.1:8554/live", "rtsp");
  media.addFilter(&filter);
  media.start();

  int rc = app.exec();
  avformat_network_deinit();
  return rc;
}
