//
// Created by <Zifeng Deng>binape@126.com on 2024/7/17
//

#ifndef FFMPEG_STREAM_H
#define FFMPEG_STREAM_H

#define FFDECODER_EXPORT

#include <string>

struct AVFrame;

namespace ffmpeg {
  class StreamDecoder;
  class StreamEncoder;

  class AVFrameFilter {
  public:
    virtual void doFilter(AVFrame *frame) { };
    virtual void onStreamClosed() {};
    virtual void onStreamFinished() { };
    virtual void onStreamShouldStop() { };
  };

  class FFDECODER_EXPORT StreamMedia
  {
  public:
    enum Type {
      TYPE_OUTPUT,
      TYPE_INPUT,
    };

    StreamMedia(const std::string &source, const std::string &format = {}, Type type_ = TYPE_INPUT);
    virtual ~StreamMedia();

    void addOutput(const std::string &output, const std::string &format = {});
    void start();
    void stop();
    void addFilter(AVFrameFilter *filter);

  private:
    class Private;
    friend class ffmpeg::StreamMedia::Private;
    friend class ffmpeg::StreamDecoder;
    friend class ffmpeg::StreamEncoder;
    ffmpeg::StreamMedia::Private *d;
  };
}; // namespace ffmpeg


#endif // FFMPEG_STREAM_H
