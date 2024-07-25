//
// Created by <Zifeng Deng>binape@126.com on 2024/7/17
//

#include "ffstream.h"

#include <map>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <queue>
#include <cstdio>
#include <chrono>

#include "ffmpeg.h"

#define STREAM_DURATION   10.0
#define STREAM_FRAME_RATE 25 /* 25 images/s */
#define STREAM_PIX_FMT    AV_PIX_FMT_YUV420P /* default pix_fmt */

#define FF_ERRBUF_SIZE 512

template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto ff_since(std::chrono::time_point<clock_t, duration_t> const& start)
{
  return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

static char* ff_av_err2str(int ret)
{
  static char errbuf[FF_ERRBUF_SIZE + 1];
  memset(errbuf, 0, FF_ERRBUF_SIZE + 1);
  av_make_error_string(errbuf, FF_ERRBUF_SIZE, ret);
  return errbuf;
}

static AVFrame *ff_alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
  AVFrame *picture;
  int ret;

  picture = av_frame_alloc();
  if (!picture)
    return NULL;

  picture->format = pix_fmt;
  picture->width  = width;
  picture->height = height;

  /* allocate the buffers for the frame data */
  ret = av_frame_get_buffer(picture, 32);
  if (ret < 0) {
    fprintf(stderr, "Could not allocate frame data.\n");
    exit(1);
  }

  return picture;
}

using namespace ffmpeg;

namespace ffmpeg
{

class StreamDecoder
{

public:
  explicit StreamDecoder(StreamMedia::Private *m)
      : d(m), codec(nullptr), avctx(nullptr), frame(nullptr) {
    open();
  }

  ~StreamDecoder() { close(); }

  bool isReady() const { return decodeReady; }

private:
  void open();
  void decode(AVPacket *pkt);
  void close();

private:
  friend class StreamMedia::Private;

  StreamMedia::Private *d;
  AVCodec *codec;
  AVCodecContext *avctx;
  AVFrame *frame;
  bool decodeReady{false};
};

class StreamEncoder {
public:
  StreamEncoder(StreamMedia::Private *m);

  void open(AVFrame *frame);
  void close();

  void encode(AVFormatContext *oc, AVFrame *frame);

private:
  friend class StreamMedia::Private;
  StreamMedia::Private *d;
  AVCodec *codec;
  AVCodecContext *avctx;
  AVFrame *frame;
  AVFrame *tmp_frame;
};

}

class StreamMedia::Private
{
  Private(const std::string &source_, const std::string &format_, StreamMedia::Type type_, StreamMedia *m)
      : q(m), parentMedia(nullptr), type(type_), fctx(nullptr), source(source_), format(format_), stream(nullptr),
        stream_idx(-1), remux_thread(nullptr), decode_thread(nullptr), stream_thread(nullptr) , remux_frame_thread(nullptr)
        {
    stop_remux.store(false);
  }

private:
  bool openMedia(const std::string &format = {});
  void closeMedia();
  void addOutputMedia(const std::string &output, const std::string &format);
  void startThreads();
  void stopThreads();
  void stopThread(std::thread **th, const char *name);

  void doFilter(AVFrame *frame);
  void onStreamClosed();
  void onStreamFinished();
  void onStreamShouldClose();

private:
  static void remux_thread_func(void *arg);
  static void decode_thread_func(void *arg);
  static void stream_thread_func(void *arg);
  static void remux_frame_thread_func(void *arg);

private:
  friend class StreamMedia;
  friend class StreamDecoder;
  friend class StreamEncoder;

  StreamMedia *q;
  StreamMedia *parentMedia;
  StreamMedia::Type type;
  AVFormatContext *fctx;
  std::string source;
  std::string format;
  AVStream *stream;
  int stream_idx;
  std::vector<AVFrameFilter *> filters;
  std::map<int, AVStream *> streams;
  int stream_type_idx[AVMEDIA_TYPE_NB]{};
  std::atomic<bool> stop_remux{};
  std::thread *remux_thread;
  std::thread *decode_thread;
  std::thread *stream_thread;
  std::thread *remux_frame_thread;
  bool stream_ready{false};
  std::queue<AVFrame *> remux_frame_q;
  std::mutex remux_frame_mutex;
  std::condition_variable remux_frame_cv;
  std::queue<AVPacket *> pkt_q;
  std::mutex pkt_mutex;
  std::condition_variable pkt_cv;
  std::queue<AVPacket *> remux_q;
  std::mutex remux_mutex;
  std::condition_variable remux_cv;
  std::map<std::string, StreamMedia *> outputs;
  bool mediaStopping{false};
  std::mutex state_mutex;
};

void StreamMedia::Private::doFilter(AVFrame *frame)
{
  for (auto &filter : filters) {
    filter->doFilter(frame);
  }
}

void StreamMedia::Private::onStreamClosed() {
  for (auto &filter : filters) {
    filter->onStreamClosed();
  }
}

void StreamMedia::Private::onStreamFinished() {
  for (auto &filter : filters) {
    filter->onStreamFinished();
  }
}

void StreamMedia::Private::onStreamShouldClose() {
  for (auto &filter : filters) {
    filter->onStreamShouldStop();
  }
}

void StreamMedia::Private::stopThread(std::thread **th, const char *name)
{
  std::thread *thread = *th;
  if (thread == remux_frame_thread) {
    std::lock_guard<std::mutex> lock(remux_frame_mutex);
    remux_frame_q.push(nullptr);
    remux_frame_cv.notify_all();
  }
  if (thread != nullptr) {
    if (thread->joinable())
      thread->join();
    delete thread;
    std::cout << "[INPUT] terminate" << name << "thread gracefully:" << (uint64_t)thread  << std::endl;
    *th = nullptr;
  }
}

void StreamMedia::Private::stopThreads() {
  if (type == TYPE_INPUT) {
    stop_remux.store(true);
    stopThread(&remux_thread, "remuxing");
    stopThread(&stream_thread, "streaming");
    stopThread(&decode_thread, "decoding");
    std::cout << "[INPUT] all sub-threads terminated:" << this  << std::endl;
  }
}

void StreamDecoder::open() {
  int rc;
  AVDictionary *options = nullptr;

  avctx = avcodec_alloc_context3(nullptr);

  rc = avcodec_parameters_to_context(avctx, d->stream->codecpar);
  if (rc < 0) {
    avcodec_free_context(&avctx);
    avctx = nullptr;
    std::cout << "[DECODER] ERR avcodec_parameters_to_context():" << rc << std::endl;
    return;
  }

  codec = const_cast<AVCodec *>(avcodec_find_decoder(avctx->codec_id));

  avctx->lowres = 0;
  avctx->flags2 |= AV_CODEC_FLAG2_FAST;
  avctx->flags |= AV_CODEC_FLAG_LOW_DELAY;

  av_dict_set(&options, "threads", "auto", 0);
  av_dict_set_int(&options, "lowres", 0, 0);
  av_dict_set(&options, "refcounted_frames", "1", 0);
  av_dict_set(&options, "frame_duration", "5", 0);
  av_dict_set(&options, "application", "lowdelay", 0);
  av_dict_set(&options, "flags", "low_delay", 0);

  rc = avcodec_open2(avctx, codec, &options);
  if (rc < 0) {
    avcodec_free_context(&avctx);
    av_dict_free(&options);
    avctx = nullptr;
    std::cout << "[DECODER] ERR avcodec_open2():" << rc  << std::endl;
    return;
  }
  av_dict_free(&options);
  frame = av_frame_alloc();
  decodeReady = true;
}

void StreamDecoder::close() {
  if (avctx != nullptr) {
    avcodec_close(avctx);
    avcodec_free_context(&avctx);
    avctx = nullptr;
  }
  if (frame != nullptr) {
    av_frame_free(&frame);
    frame = nullptr;
  }
}

void StreamDecoder::decode(AVPacket *pkt) {
  int rc;

  if (avctx == nullptr || frame == nullptr)
    return;

  do {
    rc = avcodec_receive_frame(avctx, frame);
    if (rc >= 0) {
      frame->pts = frame->pkt_dts;
    }
    if (rc == AVERROR_EOF) {
      avcodec_flush_buffers(avctx);
      break;
    }
    if (frame->width && frame->height) {
      {
        std::lock_guard<std::mutex> lock(d->remux_frame_mutex);
        d->remux_frame_q.push(av_frame_clone(frame));
        d->remux_frame_cv.notify_one();
      }
      d->doFilter(frame);
    }
  } while (rc != AVERROR(EAGAIN));
  avcodec_send_packet(avctx, pkt);
}


StreamEncoder::StreamEncoder(StreamMedia::Private *m)
    : d(m)
{

}

void ffmpeg::StreamEncoder::open(AVFrame *frame)
{
  AVCodecContext *c;
  int ret;
  AVDictionary *opt = nullptr;

  AVCodecID codec_id = d->stream->codecpar->codec_id;

  codec = avcodec_find_encoder(codec_id);
  codec_id = codec->id;

  avctx = avcodec_alloc_context3(codec);

  c = avctx;

  c->codec_id = codec_id;

  c->bit_rate = 400000;
  /* Resolution must be a multiple of two. */
  c->width    = frame->width;
  c->height   = frame->height;
  /* timebase: This is the fundamental unit of time (in seconds) in terms
   * of which frame timestamps are represented. For fixed-fps content,
   * timebase should be 1/framerate and timestamp increments should be
   * identical to 1. */
  c->time_base       = (AVRational){ 1, STREAM_FRAME_RATE };

  c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
  c->pix_fmt       = (AVPixelFormat)frame->format;
  if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
    /* just for testing, we also add B-frames */
    c->max_b_frames = 2;
  }
  if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
    /* Needed to avoid using macroblocks in which some coeffs overflow.
     * This does not happen with normal video, it just happens here as
     * the motion of the chroma plane does not match the luma plane. */
    c->mb_decision = 2;
  }

  av_opt_set(c->priv_data, "preset", "medium", 0);
  av_opt_set(c->priv_data, "look_ahead", "0", 0);
  av_opt_set(c->priv_data, "tune", "zerolatency", 0);
  /* open the codec */
  ret = avcodec_open2(c, codec, &opt);
  av_opt_free(opt);
  if (ret < 0) {
    fprintf(stderr, "Could not open video codec: %s\n", ff_av_err2str(ret));
    exit(1);
  }

  /* allocate and init a re-usable frame */
  frame = ff_alloc_picture(c->pix_fmt, c->width, c->height);
  if (!frame) {
    fprintf(stderr, "Could not allocate video frame\n");
    exit(1);
  }

  /* If the output format is not YUV420P, then a temporary YUV420P
   * picture is needed too. It is then converted to the required
   * output format. */
  tmp_frame = NULL;
  if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
    tmp_frame = ff_alloc_picture(AV_PIX_FMT_YUV420P, c->width, c->height);
    if (!tmp_frame) {
      fprintf(stderr, "Could not allocate temporary picture\n");
      exit(1);
    }
  }

  /* copy the stream parameters to the muxer */
  ret = avcodec_parameters_from_context(d->stream->codecpar, c);
  if (ret < 0) {
    fprintf(stderr, "Could not copy the stream parameters\n");
    exit(1);
  }
}

void StreamEncoder::encode(AVFormatContext *oc, AVFrame *frame)
{
  AVPacket pkt = { 0 };

  int rc = avcodec_send_frame(avctx, frame);
  if (rc != 0) {
    std::cout << "avcodec_send_frame:" << ff_av_err2str(rc) << std::endl;
    return;
  }
  av_init_packet(&pkt);
  while (avcodec_receive_packet(avctx, &pkt) == 0) {
    av_write_frame(oc, &pkt);
  }
}

void ffmpeg::StreamEncoder::close()
{
  if (avctx != nullptr) {
    avcodec_close(avctx);
    avcodec_free_context(&avctx);
    avctx = nullptr;
  }
  if (frame != nullptr) {
    av_frame_free(&frame);
    frame = nullptr;
  }
}

void StreamMedia::Private::startThreads()
{
  //  remux_thread = new std::thread(&remux_thread_func, (void *) this);
  // std::cout << "[INPUT] Remuxing thread created:" << remux_thread  << std::endl;

  remux_frame_thread = new std::thread(&remux_frame_thread_func, (void *) this);
  std::cout << "[INPUT] Frame Remuxing thread created:" << remux_frame_thread << std::endl;

  decode_thread = new std::thread(&decode_thread_func, (void *) this);
  std::cout << "[INPUT] Decoding thread created:" << decode_thread  << std::endl;

  stream_thread = new std::thread(&stream_thread_func, (void *) this);
  std::cout << "[INPUT] Streaming thread created:" << stream_thread  << std::endl;

}

void StreamMedia::Private::decode_thread_func(void *arg) {
  int rc;
  StreamMedia::Private *d = (StreamMedia::Private *) arg;

  std::cout << "[DECODER] Decoding thread started:" << d->source.c_str()  << std::endl;

  StreamDecoder decoder(d);
  if (!decoder.isReady()) {
    d->onStreamClosed();
    return;
  }
  while (!d->stop_remux.load()) {
    std::unique_lock<std::mutex> lock(d->pkt_mutex);
    d->pkt_cv.wait(lock, [d]() { return !d->pkt_q.empty(); });

    AVPacket *pkt = d->pkt_q.front();
    d->pkt_q.pop();
    if (pkt == nullptr) break;
    lock.unlock();
    decoder.decode(pkt);
    av_packet_unref(pkt);
  }
  std::thread::id tid = std::this_thread::get_id();
  std::cout << "[DECODER] Decoding thread terminated:" << &tid << ", source:" << d->source.c_str()  << std::endl;
}

void StreamMedia::Private::remux_thread_func(void *arg) {
  int rc;
  StreamMedia::Private *d = (StreamMedia::Private *) arg;

  std::cout << "[REMUXER] Recording thread started:" << d->source.c_str()  << std::endl;

  bool header_wrote = false;
  bool do_remux = true;

  while (!d->stop_remux.load()) {
    std::unique_lock<std::mutex> lock(d->remux_mutex);
    d->remux_cv.wait(lock, [d]() { return !d->remux_q.empty(); });

    if (!header_wrote) {
      for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
        StreamMedia *media = it->second;
        rc = avformat_write_header(media->d->fctx, nullptr);
        std::cout << "[REMUXER] write header:" << media->d->stream << ", stream index:" << media->d->stream_idx
                  << ", STATE=" << rc << std::endl;
      }
      header_wrote = true;
    }

    AVPacket *pkt = d->remux_q.front();
    d->remux_q.pop();
    if (pkt == nullptr)
      break;
    lock.unlock();

    if (do_remux) {
      AVStream *in_stream = d->stream;
      for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
        StreamMedia *media = it->second;
        AVStream *out_stream = media->d->stream;
        pkt->stream_index = out_stream->index;
        av_packet_rescale_ts(pkt, in_stream->time_base, out_stream->time_base);
         // pkt->duration = 100;
         pkt->pos = -1;
        // rc = av_write_frame(media->d->fctx, pkt);
         rc = av_interleaved_write_frame(media->d->fctx, pkt);
        if (rc < 0) {
          std::cout << "[REMUXER] ERR: av_interleaved_write_frame:" << rc  << std::endl;
        }
      }
    }
    av_packet_unref(pkt);
  }

  for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
    StreamMedia *media = it->second;
    av_write_trailer(media->d->fctx);
  }

  std::thread::id tid = std::this_thread::get_id();
  std::cout << "[REMUXER] Recording thread terminated:" << &tid << ", source:" << d->source.c_str()  << std::endl;
}


void StreamMedia::Private::remux_frame_thread_func(void *arg)
{
  int rc;

  bool header_wrote = false;
  StreamMedia::Private *d = (StreamMedia::Private *) arg;

  StreamEncoder *encoder = nullptr;

  std::cout << "[REMUXER:F] Frame recording thread started:" << d->source.c_str() << std::endl;
  while (!d->stop_remux.load()) {
    std::unique_lock<std::mutex> lock(d->remux_frame_mutex);
    d->remux_frame_cv.wait(lock, [d]() { return !d->remux_frame_q.empty(); });

    AVFrame *frame = d->remux_frame_q.front();
    d->remux_frame_q.pop();
    if (frame == nullptr)
      break;
    lock.unlock();


    if (!header_wrote) {
      for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
        StreamMedia *media = it->second;
        rc = avformat_write_header(media->d->fctx, nullptr);
        std::cout << "[REMUXER] write header:" << media->d->stream << ", stream index:" << media->d->stream_idx
                  << ", STATE=" << rc << std::endl;
      }
      header_wrote = true;
    }

    if (encoder == nullptr) {
      encoder = new StreamEncoder(d);
      encoder->open(frame);
    }

    AVStream *in_stream = d->stream;
    for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
      StreamMedia *media = it->second;
      // AVStream *out_stream = media->d->stream;
      encoder->encode(media->d->fctx, frame);
    }

    av_frame_free(&frame);
  }

  for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
    StreamMedia *media = it->second;
    av_write_trailer(media->d->fctx);
  }

  encoder->close();
  delete encoder;
}

void StreamMedia::Private::stream_thread_func(void *arg) {
  int rc;

  StreamMedia::Private *d = (StreamMedia::Private *) arg;

  std::cout << "[INPUT] Streaming thread started:" << d->source.c_str()  << std::endl;

  AVPacket *pkt = av_packet_alloc();
  while (!d->stop_remux.load()) {
    rc = av_read_frame(d->fctx, pkt);
    if (rc < 0) {
      if (rc != AVERROR_EOF)
        std::cout << "[INPUT] failed to read from source:" << d->source.c_str()  << std::endl;
      {
        std::lock_guard<std::mutex> lock(d->pkt_mutex);
        d->pkt_q.push(nullptr);
        d->pkt_cv.notify_all();
      }
      {
        std::lock_guard<std::mutex> lock(d->remux_mutex);
        d->remux_q.push(nullptr);
        d->remux_cv.notify_all();
      }
      break;
    }

    if (pkt->stream_index != d->stream_idx) {
      av_packet_unref(pkt);
      continue;
    }

    {
      // send packet to remux thread
      std::lock_guard<std::mutex> lock(d->remux_mutex);
      AVPacket *pkt1 = av_packet_clone((const AVPacket *) pkt);
      d->remux_q.push(pkt1);
      d->remux_cv.notify_one();
    }

    {
      // send packet to decode thread
      std::lock_guard<std::mutex> lock(d->pkt_mutex);
      AVPacket *pkt1 = av_packet_clone((const AVPacket *) pkt);
      d->pkt_q.push(pkt1);
      d->pkt_cv.notify_one();
    }
    av_packet_unref(pkt);
  }

  av_packet_free(&pkt);
  std::cout << "[INPUT] Streaming thread terminated:" << d->source.c_str()  << std::endl;
  d->onStreamClosed();
}

void StreamMedia::Private::addOutputMedia(const std::string &output, const std::string &format) {
  StreamMedia *media = new StreamMedia(output, format, TYPE_OUTPUT);
  media->d->parentMedia = q;
  outputs[output] = media;
}

bool StreamMedia::Private::openMedia(const std::string &format_) {
  int rc;
  const char *format_name = nullptr;

  if (!format_.empty()) {
    format_name = format_.c_str();
  } else if (!this->format.empty()) {
    format_name = this->format.c_str();
  }

  if (TYPE_OUTPUT == type) {
    rc = avformat_alloc_output_context2(&fctx, nullptr, format_name, source.c_str());
    if (rc < 0) {
      std::cout << "[OUTPUT] failed to allocate output context:" << rc  << std::endl;
      return false;
    }

    std::cout << "[OUTPUT] output media ready:" << source.c_str() << ", format:" << format_name << ",ctx:" << fctx  << std::endl;

    for (auto iter = parentMedia->d->streams.begin();
         iter != parentMedia->d->streams.end(); iter++) {
      AVStream *istrm = iter->second;

      AVCodec *codec = avcodec_find_encoder(istrm->codecpar->codec_id);
      AVStream *ostrm = avformat_new_stream(fctx, codec);
      avcodec_parameters_copy(ostrm->codecpar, istrm->codecpar);
      ostrm->index = iter->first;
      ostrm->codecpar->codec_tag = 0;
      std::cout << "[OUTPUT] create new stream:" << ostrm->codecpar->codec_type << ", stream=" << ostrm  << std::endl;
      if (ostrm->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        stream = ostrm;
        stream_idx = ostrm->index;
        break;
      }
    }
    std::cout << "[OUTPUT] output stream ready:" << source.c_str() << ",ctx:" << fctx  << std::endl;
    return true;
  } else {
    // OPEN Input Media
    AVDictionary *options = nullptr;
    av_dict_set(&options, "avioflags", "direct", 1);
    av_dict_set(&options, "fflags", "+genpts", 1);
    av_dict_set(&options, "fflags", "nobuffer", 1);
    av_dict_set(&options, "fflags", "discardcorrupt", 1);
    av_dict_set(&options, "reconnect", "1", 0);
    av_dict_set(&options, "reconnect_at_eof", "0", 0);
    av_dict_set(&options, "reconnect_delay_max", "2", 0);
    av_dict_set(&options, "max_delay", "80", 0);
    av_dict_set(&options, "stimeout", "5000000", 0);
    av_dict_set(&options, "rtsp_transport", "udp", 0);
    av_dict_set(&options, "propesize", "256", 0);

    AVInputFormat *iformat = nullptr;
    if (source.find("rtsp://") != std::string::npos) {
      iformat = av_find_input_format("rtsp");
      av_dict_set(&options, "rtsp_transport", "udp", 0);
    } else if (source.find("udp://") != std::string::npos) {
      iformat = av_find_input_format("mpegts");
    } else if (!format_.empty()) {
      iformat = av_find_input_format(format_name);
    }

    rc = avformat_open_input(&fctx, source.c_str(), iformat, &options);
    if (rc != 0) {
      if (!mediaStopping) {
        std::cout << "[INPUT] failed to open input media:" << source.c_str()  << std::endl;
        closeMedia();
        onStreamClosed();
      }
      return false;
    }
    rc = avformat_find_stream_info(fctx, nullptr);
    if (rc != 0) {
      if (!mediaStopping) {
        std::cout << "[INPUT] No stream found for input:" << source.c_str()  << std::endl;
        closeMedia();
        onStreamClosed();
      }
      return false;
    }

    for (uint32_t i = 0; i < fctx->nb_streams; i++) {
      AVStream *it = fctx->streams[i];
      AVCodecParameters *codecpar = it->codecpar;
      if (codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        stream_idx = i;
        stream = it;
      }
      streams[i] = it;
      stream_type_idx[codecpar->codec_type] = i;
    }

    if (stream == nullptr) {
      std::cout << "[INPUT] No video stream found for input:" << source.c_str()  << std::endl;
      closeMedia();
      onStreamClosed();
      return false;
    }
    stream_ready = true;
    std::cout << "[INPUT] input stream ready:" << source.c_str()  << std::endl;
    return true;
  }
}

void StreamMedia::Private::closeMedia() {
  if (fctx != nullptr) {
    if (TYPE_OUTPUT == type) {
      std::cout << "[OUTPUT] output media closing:" << source.c_str()  << std::endl;
      avio_close(fctx->pb);
      avformat_free_context(fctx);
      std::cout << "[OUTPUT] output media closed:" << source.c_str()  << std::endl;
    } else {
      std::cout << "[INPUT] input media closing:" << this->source.c_str()  << std::endl;
      for (auto it = outputs.begin(); it != outputs.end(); it++) {
        StreamMedia *media = it->second;
        media->d->closeMedia();
        delete media;
      }
      outputs.clear();
      if (stream_ready) {
        avformat_close_input(&fctx);
        stream_ready = false;
      }
      std::cout << "[INPUT] input media closed:" << this->source.c_str()  << std::endl;
    }
    fctx = nullptr;
  }
}

StreamMedia::StreamMedia(const std::string &source, const std::string &format, StreamMedia::Type type_)
    : d(new StreamMedia::Private(source, format, type_, this)) {

}

StreamMedia::~StreamMedia() {
  std::cout << "[" << (d->type == TYPE_INPUT ? "INPUT" : "OUTPUT") << "] media released:" << d->source.c_str()  << std::endl;
  delete d;
}

void StreamMedia::addOutput(const std::string &output, const std::string &format) {
  if (d->outputs.find(output) != d->outputs.end())
    return;
  d->addOutputMedia(output, format);
}

void StreamMedia::addFilter(ffmpeg::AVFrameFilter *filter)
{
  d->filters.push_back(filter);
}

void StreamMedia::start() {
  if (d->type == TYPE_OUTPUT) {
    std::cout << "[INPUT] Invalid stream media, INPUT type required"  << std::endl;
    return;
  }
  d->state_mutex.lock();
  if (d->openMedia()) {
    std::cout << "[INPUT] stream and threads starting:" << this << ", " << d->source.c_str()  << std::endl;
    if (!d->outputs.empty()) {
      for (auto it = d->outputs.begin(); it != d->outputs.end(); it++) {
        StreamMedia *media = it->second;
        if (!media->d->openMedia()) {
          std::cout << "[INPUT] WARN: failed to open output media:" << media->d->source.c_str()  << std::endl;
        }
      }
    }
    d->startThreads();
    std::cout << "[INPUT] stream and threads started:" << this << ", " << d->source.c_str()  << std::endl;
  }
  d->state_mutex.unlock();
}

void StreamMedia::stop() {
  d->mediaStopping = true;
  std::cout << "[INPUT] stream and threads stopping:" << this << ", " << d->source.c_str()  << std::endl;
  d->state_mutex.lock();
  {
    d->stop_remux.store(true);
    d->stopThreads();
    d->closeMedia();
    d->onStreamFinished();
  }
  d->state_mutex.unlock();
  std::cout << "[INPUT] stream and threads stopped:" << this << ", " << d->source.c_str()  << std::endl;
}



