//
// Created by zifeng on 2024/7/17.
//

#ifndef FFMPEG_DECODER_FFMPEG_H
#define FFMPEG_DECODER_FFMPEG_H

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/avutil.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/mathematics.h>
#include <libavutil/rational.h>
#include <libavutil/timestamp.h>
#include <libavutil/intreadwrite.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libswresample/swresample.h>

#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libavutil/timestamp.h>
}


#endif //FFMPEG_DECODER_FFMPEG_H
