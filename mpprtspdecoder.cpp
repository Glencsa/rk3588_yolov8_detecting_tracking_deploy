#include "mpprtspdecoder.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/syscall.h>
MppRtspDecoder::MppRtspDecoder(std::string url)
{
    this->url = url;
}

MppRtspDecoder::~MppRtspDecoder()
{
    if (pFormatCtx)
    {
        avformat_close_input(&pFormatCtx);
        pFormatCtx = NULL;
    }

    if (av_packet)
    {
        av_free(av_packet);
        av_packet = NULL;
    }

    if (data)
    {
        if (data->pkt_grp) {
            mpp_buffer_group_clear(data->pkt_grp);
            data->pkt_grp = NULL;
        }
        if (data->frm_grp) {
//            mpp_buffer_group_put(data->frm_grp);
            mpp_buffer_group_clear(data->frm_grp);
            data->frm_grp = NULL;
        }
        if (data->fp_output) {
            fclose(data->fp_output);
            data->fp_output = NULL;
        }

        if (data->fp_input) {
            fclose(data->fp_input);
            data->fp_input = NULL;
        }

        if (data->ctx) {
            mpp_destroy(data->ctx);
            data->ctx = NULL;
        }

        if (data->frame) {
            mpp_frame_deinit(&(data->ctx));
            data->ctx = NULL;
        }

        if (data->packet) {
            mpp_packet_deinit(&(data->packet));
            data->packet = NULL;
        }

        delete data;
        data = NULL;
    }

}

int MppRtspDecoder::init()
{
    AVDictionary *options = NULL;

    avformat_network_init();
    av_dict_set(&options, "buffer_size", "1024000", 0); //设置缓存大小,1080p可将值跳到最大
    av_dict_set(&options, "rtsp_transport", "tcp", 0); //以tcp的方式打开,
    av_dict_set(&options, "stimeout", "5000000", 0);  //设置超时断开链接时间，单位us
    av_dict_set(&options, "max_delay", "500000", 0); //设置最大时延

    pFormatCtx = avformat_alloc_context(); //用来申请AVFormatContext类型变量并初始化默认参数,申请的空间

    //打开网络流或文件流
    if (avformat_open_input(&pFormatCtx, url.c_str(), NULL, &options) != 0)
    {
        printf("Couldn't open input stream.\n");
        return -1;
    }

    av_dict_free(&options);

    //获取视频文件信息
    if (avformat_find_stream_info(pFormatCtx, NULL)<0)
    {
        printf("Couldn't find stream information.\n");
        return -1;
    }


    //查找码流中是否有视频流
    int videoindex = -1;
    unsigned i = 0;
    for (i = 0; i<pFormatCtx->nb_streams; i++)
        if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            printf("find video stream!\n");
            videoindex = i;
            break;
        }
    if (videoindex == -1)
    {
        printf("Didn't find a video stream.\n");
        return -1;
    }


    av_packet = (AVPacket *)av_malloc(sizeof(AVPacket)); // 申请空间，存放的每一帧数据 （h264、h265）

    //// 初始化
    MPP_RET ret         = MPP_OK;

    // base flow context
    MppCtx ctx          = NULL;
    MppApi *mpi         = NULL;

    // input / output
    MppFrame  frame     = NULL;

    MpiCmd mpi_cmd      = MPP_CMD_BASE;
    MppParam param      = NULL;
    RK_U32 need_split   = 1;
//    MppPollType timeout = 5;

    // paramter for resource malloc
    MppCodingType type  = MPP_VIDEO_CodingAVC;

    // resources
    size_t packet_size  = 8*1024;
\
    data = new  MpiDecLoopData2;

    memset(data, 0, sizeof(MpiDecLoopData2));

//    data.fp_output = fopen("./tenoutput.yuv", "w+b");
//    if (NULL == data.fp_output) {
//        printf("failed to open output file %s\n", "tenoutput.yuv");
//        deInit(&packet, &frame, ctx, buf, data);
//    }

    // decoder demo
    ret = mpp_create(&ctx, &mpi);

    if (MPP_OK != ret) {
        printf("mpp_create failed\n");
        return -1;
    }

    // NOTE: decoder split mode need to be set before init
    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        printf("mpi->control failed\n");
        return -1;
    }

    mpi_cmd = MPP_SET_INPUT_BLOCK;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        printf("mpi->control failed\n");
        return -1;
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, type);
    if (MPP_OK != ret) {
        printf("mpp_init failed\n");
        return -1;
    }

    data->ctx            = ctx;
    data->mpi            = mpi;
    data->eos            = 0;
    data->packet_size    = packet_size;
    data->frame          = frame;
    data->frame_count    = 0;

    return 0;
}


void dump_mpp_frame_to_file(MppFrame frame, FILE *fp)
{
    RK_U32 width    = 0;
    RK_U32 height   = 0;
    RK_U32 h_stride = 0;
    RK_U32 v_stride = 0;

    MppBuffer buffer    = NULL;
    RK_U8 *base = NULL;

    width    = mpp_frame_get_width(frame);
    height   = mpp_frame_get_height(frame);
    h_stride = mpp_frame_get_hor_stride(frame);
    v_stride = mpp_frame_get_ver_stride(frame);
    buffer   = mpp_frame_get_buffer(frame);

    base = (RK_U8 *)mpp_buffer_get_ptr(buffer);
    RK_U32 buf_size = mpp_frame_get_buf_size(frame);
    size_t base_length = mpp_buffer_get_size(buffer);

    RK_U32 i;
    RK_U8 *base_y = base;
    RK_U8 *base_c = base + h_stride * v_stride;

    //保存为YUV420sp格式
    /*for (i = 0; i < height; i++, base_y += h_stride)
    {
        fwrite(base_y, 1, width, fp);
    }
    for (i = 0; i < height / 2; i++, base_c += h_stride)
    {
        fwrite(base_c, 1, width, fp);
    }*/

    //保存为YUV420p格式
    for(i = 0; i < height; i++, base_y += h_stride)
    {
        fwrite(base_y, 1, width, fp);
    }
    for(i = 0; i < height * width / 2; i+=2)
    {
        fwrite((base_c + i), 1, 1, fp);
    }
    for(i = 1; i < height * width / 2; i+=2)
    {
        fwrite((base_c + i), 1, 1, fp);
    }
}

// size_t mpp_buffer_group_usage(MppBufferGroup group)
// {
//     if (NULL == group)
//     {
//         printf("input invalid group %p\n", group);
//         return MPP_BUFFER_MODE_BUTT;
//     }

//     MppBufferGroupImpl *p = (MppBufferGroupImpl *)group;
//     return p->usage;

    
// }

int MppRtspDecoder::decode_simple( cv::Mat rgbImg )
{
    RK_U32 pkt_done = 0;
    RK_U32 pkt_eos  = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;
    // char   *buf = data->buf;
    MppPacket packet = NULL;
    MppFrame  frame  = NULL;

    if (av_read_frame(pFormatCtx, av_packet) < 0)
    {
        printf("av_read_frame get packet failed!");
        return -1;
    }


    ret = mpp_packet_init(&packet, av_packet->data, av_packet->size);
    mpp_packet_set_pts(packet, av_packet->pts);

    do {
        RK_S32 times = 5;
        // send the packet first if packet is not done
        if (!pkt_done) {
            ret = mpi->decode_put_packet(ctx, packet);
            if (MPP_OK == ret)
                pkt_done = 1;
        }

        // then get all available frame and release
        do {
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;

            try_again:
            ret = mpi->decode_get_frame(ctx, &frame);
            if (MPP_ERR_TIMEOUT == ret) {
                if (times > 0) {
                    times--;
                    usleep(2000);
                    goto try_again;
                }
                printf("decode_get_frame failed too much time\n");
            }
            if (MPP_OK != ret) {
                printf("decode_get_frame failed ret %d\n", ret);
                break;
            }

            if (frame) {
                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                    RK_U32 buf_size = mpp_frame_get_buf_size(frame);

                    printf("decode_get_frame get info changed found\n");
                    printf("decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d",
                            width, height, hor_stride, ver_stride, buf_size);

                    ret = mpp_buffer_group_get_internal(&data->frm_grp, MPP_BUFFER_TYPE_ION);
                    if (ret) {
                        printf("get mpp buffer group  failed ret %d\n", ret);
                        break;
                    }
                    mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, data->frm_grp);

                    mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                } else {
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) {
                        printf("decoder_get_frame get err info:%d discard:%d.\n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
                    data->frame_count++;
                    printf("decode_get_frame get frame %d\n", data->frame_count);
                    YUV420SP2Mat(frame, rgbImg);
                   if (data->fp_output && !err_info){

                    //    cv::imwrite("./"+std::to_string(count++)+".jpg", rgbImg);

//                        dump_mpp_frame_to_file(frame, data->fp_output);
                   }
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);

                frame = NULL;
                get_frm = 1;
            }

            // try get runtime frame memory usage
            if (data->frm_grp) {
                size_t usage = mpp_buffer_group_usage(data->frm_grp);
                if (usage > data->max_usage)
                    data->max_usage = usage;
            }

            // if last packet is send but last frame is not found continue
            if (pkt_eos && pkt_done && !frm_eos) {
                usleep(1*1000);
                continue;
            }

            if (frm_eos) {
                printf("found last frame\n");
                break;
            }

            if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
                data->eos = 1;
                break;
            }

            if (get_frm)
                continue;
            break;
        } while (1);

        if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
            data->eos = 1;
            printf("reach max frame number %d\n", data->frame_count);
            break;
        }

        if (pkt_done)
            break;

        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 3ms
         * * is enough.
         */
        usleep(3*1000);
    } while (1);
    mpp_packet_deinit(&packet);
    if (av_packet != NULL)
        av_packet_unref(av_packet);

    return ret;
}

void MppRtspDecoder::YUV420SP2Mat(MppFrame frame, cv::Mat rgbImg ) {
    RK_U32 width = 0;
    RK_U32 height = 0;
    RK_U32 h_stride = 0;
    RK_U32 v_stride = 0;

    MppBuffer buffer = NULL;
    RK_U8 *base = NULL;

    width = mpp_frame_get_width(frame);
    height = mpp_frame_get_height(frame);
    h_stride = mpp_frame_get_hor_stride(frame);
    v_stride = mpp_frame_get_ver_stride(frame);
    buffer = mpp_frame_get_buffer(frame);

    base = (RK_U8 *)mpp_buffer_get_ptr(buffer);
    RK_U32 buf_size = mpp_frame_get_buf_size(frame);
    size_t base_length = mpp_buffer_get_size(buffer);
    // mpp_log("base_length = %d\n",base_length);

    RK_U32 i;
    RK_U8 *base_y = base;
    RK_U8 *base_c = base + h_stride * v_stride;

    cv::Mat yuvImg;
    yuvImg.create(height * 3 / 2, width, CV_8UC1);

    //转为YUV420p格式
    int idx = 0;
    for (i = 0; i < height; i++, base_y += h_stride) {
        //        fwrite(base_y, 1, width, fp);
        memcpy(yuvImg.data + idx, base_y, width);
        idx += width;
    }
    for (i = 0; i < height / 2; i++, base_c += h_stride) {
        //        fwrite(base_c, 1, width, fp);
        memcpy(yuvImg.data + idx, base_c, width);
        idx += width;
    }
    //这里的转码需要转为RGB 3通道， RGBA四通道则不能检测成功
    cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2RGB_NV21);
}

