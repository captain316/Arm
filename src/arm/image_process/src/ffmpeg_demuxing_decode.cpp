#include "ffmpeg_demuxing_decode.h"
#include <string>
#include <iostream>


void ffmpegDemuxingDecode::set_srcFilename(std::string _name)
{
    strInputPath = _name;
}

void ffmpegDemuxingDecode::safeDelete()
{
    //使用try catch执行
    if (m_pFmtCtx != nullptr) {
        avio_close(m_pFmtCtx->pb); 
        avformat_free_context(m_pFmtCtx);
        m_pFmtCtx = nullptr;
    }
    if(m_pCodecCtx != nullptr) {
        avcodec_free_context(&m_pCodecCtx);
        m_pCodecCtx = nullptr;
    }
    if (m_pSwsCtx != nullptr) {
        sws_freeContext(m_pSwsCtx);
        m_pSwsCtx = nullptr;
    }
    if (pFrame_bgr->data != NULL) {
        av_freep(pFrame_bgr->data);    
    }
    if (pInputOptions != nullptr) {
        av_dict_free(&pInputOptions);
    }
    if (pFrame_bgr != nullptr) {
        av_frame_free(&pFrame_bgr);
    }
    if (pFrame != nullptr) {
        av_frame_free(&pFrame);
    } 
    pCodec = nullptr;
}


int ffmpegDemuxingDecode::init_onlyWrite(const char* _path)
{
    strOutputPath = _path;
    if (strInputPath.empty()) {
        std::cout << "ffmpegDemuxingDecode.init -- strInputPath is null" << std::endl;                  ////程序不能崩
        bIsInit = false;
        return -1;
    }

    avformat_network_init();   
    avdevice_register_all();                                            //网络初始化,如果使用旧GnuTLS或OpenSSL库,则需要在库使用前初始化这个函数
    m_pFmtCtx = avformat_alloc_context();                                   //申请一个解复用器上下文格式的内存，主要存储视音频封装格式中包含的信息
    av_dict_set(&pInputOptions,"rtsp_transport", "tcp", 0);                 //字典中添加一对,rtsp_transport的值为tcp
    av_dict_set(&pInputOptions, "allowed_media_types", "video", 0);
    //av_dict_set(&pInputOptions, "buffer_size", "40960000", 0);            //udp设置缓存大小，1080p可将值调大 40M  可以试着增大
	av_dict_set(&pInputOptions, "stimeout", "20000000", 0);                 //字典中添加一对,stimeout值为10000000(超时参数stimeout,单位us)
    av_dict_set(&pInputOptions, "max_delay", "5000000", 0);                 //只对tcp有用，设置ACK时间延迟5秒
    int nRet = avformat_open_input(&m_pFmtCtx, strInputPath.c_str(), nullptr, &pInputOptions);//打开输入流,第三个nullptr自动检测格式,也可以指定,返回0表示成功,负数表示失败
    if (nRet < 0) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_open_input failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    nRet = avformat_find_stream_info(m_pFmtCtx, nullptr);                   //读取媒体包的文件获取流信息,因为有些媒体流比如(MPEG)是没有header的,需要另外获取流信息,但无法保证读取出所有解码器格式
    if (nRet < 0) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_find_stream_info Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    for (size_t i = 0; i < m_pFmtCtx->nb_streams; i++) {                    //遍历媒体流的所有元素，流媒体结构体->第i个流->编解码器参数信息->编解码器类型. 直到类型等于视频类型,就是跳过header
        if (m_pFmtCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {                               
            m_nStreamIdx = static_cast<int>(i);                             //记录第一个流ID为i
            break;
        }
    }
    if (m_nStreamIdx == -1) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_find_stream_info Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    fpSave = fopen(strOutputPath, "wb");
    bIsInit = true;
    return 0;
}


int ffmpegDemuxingDecode::writeStreamToFile()
{
    if (!bIsInit) {
        std::cout<< "ffmpegDemuxingDecode.writeStreamToFile -- bIsInit is false" << std::endl;
        safeDelete();
        return -1;
    }

    int nRet = av_read_frame(m_pFmtCtx, &packet);
    if (nRet >= 0) {
        if ((packet.buf != nullptr) && (packet.stream_index == m_nStreamIdx)) {
            fwrite(packet.data, 1, packet.size, fpSave);
            fflush(fpSave);
        }
        av_packet_unref(&packet);
    }
    return 0;
}

int ffmpegDemuxingDecode::init()
{
    if (strInputPath.empty()) {
        std::cout << "ffmpegDemuxingDecode.init -- strInputPath is null" << std::endl;                  ////程序不能崩
        bIsInit = false;
        return -1;
    }

    avformat_network_init();                                                //网络初始化,如果使用旧GnuTLS或OpenSSL库,则需要在库使用前初始化这个函数
    avdevice_register_all();
    m_pFmtCtx = avformat_alloc_context();                                   //申请一个解复用器上下文格式的内存，主要存储视音频封装格式中包含的信息
    av_dict_set(&pInputOptions,"rtsp_transport", "tcp", 0);                 //字典中添加一对,rtsp_transport的值为tcp
    av_dict_set(&pInputOptions, "allowed_media_types", "video", 0);
    //av_dict_set(&pInputOptions, "buffer_size", "40960000", 0);            //udp设置缓存大小，1080p可将值调大 40M  可以试着增大
	av_dict_set(&pInputOptions, "stimeout", "20000000", 0);                 //字典中添加一对,stimeout值为10000000(超时参数stimeout,单位us)
    av_dict_set(&pInputOptions, "max_delay", "5000000", 0);                 //只对tcp有用，设置ACK时间延迟5秒

    int nRet = avformat_open_input(&m_pFmtCtx, strInputPath.c_str(), nullptr, &pInputOptions);//打开输入流,第三个nullptr自动检测格式,也可以指定,返回0表示成功,负数表示失败
    if (nRet < 0) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_open_input failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    m_pFmtCtx->flags |= AVFMT_FLAG_NOBUFFER;                                //avformat_find_stream_info探测的码流不用于显示
    nRet = avformat_find_stream_info(m_pFmtCtx, nullptr);                   //读取媒体包的文件获取流信息,因为有些媒体流比如(MPEG)是没有header的,需要另外获取流信息,但无法保证读取出所有解码器格式
    if (nRet < 0) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_find_stream_info Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    // av_dump_format(m_pFmtCtx,0,strInputPath.c_str(),0);                  //在终端输出视频流的信息
    for (size_t i = 0; i < m_pFmtCtx->nb_streams; i++) {                    //遍历媒体流的所有元素，流媒体结构体->第i个流->编解码器参数信息->编解码器类型. 直到类型等于视频类型,就是跳过header
        if (m_pFmtCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {                               
            m_nStreamIdx = static_cast<int>(i);                             //记录第一个流ID为i
            break;
        }
    }
    if (m_nStreamIdx == -1) {
        std::cout << "ffmpegDemuxingDecode.init -- avformat_find_stream_info Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }

    m_pCodecCtx = avcodec_alloc_context3(nullptr);                          //为解码器结构体分配内存,并初始化缺省值
    if (m_pCodecCtx == nullptr) {
        std::cout << "ffmpegDemuxingDecode.init -- avcodec_alloc_context3 Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    
    auto* pStream = m_pFmtCtx->streams[m_nStreamIdx];                       //指向第一个视频数据流
    nRet = avcodec_parameters_to_context(m_pCodecCtx, pStream->codecpar);   //用编码器参数中的值填充编码器结构体
    if (nRet < 0) {
        std::cout<< "ffmpegDemuxingDecode.init -- avcodec_parameters_to_context Failed!\n" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }

    m_nInputHeight = m_pCodecCtx->height;                                   //输入的高度
    m_nInputWidth =  m_pCodecCtx->width;                                    //输入的宽度
   
    pCodec = avcodec_find_decoder(m_pCodecCtx->codec_id);                   //编解码器指针指向已注册编解码器
    if (pCodec == nullptr) {
        std::cout << "ffmpegDemuxingDecode.init -- Cant find Codec" <<std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    if(strInputPath[0] == '/' && strInputPath[1] == 'd') {
        m_pSwsCtx = sws_getContext(m_nInputWidth, m_nInputHeight, AV_PIX_FMT_YUYV422,
                                m_nInputWidth, m_nInputHeight,
                                AV_PIX_FMT_BGR24, SWS_BICUBIC, nullptr, nullptr, nullptr);//按输入图像的宽度和高度设置输出的宽度和高度,像素格式由YUV4:2:0变成BGR8:8:8
    } else {
        m_pSwsCtx = sws_getContext(m_nInputWidth, m_nInputHeight, AV_PIX_FMT_YUV420P,
                                m_nInputWidth, m_nInputHeight,
                                AV_PIX_FMT_BGR24, SWS_BICUBIC, nullptr, nullptr, nullptr);//按输入图像的宽度和高度设置输出的宽度和高度,像素格式由YUV4:2:0变成BGR8:8:8
    }
    
    if (m_pSwsCtx == nullptr) {
        std::cout<< "ffmpegDemuxingDecode.init -- Cant create Sws" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }
    m_pCodecCtx->skip_frame = AVDISCARD_NONKEY;                             //跳过非关键帧

    nRet = avcodec_open2(m_pCodecCtx, pCodec, &pCodecOptions);              //初始化音视频编解码器的AVCodecContext,根据编解码器类型和参数设置音视频解码器的上下文
                                                                            //此处可以添加参数设置解码器 m_pCodecCtx->thread_count = 1
    if (nRet < 0) {
        std::cout << "ffmpegDemuxingDecode.init -- avcodec_open2 Failed" << std::endl;
        safeDelete();
        bIsInit = false;
        return -1;
    }

    pFrame = av_frame_alloc();//创建解压缩的输出数据帧,并分配空间 
    pFrame_bgr = av_frame_alloc();
    nRet = av_image_alloc(pFrame_bgr->data, pFrame_bgr->linesize, m_nInputWidth, m_nInputHeight, AV_PIX_FMT_BGR24, 1);

    if (nRet < 0) {
        safeDelete();
        bIsInit = false;
        return -1;
    }
    bIsInit = true;
    return 0;
}

cv::Mat ffmpegDemuxingDecode::demux_decode_a_frame()
{
    if (!bIsInit) {
        std::cout<< "ffmpegDemuxingDecode.demux_decode_a_frame -- bIsInit is false" << std::endl;
        safeDelete();
        return frame;
    }
    int nRet = 0;
    bool isDecodeAFrame = false;
decode:
    nRet = av_read_frame(m_pFmtCtx, &packet);
    if (nRet >= 0) {
        if ((packet.buf != nullptr) && (packet.stream_index == m_nStreamIdx)) {     //判断包非空
            nRet = avcodec_send_packet(m_pCodecCtx, &packet);                       //将包发送给编解码器结构体,调用前必须用avcodec_open2打开编解码器结构体,返回0表示成功
            if(nRet != 0 ) {
                std::cout << "ffmpegDemuxingDecode.demux_decode_a_frame -- avcodec_send_packet failed" << std::endl;
                av_packet_unref(&packet);
                return frame;
            }
            //std::cout << "packet.dts = " <<  packet.dts /90000 << std::endl;      //帧的解码时间（无B帧时和显示时间pts一致）
            while (true) {                                                          //可能一个包里有多帧 一般是一帧
                if (avcodec_receive_frame(m_pCodecCtx, pFrame) == 0) {              //将解码器解码的数据给输出数据帧
                    // std::cout << "pFrame->coded_picture_number = " << pFrame->coded_picture_number << std::endl;
                    int ret = sws_scale(m_pSwsCtx, (const uint8_t* const *)pFrame->data, pFrame->linesize, 0 , m_nInputHeight, pFrame_bgr->data, pFrame_bgr->linesize); //将yuv格式转换成rgb24                                      
                    frame = cv::Mat(m_nInputHeight, m_nInputWidth, CV_8UC3, pFrame_bgr->data[0]);//将数据赋值给frame，对于RGB24来说，数据只会存在data[0]中
                    isDecodeAFrame = true;  
                }
                else {
                    break;
                }
            }
        }
    }
    else {                                                                          //解封装 解码完成  进入结束
        std::cout<< "ffmpegDemuxingDecode.demux_decode_a_frame -- decode finish" << std::endl;
        return frame;
    }

    av_packet_unref(&packet);

    if (!isDecodeAFrame) {
        goto decode;   
    }
    return frame;
}
