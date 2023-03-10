#ifndef FFMPEG_DEMUXING_DECODE_H
#define FFMPEG_DEMUXING_DECODE_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C"
{
    #include <libavutil/imgutils.h>
    #include <libavutil/samplefmt.h>
    #include <libavutil/timestamp.h>
    #include <libswresample/swresample.h>
    #include <libavcodec/avcodec.h>
    #include <libavutil/opt.h>
    #include <libavdevice/avdevice.h>
    #include <libavformat/avformat.h>
    #include <libavutil/mathematics.h>
    #include <libavutil/time.h>
    #include <libavutil/pixfmt.h>
    #include <libswscale/swscale.h>
    #include <libavutil/hwcontext.h>
}

class ffmpegDemuxingDecode
{
public:
    int init();
    int init_onlyWrite(const char* _path);
    int writeStreamToFile();
    cv::Mat demux_decode_a_frame();
    void safeDelete();
    void set_srcFilename(std::string _name);

private:
    AVFormatContext *m_pFmtCtx = nullptr;       //创建音视频格式上下文类型的指针,AVFormatContext描述了一个媒体文件或媒体流的构成和基本信息
    AVCodecContext *m_pCodecCtx = nullptr;      //创建音视频编解码上下文类型的指针,AVCodecContext描述编解码器上下文的数据结构，包含了众多编解码器需要的参数信息
    SwsContext* m_pSwsCtx = nullptr;            //进行图像数据格式的转换以及图片的缩放应用
    AVFrame *pFrame = nullptr;                  //AVFrame结构体一般用于存储原始数据（即非压缩数据，例如对视频来说是YUV，RGB，对音频来说是PCM），此外还包含了一些相关的信息。
    AVPacket packet;                            //接收到的压缩视频数据
    int m_nStreamIdx = -1;//流ID
    int m_nInputHeight = 640;//输入图像高度
    int m_nInputWidth = 480;//输入图像宽度
    AVDictionary* pInputOptions = nullptr;//创建一个字典,存储参数,方便后面打开输入媒体流
    std::string strInputPath = ""; //输入文件路径或url
    const char* strOutputPath; // 输出文件路径
    const AVCodec* pCodec = nullptr;//编解码器指针
    AVDictionary* pCodecOptions = nullptr;//创建一个字典,存储编解码器参数
    AVFrame* pFrame_bgr = nullptr;
    bool bIsInit = false;
    bool bIsFinsh = false;
    cv::Mat frame = cv::Mat(cv::Size(m_nInputWidth, m_nInputHeight), CV_8UC3);

    FILE *fpSave;
};

#endif // FFMPEG_DEMUXING_DECODE_H
