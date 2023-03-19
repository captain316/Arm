/*这个 C++ 头文件，提供了一个名为 ffmpegDemuxingDecode 的类，用于解码音视频文件。
该头文件引用了许多 FFmpeg 库，并使用了一些 FFmpeg 的数据结构。类的主要方法包括：

init()：初始化解码器
init_onlyWrite(const char* _path)：初始化编码器，用于将视频帧写入文件
writeStreamToFile()：将视频帧写入文件
demux_decode_a_frame()：解码一帧视频
safeDelete()：释放解码器和编码器
set_srcFilename(std::string _name)：设置输入文件路径或 url。
在类的构造函数中，它使用了一些默认参数来初始化类的一些变量，
例如输入视频的宽度和高度、流 ID 等等。类还定义了一些私有变量来存储 FFmpeg 相关数据结构的指针和一些配置选项。*/
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
    int init_onlyWrite(const char* _path); //初始化编码器，用于将视频帧写入文件
    int writeStreamToFile();  //将视频帧写入文件
    cv::Mat demux_decode_a_frame();  //解码一帧视频
    void safeDelete();               //释放解码器和编码器
    void set_srcFilename(std::string _name);  //设置输入文件路径或 url。

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
