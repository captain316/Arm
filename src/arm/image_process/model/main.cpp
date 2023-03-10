#include "nanodet_openvino.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include <set>
#include <deque>
#include <unistd.h>
#include <thread>
#include <mutex>
// #include <tbb/concurrent_queue.h>
#include "Hungarian.h"
#include "KalmanTracker.h"
#include "ffmpeg_demuxing_decode.h"


// using namespace tbb;

#define CNUM 20
#define CAM_NUM 6

Mat frame_queue[CAM_NUM] = {
    cv::Mat(cv::Size(320,320),CV_8UC3),
    cv::Mat(cv::Size(320,320),CV_8UC3),
    cv::Mat(cv::Size(320,320),CV_8UC3),
    cv::Mat(cv::Size(320,320),CV_8UC3),
    cv::Mat(cv::Size(320,320),CV_8UC3),
    cv::Mat(cv::Size(320,320),CV_8UC3)
};
static const char* capture_source[] = { 
    "rtsp://admin:like1234@192.168.1.103:554/mpeg4/ch1/101?transportmode=multicast",
    "rtsp://admin:like1234@192.168.1.104:554/Streaming/Channels/1",
    "rtsp://admin:like1234@192.168.1.105:554/Streaming/Channels/1",
    "rtsp://admin:like1234@192.168.1.106:554/Streaming/Channels/1",
    "rtsp://admin:like1234@192.168.1.107:554/Streaming/Channels/1",
    "rtsp://admin:like1234@192.168.1.108:554/Streaming/Channels/1",
    //"/media/sda/video/hand1.mp4",
    //"/dev/video0",
    //"/media/sda/video/hand1.mp4",
    //"/media/sda/video/hand1.mp4"
    // "../Tokyo_1080p.mp4"
};
std::mutex m;

extern int person_num;
int speed_flag=0;
typedef struct frame_param {
    cv::Rect_<float> box;
    int id=-1;
}frame_param;

typedef struct Tracking_v {
	int id;
	float v;
}Tracking_v;

struct object_rect {
    int x;
    int y;
    int width;
    int height;
};
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}
int resize_uniform(cv::Mat& src, cv::Mat& dst, cv::Size dst_size, object_rect& effect_area)
{
    int w = src.cols;
    int h = src.rows;
    int dst_w = dst_size.width;
    int dst_h = dst_size.height;
    //std::cout << "src: (" << h << ", " << w << ")" << std::endl;
    dst = cv::Mat(cv::Size(dst_w, dst_h), CV_8UC3, cv::Scalar(0));

    float ratio_src = w * 1.0 / h;
    float ratio_dst = dst_w * 1.0 / dst_h;

    int tmp_w = 0;
    int tmp_h = 0;
    if (ratio_src > ratio_dst) {
        tmp_w = dst_w;
        tmp_h = floor((dst_w * 1.0 / w) * h);
    }
    else if (ratio_src < ratio_dst) {
        tmp_h = dst_h;
        tmp_w = floor((dst_h * 1.0 / h) * w);
    }
    else {
        cv::resize(src, dst, dst_size);
        effect_area.x = 0;
        effect_area.y = 0;
        effect_area.width = dst_w;
        effect_area.height = dst_h;
        return 0;
    }

    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(tmp_w, tmp_h));

    if (tmp_w != dst_w) {
        int index_w = floor((dst_w - tmp_w) / 2.0);
        for (int i = 0; i < dst_h; i++) {
            memcpy(dst.data + i * dst_w * 3 + index_w * 3, tmp.data + i * tmp_w * 3, tmp_w * 3);
        }
        effect_area.x = index_w;
        effect_area.y = 0;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else if (tmp_h != dst_h) {
        int index_h = floor((dst_h - tmp_h) / 2.0);
        memcpy(dst.data + index_h * dst_w * 3, tmp.data, tmp_w * tmp_h * 3);
        effect_area.x = 0;
        effect_area.y = index_h;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else {
        printf("error\n");
    }
    return 0;
}

const int color_list[80][3] =
{
    //{255 ,255 ,50}, //bg
    {216 , 82 , 24},
    {236 ,176 , 31},
    {125 , 46 ,141},
    {118 ,171 , 47},
    { 76 ,189 ,237},
    {238 , 19 , 46},
    { 76 , 76 , 76},
    {153 ,153 ,153},
    {255 ,  0 ,  0},
    {255 ,127 ,  0},
    {190 ,190 ,  0},
    {  0 ,255 ,  0},
    {  0 ,  0 ,255},
    {170 ,  0 ,255},
    { 84 , 84 ,  0},
    { 84 ,170 ,  0},
    { 84 ,255 ,  0},
    {170 , 84 ,  0},
    {170 ,170 ,  0},
    {170 ,255 ,  0},
    {255 , 84 ,  0},
    {255 ,170 ,  0},
    {255 ,255 ,  0},
    {  0 , 84 ,127},
    {  0 ,170 ,127},
    {  0 ,255 ,127},
    { 84 ,  0 ,127},
    { 84 , 84 ,127},
    { 84 ,170 ,127},
    { 84 ,255 ,127},
    {170 ,  0 ,127},
    {170 , 84 ,127},
    {170 ,170 ,127},
    {170 ,255 ,127},
    {255 ,  0 ,127},
    {255 , 84 ,127},
    {255 ,170 ,127},
    {255 ,255 ,127},
    {  0 , 84 ,255},
    {  0 ,170 ,255},
    {  0 ,255 ,255},
    { 84 ,  0 ,255},
    { 84 , 84 ,255},
    { 84 ,170 ,255},
    { 84 ,255 ,255},
    {170 ,  0 ,255},
    {170 , 84 ,255},
    {170 ,170 ,255},
    {170 ,255 ,255},
    {255 ,  0 ,255},
    {255 , 84 ,255},
    {255 ,170 ,255},
    { 42 ,  0 ,  0},
    { 84 ,  0 ,  0},
    {127 ,  0 ,  0},
    {170 ,  0 ,  0},
    {212 ,  0 ,  0},
    {255 ,  0 ,  0},
    {  0 , 42 ,  0},
    {  0 , 84 ,  0},
    {  0 ,127 ,  0},
    {  0 ,170 ,  0},
    {  0 ,212 ,  0},
    {  0 ,255 ,  0},
    {  0 ,  0 , 42},
    {  0 ,  0 , 84},
    {  0 ,  0 ,127},
    {  0 ,  0 ,170},
    {  0 ,  0 ,212},
    {  0 ,  0 ,255},
    {  0 ,  0 ,  0},
    { 36 , 36 , 36},
    { 72 , 72 , 72},
    {109 ,109 ,109},
    {145 ,145 ,145},
    {182 ,182 ,182},
    {218 ,218 ,218},
    {  0 ,113 ,188},
    { 80 ,182 ,188},
    {127 ,127 ,  0},
};

void draw_bboxes(const cv::Mat& image, const std::vector<BoxInfo>& bboxes, object_rect effect_roi)
{
    int person_num_in_danger_area = 0;
    // static const char* class_names[] = { "person"};
    int src_w = image.cols;
    int src_h = image.rows;
    int dst_w = effect_roi.width;
    int dst_h = effect_roi.height;
    float width_ratio = (float)src_w / (float)dst_w;
    float height_ratio = (float)src_h / (float)dst_h;

    for (size_t i = 0; i < bboxes.size(); i++) {
        const BoxInfo& bbox = bboxes[i];
        cv::Scalar color = cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1], color_list[bbox.label][2]);
        cv::rectangle(image, cv::Rect(cv::Point((bbox.x1 - effect_roi.x) * width_ratio, (bbox.y1 - effect_roi.y) * height_ratio), 
                                      cv::Point((bbox.x2 - effect_roi.x) * width_ratio, (bbox.y2 - effect_roi.y) * height_ratio)), color);
        char text[256];
        sprintf(text, "%s %.1f%%", "person", bbox.score * 100);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        int x = (bbox.x1 - effect_roi.x) * width_ratio;
        int y = (bbox.y1 - effect_roi.y) * height_ratio - label_size.height - baseLine;
        if (y < 0) {
            y = 0;
        }   
        if (x + label_size.width > image.cols) {
             x = image.cols - label_size.width;
        }
        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),color, -1);
        
        if( (0<=x)&&(x<=65) && (170<=y)&&(y<=200) || (250<=x)&&(x<=315) && (170<=y)&&(y<=200) ) {
            person_num_in_danger_area+=1;
        }
        cv::putText(image, text, cv::Point(x, y + label_size.height),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }
}

void cam_read(int i, NanoDet detector)
{
    int min_width = 320;
	int min_height = 320;
    cv::Mat img = cv::Mat(cv::Size(320,320),CV_8UC3);
    
    auto start = std::chrono::steady_clock::now();
    ffmpegDemuxingDecode cap;
    cap.set_srcFilename(capture_source[i-1]);
    cap.init();
    auto end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "Camera loaded time(ms):\t " << "---" << i << "---" <<time << std::endl;
    while(true) {
        auto frame_start = std::chrono::steady_clock::now();
        img = cap.demux_decode_a_frame();
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_time = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
        std::cout << "frame decode time(ms):\t " << frame_time << std::endl;
        object_rect effect_roi;
        cv::Mat resized_img;
        cv::resize(img,img,cv::Size(min_width,min_height));
        resize_uniform(img, resized_img, cv::Size(320, 320), effect_roi);
        m.lock();
        auto results = detector.detect(resized_img, 0.25, 0.5);
        m.unlock();
        draw_bboxes(img, results, effect_roi);
        m.lock();
        cv::imshow(to_string(i), img);
        cv::waitKey(1);
        m.unlock();
        sleep(0.08);
    }
}

int main(int argc, char** argv)
{
    std::cout<<"start init model"<<std::endl;
    auto start = std::chrono::steady_clock::now();
    auto detector = NanoDet("../model/nanodet_m.xml");
    auto end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration<double>(end - start).count();
    std::cout<<"The model was loaded successfully."<<std::endl;
    std::cout<<"*** loaded model's time(s):\t" << time<<std::endl;

    thread cam0(cam_read, 1, detector);
    thread cam1(cam_read, 2, detector);
    thread cam2(cam_read, 3, detector);
    thread cam3(cam_read, 4, detector);
    thread cam4(cam_read, 5, detector);
    thread cam5(cam_read, 6, detector);

    while (true) {
    }
    return 0;       
}
