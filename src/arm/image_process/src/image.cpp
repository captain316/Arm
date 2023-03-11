#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "nanodet_openvino.h"
#include <string>
#include <iostream>
#include "ffmpeg_demuxing_decode.h"
#include <algorithm>
#include <chrono>

#include <csignal>
void signalHandle(int signum)
{
	std::cout << "程序停止" << std::endl;
	exit(signum);
}

struct object_rect {
    int x;
    int y;
    int width;
    int height;
};

struct object_center {
    int cx;
    int cy;
    std::string label = "";
    int label_int;
};


double GetIOU(cv::Rect_<float> bb_test, cv::Rect_<float> bb_gt)
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

// 去光照
void normalizeRGB1(const cv::Mat& _src,cv::Mat& dst)
{
    assert( CV_8UC3 == _src.type() );

    cv::Mat src;
    _src.convertTo(src,CV_32FC3);

    for(auto iter=src.begin<cv::Vec3f>(); iter != src.end<cv::Vec3f>(); ++iter)
    {
        float denominator = 1.0f /((*iter)[0]+(*iter)[1]+(*iter)[2]+FLT_EPSILON);

        (*iter)[0] = (*iter)[0]*denominator;
        (*iter)[1] = (*iter)[1]*denominator;
        (*iter)[2] = (*iter)[2]*denominator;
    }

    //将结果量化到[0,255]范围
    //NORM_MINMAX表示线性量化，CV_8UC3表示将图像转回
    cv::normalize(src,src,0,255,cv::NORM_MINMAX);
    cv::convertScaleAbs(src,dst);
}

// 高效版本
void normalizeRGB(const cv::Mat &_src, cv::Mat &dst)
{
    assert( CV_8UC3 == _src.type() );

    cv::Mat src;
    _src.convertTo(src,CV_32FC3);

    //拆分通道
    std::vector<cv::Mat> channels;
    cv::split(src,channels);

    //各通道归一化
    cv::Mat denominator = 1.0 / (channels.at(0) + channels.at(1) + channels.at(2) + FLT_EPSILON);

    //对应元素点乘
    channels.at(0) = channels.at(0).mul(denominator);
    channels.at(1) = channels.at(1).mul(denominator);
    channels.at(2) = channels.at(2).mul(denominator);

    //合并通道
    cv::merge(channels,src);

    // 将结果量化到[0,255]范围
    // NORM_MINMAX表示线性量化，CV_8UC3表示将图像转回
    cv::normalize(src,src,0,255,cv::NORM_MINMAX);
    cv::convertScaleAbs(src,dst);
}

void draw_bboxes(cv::Mat& image, const std::vector<BoxInfo>& bboxes, object_rect effect_roi)
{
    
    cv::Mat dst(image.size(), CV_32FC3);
    cv::Mat imgray;
    
    
    //去除光照
    normalizeRGB(image, dst);
    
    cv::imshow("去光照", dst);
    
    cv::cvtColor(dst, imgray, cv::COLOR_BGR2GRAY);
    // cv::imshow("去光照后的灰度图像", imgray);
    cv::GaussianBlur(imgray, imgray, cv::Size(3, 3), 0.5, 0.5);
    // cv::imshow("平滑灰度图像", imgray);
    
    cv::Mat test_ding = imgray.clone();
    std::vector<object_center> boxes_center;

    static const char* class_names[] = {"person", "bicycle", "car", "motorcycle", "airplane", "bus",
                                        "train", "truck", "boat", "traffic light", "fire hydrant",
                                        "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                                        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                                        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                        "skis", "snowboard", "sports ball", "kite", "baseball bat",
                                        "baseball glove", "skateboard", "surfboard", "tennis racket",
                                        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                                        "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                                        "hot dog", "pizza", "donut", "cake", "chair", "couch",
                                        "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                                        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                                        "toaster", "sink", "refrigerator", "book", "clock", "vase",
                                        "scissors", "teddy bear", "hair drier", "toothbrush" 
                                        };
    int src_w = image.cols;
    int src_h = image.rows;
    int dst_w = effect_roi.width;
    int dst_h = effect_roi.height;
    float width_ratio = (float)src_w / (float)dst_w;
    float height_ratio = (float)src_h / (float)dst_h;

    for (size_t i = 0; i < bboxes.size(); i++) {
        object_center box_center;
        const BoxInfo& bbox = bboxes[i];
        cv::Scalar color = cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1], color_list[bbox.label][2]);
        
        float lx = (bbox.x1 - effect_roi.x) * width_ratio;
        float ly = (bbox.y1 - effect_roi.y) * height_ratio;
        float rx = (bbox.x2 - effect_roi.x) * width_ratio;
        float ry = (bbox.y2 - effect_roi.y) * height_ratio;

        // cv::rectangle(image, cv::Rect(cv::Point((bbox.x1 - effect_roi.x) * width_ratio, (bbox.y1 - effect_roi.y) * height_ratio), 
        //                               cv::Point((bbox.x2 - effect_roi.x) * width_ratio, (bbox.y2 - effect_roi.y) * height_ratio)), color);
        // cv::rectangle(image, cv::Rect(cv::Point(lx, ly), 
                                    //   cv::Point(rx, ry)), color);
        
        box_center.cx = lx + (rx - lx) / 2;
        box_center.cy = ly + (ry - ly) / 2;
        box_center.label_int = bbox.label;
        box_center.label = class_names[bbox.label];
        boxes_center.emplace_back(box_center);
        // 中心点画圆
        // cv::circle(image, cv::Point(box_center.cx, box_center.cy), 1, color);

        char text[256];
        // sprintf(text, "%s %.1f%%",  class_names[bbox.label], bbox.score * 100);
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
        //
        // cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),color, -1);
        
        // cv::putText(image, text, cv::Point(x, y + label_size.height),
        // cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }
    // cv::threshold(imgray, imgray, 150, 255, cv::THRESH_BINARY);
    // 去光照后的 100
    cv::threshold(imgray, imgray, 90, 255, cv::THRESH_BINARY);

    // cv::imshow("去光照后的二值图像", imgray);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(imgray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    
    for(int index = 0; index < contours.size(); ++index) {
        bool flag = false;
        // if(contours[index][0].x >= 320 || contours[index][0].y >= 320 || contours[index][0].x <= 10 || contours[index][0].y <= 10) {
        //     continue;
        // }
        // 找到各个分割区域的最小矩形，rect包括矩形的中心点、W、L、angle
        cv::RotatedRect rect = cv::minAreaRect(contours[index]);
        // 矩形的四个点：左下、左上、右上、右下
        cv::Mat boxPts;
        cv::boxPoints(rect, boxPts);
        // cout << boxPts << endl;
        std::vector<cv::Point> draw_box;
        // cout << "type is " << boxPts.type() << endl;
        for(int row = 0; row < boxPts.rows; ++row) {
            // uchar* data = boxPts.ptr(row);
            cv::Point p;
            float t1 = boxPts.at<float>(row, 0);
            float t2 = boxPts.at<float>(row, 1);
            p.x = int(t1);
            p.y = int(t2);
            // cout << "ROW = " << row << ", p.x = " << p.x << ", p.y = " << p.y << endl;
            draw_box.push_back(p);
        }
        // cout << draw_box.size() << endl;
        // 矩形的中心值
        cv::Point2f center = rect.center; 
        int cx = int(center.x);
        int cy = int(center.y);
        for(int j = 0; j < boxes_center.size(); ++j) {
            int x_err = cx - boxes_center[j].cx;
            int y_err = cy - boxes_center[j].cy;
            // 这儿加限制条件
            int area = rect.size.height * rect.size.width;
            // if( (pow(x_err, 2) + pow(y_err, 2)) < 500 && (boxes_center[j].label == "banana") && (area < 40000 && area > 2000)) {
            //     flag = true;
            //     break;
            // }
            // 去光照
            if( (pow(x_err, 2) + pow(y_err, 2)) < 2000 && (boxes_center[j].label == "banana") && (area > 2000)) {
                flag = true;
                break;
            }
        } 
        if(flag) {
            cv::circle(image, cv::Point(cx, cy), 1, (0,0,255));
            cv::Size2f size = rect.size;
            // cout << "width = " << size.width << ", height = " << size.height << endl;
            double angle = 0;
            if(size.width < size.height) {
                angle = rect.angle + 90.0;
                std::string text = "left" + std::to_string(angle);
                cv::putText(image, text, cv::Point(cx + 10, cy - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
            } else {
                angle = rect.angle + 180;
                std::string text = "right" + std::to_string(angle);
                cv::putText(image, text, cv::Point(cx + 10, cy - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
            }
            std::vector<std::vector<cv::Point>> input(1);
            input[0] = draw_box;
            cv::drawContours(image, input, 0, cv::Scalar(0,0,255), 1);
            
        } else {
            continue;
        }
    }
    
    cv::waitKey(1);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image");
    ros::NodeHandle n;
    ros::Publisher process_image;   // 发布处理后图像
    process_image = n.advertise<sensor_msgs::Image>("image", 500);  //发布image的话题

    auto start = std::chrono::steady_clock::now();
    auto detector = NanoDet("./src/image_process/model/nanodet_m.xml");
    auto end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration<double>(end - start).count();
    std::cout << "*** load model's time(s):\t" << time << std::endl; 
    int min_width = 320, min_height = 320;
    ffmpegDemuxingDecode cap;
    cap.set_srcFilename("/dev/video0");
    cap.init();
    // cv::Mat img = cv::Mat(cv::Size(320,320),CV_8UC3);
    cv::Mat img;
    signal(SIGINT, signalHandle);
    while(true) {
        auto start = std::chrono::steady_clock::now();
        img = cap.demux_decode_a_frame();
        
        object_rect effect_roi;
        cv::Mat resized_img;
        // cv::resize(img,img,cv::Size(min_width,min_height));
        
        resize_uniform(img, resized_img, cv::Size(320, 320), effect_roi);
        
        auto results = detector.detect(resized_img, 0.25, 0.5);
        
        draw_bboxes(img, results, effect_roi);
        
        // cv::imshow("111", img);
        
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        process_image.publish(*image);
         
        cv::waitKey(1);
        // sleep(0.08);
        auto end = std::chrono::steady_clock::now();
        auto time = std::chrono::duration<double>(end - start).count();
        std::cout << "*** for's time(s):\t" << time << std::endl; 
    }
    // ros::spin();
    return 0;

}