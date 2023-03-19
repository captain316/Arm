#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <chrono>
#include "nanodet_openvino.h"
#include <string>
#include <iostream>
#include "ffmpeg_demuxing_decode.h"
#include <geometry_msgs/Twist.h>
#include <csignal>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "ur3_move/movingObjectPosition.h"
// #include <tf2/LinearMath/Quaternion.h> 

/*
    本文件为识别动态物体位姿并发布tf变换
*/

/*  
    物体类别替换的话，需要改的是：
    1、double Z_c = 0.400; 83行
    2、496行 :if( (pow(x_err, 2) + pow(y_err, 2)) < 1000 && (boxes_center[j].label == "banana") && (area > 2000)
    3、objectPose.position.z = 0.040; 540行
    4、
*/


bool tf_publish = false;
geometry_msgs::Pose objectPose; // 香蕉的位置
float ang = 0.0;                // 香蕉的姿态

tf::TransformBroadcaster* realBanana = NULL;   // 定义一个广播，发布真实香蕉位姿
tf::Transform realBananaTF;            // 声明一个变量用来存储banana和world之间的坐标变换

tf::TransformBroadcaster* trackBanana = NULL;   // 定义一个广播，发布跟踪香蕉位姿(主要差距在于Z轴)
tf::Transform trackBananaTF;            // 声明一个变量用来存储trackBanana和world之间的坐标变换

bool responsePose(ur3_move::movingObjectPosition::Request& req,
                  ur3_move::movingObjectPosition::Response& res) {
    
    res.receive = true;
    tf_publish = true;
    ROS_INFO("publish tf of banana");
    
    
    return true;
}
/*这段代码定义了相机内参矩阵cameraMatrix、畸变系数矩阵distCoeffs、相机内参矩阵的逆矩阵K_inv、
相机旋转矩阵的逆矩阵R_inv和相机平移矩阵T。这些参数用于相机标定和矫正畸变，以及计算相机和物体之间的位姿关系。*/
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 536.19728, 0.0, 307.47167, 
                                                  0.0, 535.6534, 234.31344,
                                                  0.0, 0.0, 1.0);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.038426, -0.117825, -0.003219, -0.004458, 0.0);
cv::Mat K_inv = (cv::Mat_<double>(3, 3) << 0.001865, 0.0, -0.573430, 
                                           0.0, 0.001867, -0.437435,
                                           0.0, 0.0, 1.0);

cv:: Mat R_inv = (cv::Mat_<double>(3, 3) << 0.9950767, 0.0749812, 0.0648082, 
                                           -0.0805475, 0.9928554, 0.0880358,
                                           -0.0577441, -0.0928225, 0.9940068);
cv::Mat T = (cv::Mat_<double>(3, 1) << 0.02563174, 0.05575485, 0.15251193);

// 老版本的
// cv:: Mat R_inv = (cv::Mat_<double>(3, 3) << 0.997716, 0.062318, 0.026045, 
//                                            -0.062709, 0.997927, 0.014465,
//                                            -0.025089, -0.016065, 0.999556);
// cv::Mat T = (cv::Mat_<double>(3, 1) << 0.02469801, 0.05477644, 0.12691288);
// 0.02469801, 0.05477644, 0.12691288 原来的
// -0.02915 -0.054776 -0.126913 // 自己测的
cv::Mat finger2baseRotation = (cv::Mat_<double>(3, 3) << 0.0, 1.0, 0.0, 
                                                         1.0, 0.0, 0.0,
                                                         0.0, 0.0, -1.0);
// cv::Mat finger2baseTrans = (cv::Mat_<double>(3, 1) << -0.100, 0.112, 0.342);

// 香蕉0.44，其他0.40
double Z_c = 0.440;


/*这是一个用于计算真实位置的函数，其输入参数包括一个像素坐标 pixelPosition 和
一个 TransformListener listener，输出结果为一个在 base_link 坐标系下的位置向量 baselinkFrame。

在函数中，首先通过反向变换将像素坐标转换到相机坐标系下，并将其保存在 fingerFrame 中。
然后通过 TransformListener 获取 fingers_frame 到 base_link 的变换 transform，
并将其转换为 geometry_msgs::Pose 格式的 pose。接着将 pose 中的位置信息转换为一个大小为 3x1 的矩阵 finger2baseTrans。

最后，根据手爪到 base_link 的旋转关系 finger2baseRotation，将 fingerFrame 
转换为 base_link 坐标系下的坐标 baselinkFrame。最终结果即为函数的输出。*/
cv::Mat computeRealPosition(cv::Mat& pixelPosition, tf::TransformListener& listener) 
{   
    tf::StampedTransform transform;
    
    cv:: Mat fingerFrame = R_inv * K_inv * pixelPosition - R_inv * T;
    // std::cout << "Fingers_frame is :" << std::endl;
    // std::cout << fingerFrame << std::endl;
    // 这儿有问题，这儿的finger到base不是固定的了

    geometry_msgs::Pose pose; 
    try
    {
        listener.waitForTransform("base_link", "fingers_frame", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link", "fingers_frame", ros::Time(0), transform);    
                        
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    poseTFToMsg(transform, pose);

    cv::Mat finger2baseTrans = (cv::Mat_<double>(3, 1) << pose.position.x, pose.position.y, pose.position.z);
    cv::Mat baselinkFrame = finger2baseRotation * fingerFrame + finger2baseTrans;
    // std::cout << "base_link frame is :" << std::endl;
    // std::cout << baselinkFrame << std::endl;
    return baselinkFrame;
}
// 要知道世界坐标系，需要求cameraMatrix的逆阵、旋转平移阵中旋转的逆阵
// K = cameraMatrix, R\T(旋转平移矩阵);
// (X_w, Y_w, Z_w) = R_inv * K_inv * Z_c * (u, v, 1) - R_inv * T


void signalHandle(int signum)
{
	std::cout << "程序停止" << std::endl;
	exit(signum);
}
ros::Publisher process_image;
ros::Publisher worldPosition;
ros::Subscriber original_image;

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

/*这个函数是用来将输入的图像进行统一的缩放操作，使其适配目标尺寸。
函数参数包括输入的原图像 src，缩放后的目标图像 dst，目标尺寸 dst_size，
以及一个用来记录缩放效果区域的矩形 effect_area。

函数首先获取原图像和目标图像的尺寸，并计算出它们的宽高比 ratio_src 和 ratio_dst。
如果原图像和目标图像的宽高比相等，那么直接将原图像进行缩放操作，并返回缩放效果区域的矩形。
如果它们的宽高比不相等，就需要根据比例关系来计算缩放后的宽和高。

接下来，函数将原图像 src 缩放到 tmp 中，并将其大小调整到宽和高与目标图像大小最接近的尺寸。
如果 tmp 的宽不等于目标图像的宽，就将 tmp 复制到 dst 的对应位置中，记录缩放效果区域的矩形，并返回。
如果 tmp 的高不等于目标图像的高，同样的操作。

如果缩放后的 tmp 的宽高与目标图像的宽高都不相等，那么就说明出现了错误，直接打印错误信息即可。
*/
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
    
    /*这段代码使用OpenCV库中的cv::resize函数将输入图像src缩放到指定的大小，保存到tmp中。
    cv::resize函数的第一个参数是要缩放的源图像，第二个参数是缩放后的输出图像，第三个参数是输出图像的大小（cv::Size类型）。
    因此，这行代码的作用是将原始图像缩放到与目标大小最接近的大小，以便在下一步中进行居中裁剪。*/
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

/*这是一个颜色列表，包含了80种颜色，每个颜色由RGB三个分量表示。在可视化模型预测结果时，
常常使用这样的颜色列表将不同的目标实例用不同的颜色表示出来。
在这个代码中，将使用这个颜色列表为每个目标实例随机分配一种颜色。*/
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


/*这个函数用于将输入的RGB彩色图像进行归一化处理，将RGB三个通道的值按照一定的比例进行缩放，
使得每个像素点RGB三个通道的值之和为1，从而消除不同图像之间的亮度和颜色差异，提高模型的稳定性和鲁棒性。
*/

// 去光照
void normalizeRGB1(const cv::Mat& _src, cv::Mat& dst)
{   //将输入的图像转换为CV_32FC3格式，其中CV_32FC3是OpenCV中的一种数据类型，表示三通道浮点数图像

    //通过断言语句assert来判断输入的图像类型是否为CV_8UC3，即8位无符号整数类型的三通道图像，如果不是则程序停止运行并报错。
    assert( CV_8UC3 == _src.type() ); 

    //convertTo将输入图像_src转换为CV_32FC3类型的图像，并将转换后的结果保存在src中。由于后面进行图像归一化的操作
    //需要使用浮点数类型，因此将输入图像转换为CV_32FC3类型可以避免在后面进行类型转换时出现精度损失。
    cv::Mat src;
    _src.convertTo(src, CV_32FC3);

    /*遍历每个像素点的RGB三个通道的值，计算它们之和，并将三个通道的值按照它们的和进行归一化处理。。*/
    for(auto iter = src.begin<cv::Vec3f>(); iter != src.end<cv::Vec3f>(); ++iter) {
        //这行代码计算了一个归一化系数，用于将RGB颜色空间中的每个像素的三个通道值归一化为[0,1]范围内的值。
        //其中，denominator表示这个归一化系数，FLT_EPSILON是C++标准库中定义的一个浮点型的最小值常量，用于保证分母不为0。
        float denominator = 1.0f /((*iter)[0] + (*iter)[1] + (*iter)[2] + FLT_EPSILON);
        //归一化 RGB 图像
        (*iter)[0] = (*iter)[0] * denominator;
        (*iter)[1] = (*iter)[1] * denominator;
        (*iter)[2] = (*iter)[2] * denominator;
    }
    //将结果量化到[0,255]范围
    //NORM_MINMAX表示线性量化，CV_8UC3表示将图像转回
    cv::normalize(src, src, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(src, dst); //将浮点型矩阵src转换为8位无符号整型矩阵，输出矩阵dst的每个元素值是src中对应元素值的绝对值并转换为8位整型。
}



/*这是另一个归一化RGB图像的函数。与之前的函数相比，它先将输入图像拆分成三个通道，
然后对每个通道分别进行归一化。这个函数可以避免在迭代时多次访问同一像素，
因为它是基于矩阵计算的。最后，它将三个通道合并回原始图像，再将结果量化到[0, 255]的范围内，最终输出归一化后的图像。*/
void normalizeRGB(const cv::Mat &_src, cv::Mat &dst)  // 高效版本
{
    assert( CV_8UC3 == _src.type() );

    cv::Mat src;
    _src.convertTo(src, CV_32FC3);

    //拆分通道，将一个三通道的图像 src 拆分成了三个单通道的图像，并存储在一个名为 channels 的 std::vector<cv::Mat> 中
    std::vector<cv::Mat> channels;
    cv::split(src, channels);

    //各通道归一化
    cv::Mat denominator = 1.0 / (channels.at(0) + channels.at(1) + channels.at(2) + FLT_EPSILON);

    //对应元素点乘， 使用 mul 函数进行逐个像素的乘法运算，实现了将每个通道中的像素值都除以该像素在三个通道上像素值之和的操作。
    channels.at(0) = channels.at(0).mul(denominator); 
    channels.at(1) = channels.at(1).mul(denominator);
    channels.at(2) = channels.at(2).mul(denominator);

    //合并通道
    cv::merge(channels, src);

    // 将结果量化到[0,255]范围
    // NORM_MINMAX表示线性量化，CV_8UC3表示将图像转回
    cv::normalize(src, src, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(src, dst);
}

//这个函数是用于在图像中画出检测到的物体边框和标签。
//image:输入的图像，要在其上绘制包围框
//bboxes：目标物体的包围框信息，是一个BoxInfo类型的向量，每个元素代表一个目标物体的包围框，包含了其坐标、宽度、高度、类别等信息
//effect_roi：目标物体在图像上的感兴趣区域（region of interest），即物体可能出现的区域，用于剪裁图像和计算包围框的绝对坐标
//listener：tf::TransformListener类型的引用，用于获取物体相对于相机的坐标变换信息
void draw_bboxes(cv::Mat& image, const std::vector<BoxInfo>& bboxes, object_rect effect_roi, tf::TransformListener& listener)
{
    cv::Mat dst(image.size(), CV_32FC3);  //创建了一个名为dst的cv::Mat对象，其大小与输入的image相同
    cv::Mat imgray;
    //  cv::imshow("去光照", image);
    //去除光照
    normalizeRGB(image, dst);
    // cv::imshow("去光照", dst);
    
    cv::cvtColor(dst, imgray, cv::COLOR_BGR2GRAY);
    // cv::imshow("去光照后的灰度图像", imgray);
    //高斯模糊函数cv::GaussianBlur对灰度图像imgray进行模糊处理,模糊图像可以去除噪点和细节，减少干扰，有助于后续处理。
    cv::GaussianBlur(imgray, imgray, cv::Size(3, 3), 0.5, 0.5);
    // cv::imshow("平滑灰度图像", imgray);
    
    
    cv::Mat test_ding = imgray.clone();  //创造一个imgray的副本 test_ding
    std::vector<object_center> boxes_center;  //用于存储检测到的目标的中心坐标信息。

    //静态字符数组，包含了80种物体类别的名称。这些名称是用于目标检测算法输出的结果中标记检测到的物体所属的类别。
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

    for (size_t i = 0; i < bboxes.size(); i++) 
    {
        object_center box_center;
        const BoxInfo& bbox = bboxes[i];
        //给每一个bbox对象分配一个颜色，用于在图像上标注该bbox。颜色的选择是基于bbox所属的类别，通过查找color_list数组获取。
        cv::Scalar color = cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1], color_list[bbox.label][2]);
        
        /*计算了检测框的坐标位置，并将其缩放到效果图上的对应位置。
        首先通过计算检测框左上角和右下角在原始图像中的坐标位置，然后减去效果图的 ROI 坐标，
        最后乘以宽度和高度的比例系数，得到检测框在效果图中的坐标位置。
        lx 和 ly 表示检测框的左上角位置，rx 和 ry 表示检测框的右下角位置。*/
        float lx = (bbox.x1 - effect_roi.x) * width_ratio;
        float ly = (bbox.y1 - effect_roi.y) * height_ratio;
        float rx = (bbox.x2 - effect_roi.x) * width_ratio;
        float ry = (bbox.y2 - effect_roi.y) * height_ratio;

        /*****/
        //使用了OpenCV的cv::rectangle()函数，绘制了一个矩形框在图像上。
        cv::rectangle(image, cv::Rect(cv::Point((bbox.x1 - effect_roi.x) * width_ratio, (bbox.y1 - effect_roi.y) * height_ratio), 
                                      cv::Point((bbox.x2 - effect_roi.x) * width_ratio, (bbox.y2 - effect_roi.y) * height_ratio)), color);
        cv::rectangle(image, cv::Rect(cv::Point(lx, ly), 
                                      cv::Point(rx, ry)), color);
        /*****/        
        box_center.cx = lx + (rx - lx) / 2;
        box_center.cy = ly + (ry - ly) / 2;
        box_center.label_int = bbox.label;
        box_center.label = class_names[bbox.label];
        boxes_center.emplace_back(box_center);
        // 中心点画圆
        // cv::circle(image, cv::Point(box_center.cx, box_center.cy), 1, color);

        char text[256];
        /******/
        sprintf(text, "%s %.1f%%",  class_names[bbox.label], bbox.score * 100);
        /******/        
        int baseLine = 0;
        // getTextSize 来获取绘制字符串所需的矩形框大小
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        //计算标签文本的位置
        int x = (bbox.x1 - effect_roi.x) * width_ratio;
        int y = (bbox.y1 - effect_roi.y) * height_ratio - label_size.height - baseLine;
        /*用来检查标签文字是否超出了图像边界。如果y坐标小于0，则将其设为0，以确保标签位于图像内部。
        如果将x坐标与标签的宽度相加会导致标签超出图像边界，
        则将x坐标设置为足以容纳整个标签的最大值（即image.cols - label_size.width）。这样就可以确保标签完全在图像内部。*/
        if (y < 0) {
            y = 0;
        }   
        if (x + label_size.width > image.cols) {
             x = image.cols - label_size.width;
        }
        /******/
        /*这行代码在图像上绘制一个矩形，覆盖了文本所在的区域，使得文本能够更清晰地展示。cv::Rect()指定了矩形的位置和大小，
        cv::Point()指定了矩形的左上角坐标，cv::Size()指定了矩形的大小。
        最后一个参数为负数，表示矩形内部被填充的颜色与边框颜色相同。*/
        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),color, -1);
        //将标签和置信度分数的文本写在图像上
        cv::putText(image, text, cv::Point(x, y + label_size.height),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        /******/    
    }
    // cv::threshold(imgray, imgray, 150, 255, cv::THRESH_BINARY);
    // 去光照后的 100
    // > 90 变 255  cv::THRESH_BINARY_INV相反
    //将灰度图像 imgray 进行阈值处理，将像素值大于等于 90 的像素设置为 255，将其余像素设置为 0，从而产生一个二值图像。
    cv::threshold(imgray, imgray, 90, 255, cv::THRESH_BINARY);
    // cv::adaptiveThreshold(imgray, imgray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 3, 10);

    cv::imshow("去光照后的二值图像", imgray);
    // cv::imshow("种类", image);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    //查找二值化图像中的轮廓
    /*mgray：输入图像，为单通道灰度图像。
    contours：包含所有检测到的轮廓的向量。每个轮廓表示为一组点的向量。
    hierarchy：可选的输出向量，包含每个轮廓的层次信息。
    其中，RETR_TREE表示返回一个包含完整层次信息的轮廓树，CHAIN_APPROX_SIMPLE表示仅存储水平、垂直和对角线像素的端点。*/
    cv::findContours(imgray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    // std::cout << contours.size() << std::endl;
    for(int index = 0; index < contours.size(); ++index) 
    {
        bool flag = false;
        // if(contours[index][0].x >= 320 || contours[index][0].y >= 320 || contours[index][0].x <= 10 || contours[index][0].y <= 10) {
        //     continue;
        // }
        // 找到各个分割区域的最小矩形，rect包括矩形的中心点、W、L、angle
        cv::RotatedRect rect = cv::minAreaRect(contours[index]);
        int area = rect.size.height * rect.size.width;
        // 如果最小矩形框太小，直接滤过， 进入下一次循环
        if(area < 2000) continue;
        // 矩形的四个点：左下、左上、右上、右下
        cv::Mat boxPts;
        cv::boxPoints(rect, boxPts); //计算旋转矩形rect的四个顶点坐标，并将它们存储在Mat对象boxPts中
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
        for(int j = 0; j < boxes_center.size(); ++j) 
        {
            int x_err = cx - boxes_center[j].cx;
            int y_err = cy - boxes_center[j].cy;
            // 这儿加限制条件
            
            // if( (pow(x_err, 2) + pow(y_err, 2)) < 500 && (boxes_center[j].label == "banana") && (area < 40000 && area > 2000)) {
            //     flag = true;
            //     break;
            // }
            // 去光照   换新环境需要调 1000 banana 2000
            // if(boxes_center[j].label == "apple" || boxes_center[j].label == "sports ball") {
            //     std::cout << "------------------------------------------" << std::endl;
            //     std::cout << "(pow(x_err, 2) + pow(y_err, 2)) = " << (pow(x_err, 2) + pow(y_err, 2)) << std::endl;
            //     std::cout << "area = " << area << std::endl;
            // }
            
            // 换新环境需要调 1000 banana 2000
            if( (pow(x_err, 2) + pow(y_err, 2)) < 1000 && (boxes_center[j].label == "banana") && (area > 2000)) 
            {
                flag = true;
                // std::cout << "(pow(x_err, 2) + pow(y_err, 2)) = " << (pow(x_err, 2) + pow(y_err, 2)) << std::endl;
                // std::cout << "area = " << area << std::endl;
                break;
            }
        } 
        if(flag) 
        {
            cv::circle(image, cv::Point(cx, cy), 1, (0, 0, 255)); //在图像中心(cx, cy)画一个红点
            cv::Size2f size = rect.size;//获取矩形的长宽大小
            // cout << "width = " << size.width << ", height = " << size.height << endl;
            double angle = 0;

            //根据矩形框的长和宽确定物体的朝向角度，并在图像上标注物体的方向。
            /*注：矩形框的旋转角度：
            在图像中，如果矩形不是水平或垂直的，那么它就会有一个旋转角度，
            表示矩形围绕其质心旋转的角度。这个角度可以用矩形的 angle 属性来表示，
            通常是介于 -90 度到 0 度之间的负值，其中 -90 度表示矩形逆时针旋转了 90 度，0 度表示矩形没有旋转。
            */
            if(size.width < size.height)  
            {   //如果物体的宽度小于高度，那么物体就是“竖着”的，朝向角度就等于矩形框的旋转角度；
                angle = rect.angle;
                //将文字标注添加到图像上，以指示物体的朝向。
                std::string text = "left" + std::to_string(angle);
                //cv::Point(cx + 10, cy - 10)：标注的位置，以图像上的坐标表示。
                //在这里，cx和cy是矩形的中心点，这个点是矩形的边界框的左上角，因此将标注放在其上方稍微偏移10个像素的位置。
                cv::putText(image, text, cv::Point(cx + 10, cy - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
            } else {   
                angle = rect.angle + 90; //否则，物体就是“横着”的，朝向角度就等于矩形框的旋转角度加上90度。
                std::string text = "right" + std::to_string(angle);
                cv::putText(image, text, cv::Point(cx + 10, cy - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
            }
            // std::cout << "cx = " << cx << ", cy = " << cy << ", angle = " << angle << std::endl;
            /* draw_box 是一个矩形框的四个顶点坐标构成的向量，为了使用 OpenCV 的绘制轮廓函数 cv::drawContours()，
               需要将其转换成 std::vector<std::vector<cv::Point>> 类型的变量。*/
            std::vector<std::vector<cv::Point>> input(1);
            input[0] = draw_box;
            cv::drawContours(image, input, 0, cv::Scalar(0,0,255), 1); //将轮廓绘制到原始图像image上，绘制的颜色为红色，线宽为1。
            //创建了一个3x1的double类型的矩阵pixel_frame。
            //物体的中心点在相机坐标系下的坐标(cxZ_c, cyZ_c, Z_c)，其中Z_c为相机到物体的距离（也就是深度），
            //这个距离已经通过深度图计算得出。
            cv:: Mat pixel_frame = (cv::Mat_<double>(3, 1) << cx * Z_c, cy * Z_c, Z_c);
            //调用函数computeRealPosition()，将pixel_frame转换到真实世界坐标系中，
            //得到物体在真实世界中的坐标，并将结果保存在worldFrame中。
            //computeRealPosition函数，该函数将使用相机内参矩阵和畸变系数对pixel_frame进行矫正，然后将其转换为世界坐标系下的坐标。
            cv:: Mat worldFrame = computeRealPosition(pixel_frame, listener);
            
            //通过tf库中的createQuaternionMsgFromYaw函数将物体的朝向角度转化为四元数表示的姿态信息，
            //存储在geometry_msgs::Quaternion quat中。
            geometry_msgs::Quaternion quat; //代表物体的姿态。
            // std::cout << angle << std::endl;
            /*createQuaternionMsgFromYaw 函数，它可以将一个yaw角转换为四元数。yaw角是绕着z轴旋转的角度，
            也就是矩形框的朝向角度。通过这个四元数，可以方便地将矩形框的朝向角度传递给机器人，从而实现对物体的追踪和抓取。*/
            quat = tf::createQuaternionMsgFromYaw(angle);
            
            geometry_msgs::Pose goalPose;//goalPose代表着检测到的物体在机器人坐标系下的位置和姿态
            goalPose.position.x = worldFrame.at<double>(0, 0);
            goalPose.position.y = worldFrame.at<double>(0, 1);
            goalPose.position.z = worldFrame.at<double>(0, 2);
            goalPose.orientation.x = quat.x; //goalPose.orientation.x 是一个四元数（quaternion）中的 x 分量。
            goalPose.orientation.y = quat.y;
            goalPose.orientation.z = quat.z;
            goalPose.orientation.w = quat.w;
            //在控制台输出目标物体在世界坐标系中的位置
            std::cout << "px = " << goalPose.position.x << ",  py = " << goalPose.position.y << ",  pz = " << goalPose.position.z << std::endl;
            objectPose = goalPose;
            objectPose.position.z = 0.040; // banana 0.04 ？？
            ang = (float)angle;
            worldPosition.publish(goalPose);//将goalPose消息发布到worldPosition话题上
            break;
        }
    }
    // return imgray;
    cv::waitKey(1);
}


void receive_msg_call_back(const boost::shared_ptr<sensor_msgs::Image const>& msg, NanoDet& detector, tf::TransformListener& listener)
{   
    auto result_start = std::chrono::steady_clock::now();
    int min_width = 320;
	int min_height = 320;

    //将ROS消息中的图像数据(msg)转换为OpenCV的图像格式。
    //sensor_msgs::image_encodings::TYPE_8UC3指定了将图像编码为8位无符号整数的3通道图像
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    //cv_bridge::CvImagePtr类型的指针cv_ptr，其中的image成员变量就是转换后的图像数据。
    cv::Mat img_ = cv_ptr->image;
    // cv::Mat img_ = cv::Mat(cv::Size(320, 320), CV_8UC3);
    // cv::Mat img_;
    
    /*摄像头发布的图像默认是RGB格式的，而在OpenCV中，图像是以BGR格式存储的。
    因此，在使用OpenCV处理ROS图像消息之前，需要将其转换为OpenCV所需的BGR格式。*/
    cv::cvtColor(img_, img_, cv::COLOR_RGB2BGR);//将 img_ 从RGB格式转换成了BGR格式
    // cv::imshow("原图", img_);
    object_rect effect_roi;
    cv::Mat resized_img;
    // cv::resize(img_,img_,cv::Size(min_width, min_height));

    //将输入图像调整为指定的大小
    resize_uniform(img_, resized_img, cv::Size(320, 320), effect_roi);
    //detect 方法首先将图像送入模型中进行推理得到预测框和类别，然后再进行 NMS，最后返回去掉重复预测框后的检测结果。
    auto results = detector.detect(resized_img, 0.25, 0.5);
    // cv::waitKey(1);
    // std::cout << "detect time = " << result_time << std::endl;
    // std::cout << "results size = " << results.size() << std::endl;
    // std::cout << img_ << std::endl;
    
    draw_bboxes(img_, results, effect_roi, listener);
    
    
    if (tf_publish) 
    {
        // 设置真实香蕉的tf
        realBananaTF.setOrigin( tf::Vector3(objectPose.position.x, objectPose.position.y, objectPose.position.z) ); 
        /*将ang转换为弧度制并加上1.5708（即90度）得到BananaAngle，
        这是因为在ROS中，x轴方向为前，y轴方向为左，z轴方向为上，而香蕉的默认朝向是在y-z平面上，因此需要将其旋转90度。*/
        float BananaAngle = (ang / 180 * 3.14159 * (-1));
        BananaAngle += 1.5708;
        tf::Quaternion q;
        //设置旋转矩阵，其中的参数 -3.141, 0.0, BananaAngle 表示绕x轴、y轴、z轴旋转的角度，
        //这里 -3.141 表示绕x轴旋转180度，即将物体朝向翻转180度，这也意味着原来默认的朝向是垂直于y-z平面的。
        q.setRPY( -3.141, 0.0, BananaAngle); 
        realBananaTF.setRotation(q);

        // 设置跟踪香蕉的tf
        trackBananaTF.setOrigin( tf::Vector3(objectPose.position.x, objectPose.position.y, objectPose.position.z + 0.2789) ); 
        trackBananaTF.setRotation(q);

        // 发布真实香蕉的tf
        realBanana->sendTransform(tf::StampedTransform(realBananaTF, ros::Time::now(), "world", "banana"));
        
        // 发布跟踪香蕉的tf
        trackBanana->sendTransform(tf::StampedTransform(trackBananaTF, ros::Time::now(), "world", "track_banana"));
        

    }
    auto result_end = std::chrono::steady_clock::now();
    auto result_time = std::chrono::duration<double, std::milli>(result_end - result_start).count();
    std::cout << "detect time = " << result_time << std::endl;
    //将OpenCV中的图像img_转换为ROS中的图像消息类型sensor_msgs/Image，并将其发布到ROS主题process_image上。
    sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
    process_image.publish(*image);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle n;
    tf::TransformListener listener;
    realBanana = new tf::TransformBroadcaster;
    trackBanana = new tf::TransformBroadcaster;
    signal(SIGINT, signalHandle);
    auto start = std::chrono::steady_clock::now();
    auto detector = NanoDet("/home/huo/Downloads/ur3_ws/src/arm/image_process/model/nanodet_m.xml");
    // auto detector = NanoDet("/home/huo/Downloads/ur3_ws/src/arm/image_process/model/nanodet_m.xml");

    auto end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration<double>(end - start).count();
    std::cout << "*** load model's time(s):\t" << time << std::endl; 
     
    process_image = n.advertise<sensor_msgs::Image>("image", 500);  //发布image的话题
    
    worldPosition = n.advertise<geometry_msgs::Pose>("worldPose", 500);  //发布worldPose的话题  boost::ref(detector)
    boost::function<void(const boost::shared_ptr<sensor_msgs::Image const>&)> callback = boost::bind(receive_msg_call_back, _1, boost::ref(detector), boost::ref(listener));
    
    original_image = n.subscribe("/camera/rgb/image_raw", 500, callback);    //订阅原图像话题，/camera/rgb/image_raw
    // original_image = n.subscribe("/camera/rgb/image_raw", 500, receive_msg_call_back);
    ros::ServiceServer service = n.advertiseService("getObjectPose", responsePose);

    ROS_INFO("ready!");
    ros::spin();
    delete realBanana;
    delete trackBanana;
    return 0;

}