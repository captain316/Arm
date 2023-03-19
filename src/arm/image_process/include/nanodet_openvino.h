//
// Create by RangiLyu
// 2021 / 1 / 12
//

#ifndef _NANODET_OPENVINO_H_
#define _NANODET_OPENVINO_H_

#include <string>
#include <opencv2/core.hpp>
#include <inference_engine.hpp>

/*
这个头文件，定义了一个名为NanoDet的类。该类实现了基于OpenVINO的目标检测算法：

*/
typedef struct HeadInfo
{
    std::string cls_layer;
    std::string dis_layer;
    int stride;
} HeadInfo;

typedef struct BoxInfo 
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int label;
} BoxInfo;
 



class NanoDet
{
public:
    NanoDet(const char* param);

    ~NanoDet();
    
    InferenceEngine::ExecutableNetwork network_;
    InferenceEngine::InferRequest infer_request_;
    // static bool hasGPU;
    
    std::vector<HeadInfo> heads_info_{  //包含用于目标检测的不同头信息的向量。
        // cls_pred|dis_pred|stride
            {"792", "795",    8},
            {"814", "817",   16},
            {"836", "839",   32},
    };
    //使用当前对象的目标检测算法，检测给定图像中的物体，并返回包含物体位置、得分和标签信息的向量。
    std::vector<BoxInfo> detect(cv::Mat image, float score_threshold, float nms_threshold);

private:
    //将图像预处理为神经网络所需的Blob格式。
    void preprocess(cv::Mat& image, InferenceEngine::Blob::Ptr& blob); 
    //对推理结果进行解码，并返回包含物体位置、得分和标签信息的向量。
    void decode_infer(const float*& cls_pred, const float*& dis_pred, int stride, float threshold, std::vector<std::vector<BoxInfo>>& results);
    //将位移预测（dis_pred）转换为Bbox信息。
    BoxInfo disPred2Bbox(const float*& dfl_det, int label, float score, int x, int y, int stride);
    //执行非最大抑制（NMS）算法以过滤检测结果。
    static void nms(std::vector<BoxInfo>& result, float nms_threshold);
    std::string input_name_; //输入Blob的名称。
    int input_size_ = 320;   //输入图像的大小。
    int num_class_ = 80;     //模型所能检测的物体类别数。
    int reg_max_ = 7;        //Bbox坐标预测的最大偏移量。

};


#endif //_NANODE_TOPENVINO_H_