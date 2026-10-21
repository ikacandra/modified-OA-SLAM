/**
* This file is part of OA-SLAM.
*
* TensorRT backend for the YOLOv8-seg detector (.engine, e.g. FP16 built with trtexec).
* Same interface and post-processing (decodeYoloSeg) as ObjectDetector; only the
* forward pass runs through the nvinfer API. Written against TensorRT 8.5.
* Compiled only when CMake finds TensorRT (USE_TENSORRT).
*/

#ifndef TRT_DETECTOR_H
#define TRT_DETECTOR_H

#include "ImageDetections.h"

#ifdef USE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace ORB_SLAM2
{

#ifdef USE_TENSORRT
class TRTDetector : public ImageDetectionsManager
{
public:
    TRTDetector(const std::string& engine_file, const std::vector<int>& cats_to_ignore);
    ~TRTDetector();

    std::vector<Detection::Ptr> detect(cv::Mat img) const;

    std::vector<Detection::Ptr> detect(const std::string& name) const {
        std::cerr << "This function is not available. You should pass an image as input\n";
        return {};
    }
    std::vector<Detection::Ptr> detect(unsigned int idx) const {
        std::cerr << "This function is not available. You should pass an image as input\n";
        return {};
    }

private:
    class Logger : public nvinfer1::ILogger
    {
        void log(Severity severity, const char* msg) noexcept override;
    };

    Logger logger_;
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_ = nullptr;

    std::vector<void*> bindings_;   // device buffers, indexed by binding index
    int input_idx_ = -1;
    int det_idx_ = -1;              // detections: [1, 4+nc+32, N]
    int proto_idx_ = -1;            // prototype masks: [1, 32, mask_h, mask_w]
    std::vector<int> det_shape_;
    std::vector<int> proto_shape_;
    size_t input_count_ = 0;

    std::unordered_set<int> ignored_cats_;
    int input_width_ = 0;
    int input_height_ = 0;
};
#endif

}

#endif // TRT_DETECTOR_H
