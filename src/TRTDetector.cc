/**
* This file is part of OA-SLAM.
*
* TensorRT backend for the YOLOv8-seg detector. See include/TRTDetector.h.
*/

#include "TRTDetector.h"

#ifdef USE_TENSORRT

#include <fstream>
#include <iterator>
#include <stdexcept>

namespace ORB_SLAM2
{

namespace
{
void checkCuda(cudaError_t err, const char* what)
{
    if (err != cudaSuccess)
        throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
}

std::vector<int> toShape(const nvinfer1::Dims& d)
{
    return std::vector<int>(d.d, d.d + d.nbDims);
}

size_t volume(const std::vector<int>& shape)
{
    size_t v = 1;
    for (int s : shape)
        v *= static_cast<size_t>(s);
    return v;
}
}

void TRTDetector::Logger::log(Severity severity, const char* msg) noexcept
{
    if (severity <= Severity::kWARNING)
        std::cerr << "[TensorRT] " << msg << '\n';
}

TRTDetector::TRTDetector(const std::string& engine_file, const std::vector<int>& cats_to_ignore)
    : ImageDetectionsManager(), ignored_cats_(cats_to_ignore.begin(), cats_to_ignore.end())
{
    std::ifstream file(engine_file, std::ios::binary);
    if (!file)
        throw std::runtime_error("Cannot open TensorRT engine: " + engine_file);
    std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    runtime_ = nvinfer1::createInferRuntime(logger_);
    engine_ = runtime_ ? runtime_->deserializeCudaEngine(blob.data(), blob.size()) : nullptr;
    context_ = engine_ ? engine_->createExecutionContext() : nullptr;
    if (!context_)
        throw std::runtime_error("Failed to deserialize TensorRT engine: " + engine_file);

    // Bindings are identified by their rank, not by index: the engine may order
    // the outputs differently from the ONNX file.
    const int nb = engine_->getNbBindings();
    bindings_.assign(nb, nullptr);
    std::cout << "TensorRT engine bindings:\n";
    for (int i = 0; i < nb; ++i)
    {
        std::vector<int> shape = toShape(engine_->getBindingDimensions(i));
        std::cout << "  " << i << ' ' << engine_->getBindingName(i)
                  << (engine_->bindingIsInput(i) ? " (input) " : " (output) ");
        for (int s : shape)
            std::cout << s << ' ';
        std::cout << '\n';

        if (engine_->getBindingDataType(i) != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("TRTDetector expects FP32 bindings (FP16 internals are fine)");

        if (engine_->bindingIsInput(i))
        {
            input_idx_ = i;
            input_height_ = shape[2];
            input_width_ = shape[3];
            input_count_ = volume(shape);
        }
        else if (shape.size() == 3)
        {
            det_idx_ = i;
            det_shape_ = shape;
        }
        else if (shape.size() == 4)
        {
            proto_idx_ = i;
            proto_shape_ = shape;
        }
        checkCuda(cudaMalloc(&bindings_[i], volume(shape) * sizeof(float)), "cudaMalloc");
    }
    if (input_idx_ < 0 || det_idx_ < 0 || proto_idx_ < 0)
        throw std::runtime_error("Unexpected bindings: need 1 input, a 3-D detection output and a 4-D proto output");

    checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

    // Warm-up: the first enqueue initializes CUDA/cuDNN kernels (~1 s); do it here so that
    // it is counted in the initialization time instead of the first tracked frame.
    for (int i = 0; i < 5; ++i)
        if (!context_->enqueueV2(bindings_.data(), stream_, nullptr))
            throw std::runtime_error("TensorRT enqueueV2 failed during warm-up");
    checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
}

TRTDetector::~TRTDetector()
{
    for (void* buf : bindings_)
        if (buf)
            cudaFree(buf);
    if (stream_)
        cudaStreamDestroy(stream_);
    delete context_;
    delete engine_;
    delete runtime_;
}

std::vector<Detection::Ptr> TRTDetector::detect(cv::Mat img) const
{
    // Preprocessing (identical to ObjectDetector)
    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1./255.0, cv::Size(input_width_, input_height_), cv::Scalar(), true, false);

    // Forward pass through TensorRT
    cv::Mat output_dets(static_cast<int>(det_shape_.size()), det_shape_.data(), CV_32F);
    cv::Mat output_masks_proto(static_cast<int>(proto_shape_.size()), proto_shape_.data(), CV_32F);
    checkCuda(cudaMemcpyAsync(bindings_[input_idx_], blob.ptr<float>(), input_count_ * sizeof(float),
                              cudaMemcpyHostToDevice, stream_), "cudaMemcpy H2D");
    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr))
        throw std::runtime_error("TensorRT enqueueV2 failed");
    checkCuda(cudaMemcpyAsync(output_dets.ptr<float>(), bindings_[det_idx_], output_dets.total() * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_), "cudaMemcpy D2H detections");
    checkCuda(cudaMemcpyAsync(output_masks_proto.ptr<float>(), bindings_[proto_idx_], output_masks_proto.total() * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_), "cudaMemcpy D2H protos");
    checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

    return decodeYoloSeg(output_dets, output_masks_proto, img.cols, img.rows,
                         input_width_, input_height_, ignored_cats_);
}

}

#endif // USE_TENSORRT
