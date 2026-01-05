/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#include "ImageDetections.h"

#include <experimental/filesystem>
#include <unordered_set>
#include <algorithm> // for max, min

#define person_class_id 0

namespace fs = std::experimental::filesystem;


namespace ORB_SLAM2
{

// ================= HELPER FUNCTION FOR MASK GENERATION =================
// Generates binary mask from proto masks and coefficients
static cv::Mat generateMask(const cv::Mat& proto_masks, const std::vector<float>& coeffs) {
    // proto_masks shape usually: [1, 32, mask_h, mask_w] or [32, mask_h, mask_w]
    
    int dims = proto_masks.dims;
    int mask_h = proto_masks.size[dims - 2];
    int mask_w = proto_masks.size[dims - 1];
    int num_protos = proto_masks.size[dims - 3]; // Usually 32
    int area = mask_h * mask_w;

    if (coeffs.size() != num_protos) {
        std::cerr << "[Error] Mask coeffs size mismatch. Expected " << num_protos << ", got " << coeffs.size() << std::endl;
        return cv::Mat();
    }

    // Prepare mask: Dynamic size
    cv::Mat mask = cv::Mat::zeros(mask_h, mask_w, CV_32F);
    float* mask_ptr = (float*)mask.data;
    const float* proto_data = (const float*)proto_masks.data;
    
    // Linear combination: mask = sum(coeff[i] * proto[i])
    for (int i = 0; i < num_protos; i++) {
        float coeff = coeffs[i];
        for (int j = 0; j < area; j++) {
            mask_ptr[j] += coeff * proto_data[i * area + j];
        }
    }
    
    // Sigmoid activation
    for (int i = 0; i < area; i++) {
        mask_ptr[i] = 1.0f / (1.0f + std::exp(-mask_ptr[i]));
    }
    
    return mask; // Returns mask (0.0 to 1.0) with size corresponding to model output (e.g., 80x80 for 320 input)
}


std::ostream& operator <<(std::ostream& os, const Detection& det)
{
    os << "Detection:  cat = " << det.category_id << "  score = "
       << det.score << "  bbox = " << det.bbox.transpose();
    return os;
}


DetectionsFromFile::DetectionsFromFile(const std::string& filename, const std::vector<int>& cats_to_ignore)
    : ImageDetectionsManager()
{
    std::unordered_set<unsigned int> to_ignore(cats_to_ignore.begin(), cats_to_ignore.end());
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cerr << "Warning failed to open file: " << filename << std::endl;
        return ;
    }
    fin >> data_;

    for (auto& frame : data_)
    {
        std::string name = frame["file_name"].get<std::string>();
        name = fs::path(name).filename();
        frame_names_.push_back(name);

        std::vector<Detection::Ptr> detections;
        for (auto& d : frame["detections"])
        {
            double score = d["detection_score"].get<double>();
            unsigned int cat = d["category_id"].get<unsigned int>();
            if (to_ignore.find(cat) != to_ignore.end())
                continue;
            auto bb = d["bbox"];
            Eigen::Vector4d bbox(bb[0], bb[1], bb[2], bb[3]);
            detections.push_back(std::shared_ptr<Detection>(new Detection(cat, score, bbox)));
        }
        detections_[name] = detections;
    }
}


std::vector<Detection::Ptr> DetectionsFromFile::detect(const std::string& name) const {
    std::string basename = fs::path(name).filename();

    if (detections_.find(basename) == detections_.end())
        return {};
    return detections_.at(basename);
}
std::vector<Detection::Ptr> DetectionsFromFile::detect(unsigned int idx) const {
    if (idx < 0 || idx >= frame_names_.size()) {
        std::cerr << "Warning invalid index: " << idx << std::endl;
        return {};
    }
    return this->detect(frame_names_[idx]);
}


#ifdef USE_DNN

ObjectDetector::ObjectDetector(const std::string& model, const std::vector<int>& cats_to_ignore) 
    : network_(std::make_unique<cv::dnn::Net>()), ignored_cats_(cats_to_ignore.begin(), cats_to_ignore.end()), ImageDetectionsManager()
{
    if (model.substr(model.size()-4) == "onnx")
        *network_ =  cv::dnn::readNet(model);
    else
        *network_ =  cv::dnn::readNetFromDarknet(model + ".cfg", model + ".weights");
    
    // Try to use CUDA if available
    network_->setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    network_->setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

std::vector<Detection::Ptr> ObjectDetector::detect(cv::Mat img) const
{
    // Settings for YOLOv8
    const int INPUT_WIDTH = 640; 
    const int INPUT_HEIGHT = 640;
    const float SCORE_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45; 

    int original_width = img.cols;
    int original_height = img.rows;

    // Preprocessing
    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1./255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    
    network_->setInput(blob);

    // Forward pass - Requesting all outputs
    std::vector<cv::Mat> outputs;
    // For YOLOv8-seg, we usually have 2 outputs: detections and proto-masks
    // If names are not known, getUnconnectedOutLayersNames() is safest
    std::vector<std::string> out_names = network_->getUnconnectedOutLayersNames();
    
    network_->forward(outputs, out_names);

    // Expecting at least 2 outputs for Segmentation model
    if (outputs.size() < 2) {
        std::cerr << "Error: Expected at least 2 outputs for YOLOv8 Segmentation model. Got " << outputs.size() << std::endl;
        // Fallback or empty return if not matching expectation
        // return {}; 
        // Proceeding with caution, logic below assumes index 0 is dets and 1 is masks.
    }

    // Output 0: Detections [Batch, 4 + NumClasses + 32, NumAnchors]
    // Output 1: Proto-masks [Batch, 32, 160, 160]
    
    cv::Mat output_dets = outputs[0];
    cv::Mat output_masks_proto = outputs[1];

    // Reshape Logic
    // Reshape output_dets to [NumAnchors, NumValues]
    int num_detections = output_dets.size[2];
    int num_values = output_dets.size[1]; // 4 (box) + Classes + 32 (coeffs)
    
    // Transpose logic: (1, Channels, Anchors) -> (Anchors, Channels)
    cv::Mat dets_reshaped(num_detections, num_values, CV_32F);
    float* src = (float*)output_dets.data;
    float* dst = (float*)dets_reshaped.data;
    for (int i = 0; i < num_detections; i++) {
        for (int j = 0; j < num_values; j++) {
            dst[i * num_values + j] = src[j * num_detections + i];
        }
    }

    // Reshape proto masks if needed
    // Proto mask should be [Batch, 32, 160, 160]
    // We can access it directly as float array

    float x_factor = (float)original_width / INPUT_WIDTH;
    float y_factor = (float)original_height / INPUT_HEIGHT;
    float* data = (float*)dets_reshaped.data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> mask_coeffs_list;

    // Assuming COCO 80 classes by default, but calculating dynamically
    // format: [cx, cy, w, h, class_score_0, ..., class_score_N, mask_coeff_0, ... mask_coeff_31]
    int num_mask_coeffs = 32;
    int num_classes = num_values - 4 - num_mask_coeffs; 
    
    // Safety check
    if (num_classes <= 0) {
         // Fallback for detection only models if someone passes wrong model
         num_classes = num_values - 4; // standard YOLO
         num_mask_coeffs = 0; 
    }

    for (int i = 0; i < num_detections; ++i) {
        
        float* class_scores = data + 4;
        float max_score = 0.0f;
        int best_class = -1;

        // Find best class
        for (int j = 0; j < num_classes; j++) {
            if (class_scores[j] > max_score) {
                max_score = class_scores[j];
                best_class = j;
            }
        }

        if (max_score > SCORE_THRESHOLD) {
             if (ignored_cats_.count(best_class) == 0) {
                 
                float cx = data[0];
                float cy = data[1];
                float w = data[2];
                float h = data[3];

                int left = std::max(0, int((cx - 0.5f * w) * x_factor));
                int top = std::max(0, int((cy - 0.5f * h) * y_factor));
                int width = std::max(1, int(w * x_factor));
                int height = std::max(1, int(h * y_factor));
                
                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(max_score);
                class_ids.push_back(best_class);

                // Extract Mask Coeffs if available
                if (num_mask_coeffs > 0) {
                    std::vector<float> coeffs(num_mask_coeffs);
                    float* coeff_ptr = data + 4 + num_classes;
                    for (int k = 0; k < num_mask_coeffs; k++) {
                        coeffs[k] = coeff_ptr[k];
                    }
                    mask_coeffs_list.push_back(coeffs);
                }
             }
        }
        data += num_values;
    }

    // NMS
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

    std::vector<Detection::Ptr> detections;
    
    for (int idx : nms_result) {
        cv::Rect box = boxes[idx];
        
        // Generate Mask
        cv::Mat final_mask;
        if (num_mask_coeffs > 0 && !mask_coeffs_list.empty()) {
            cv::Mat raw_mask = generateMask(output_masks_proto, mask_coeffs_list[idx]);
            if (!raw_mask.empty()) {
                // Resize mask to original image size
                // Note: The mask from generateMask is 160x160.
                cv::Mat resized_mask;
                cv::resize(raw_mask, resized_mask, cv::Size(original_width, original_height));
                
                // Threshold to make it binary or keep as float probability? 
                // Let's keep it as thresholded binary for SLAM usage (CV_8U)
                // resized_mask = (resized_mask > 0.5); 
                // However, returning float mask might be more versatile. 
                // For now, let's threshold it to save memory/processing in Tracker if strictly binary is needed.
                // Or store as CV_8U 0 or 255.
                
                cv::threshold(resized_mask, final_mask, 0.5, 255, cv::THRESH_BINARY);
                final_mask.convertTo(final_mask, CV_8U);
                
                if (final_mask.empty()) {
                    std::cout << "[DEBUG] Mask EMPTY for class " << class_ids[idx] << std::endl;
                } else {
                    int white_pixels = cv::countNonZero(final_mask);
                    std::cout << "[DEBUG] Detection: Class=" << class_ids[idx] 
                              << ", Score=" << confidences[idx]
                              << ", Mask Size=" << final_mask.cols << "x" << final_mask.rows
                              << ", White Pixels=" << white_pixels << std::endl;
                    
                    // (Tùy chọn) Lưu mask ra ảnh để xem bằng mắt thường
                    // std::string debug_name = "debug_mask_" + std::to_string(class_ids[idx]) + ".png";
                    // cv::imwrite(debug_name, final_mask);
                }
                // Optional: Crop mask to bbox only to save space?
                // For global consistency, keeping full image size is easier for mapping (x,y) -> mask(x,y).
            }
        }

        // Convert cv::Rect to Eigen::Vector4d (x1, y1, x2, y2)
        Eigen::Vector4d bbox(box.x, box.y, box.x + box.width, box.y + box.height);

        // Create Detection object
        detections.push_back(std::shared_ptr<Detection>(
            new Detection(class_ids[idx], confidences[idx], bbox, final_mask)
        ));
    }

    return detections;
}
#endif

}
