/**
* This file is part of ORB-SLAM2.
* Modified for OA-SLAM ROS 2 Node
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>
#include <vector>
#include <string>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>

// OpenCV
#include <opencv2/core/core.hpp>

// OA-SLAM headers
#include "System.h"
#include "ImageDetections.h" 

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(const string& vocab_path, const string& settings_path,
                 const string &topic_rgb, const string &topic_depth, 
                 std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector,
                 ORB_SLAM2::System::eSensor sensor,
                 ORB_SLAM2::enumRelocalizationMode reloc_mode); // Thêm tham số reloc_mode

    ~ImageGrabber();

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                  const sensor_msgs::msg::Image::ConstSharedPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

private:
    std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> mpDetector;
    ORB_SLAM2::enumRelocalizationMode mRelocMode;

    // Message filters
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, 
                                                             sensor_msgs::msg::Image> sync_policy;
    typedef message_filters::Synchronizer<sync_policy> Synchronizer;
    shared_ptr<Synchronizer> sync_;
};

ImageGrabber::ImageGrabber(const string& vocab_path, const string& settings_path,
                           const string &topic_rgb, const string &topic_depth,
                           std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector,
                           ORB_SLAM2::System::eSensor sensor,
                           ORB_SLAM2::enumRelocalizationMode reloc_mode)
    : rclcpp::Node("oa_slam_rgbd"),
      mpDetector(detector),
      mRelocMode(reloc_mode)
{
    
    // Create SLAM system. 
    // Lưu ý: OA-SLAM System constructor có thể khác nhau tùy phiên bản sửa đổi.
    // Giả sử constructor chuẩn: (Vocab, Settings, Sensor, UseViewer, ...)
    mpSLAM = new ORB_SLAM2::System(vocab_path, settings_path, sensor, true);

    // Subscribers
    rgb_sub_.subscribe(this, topic_rgb);
    depth_sub_.subscribe(this, topic_depth);

    // Nếu System cần cấu hình Reloc Mode ngay khi khởi tạo, bạn có thể cần set ở đây
    // Ví dụ: mpSLAM->SetRelocalizationMode(reloc_mode); (Nếu hàm này tồn tại)

    // Create synchronizer
    // Queue size 10
    sync_ = std::make_shared<Synchronizer>(sync_policy(10), rgb_sub_, depth_sub_);
    
    // Đăng ký callback
    sync_->registerCallback(bind(&ImageGrabber::GrabRGBD, this, placeholders::_1, placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "OA-SLAM RGBD node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to RGB: %s", topic_rgb.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to Depth: %s", topic_depth.c_str());
}

ImageGrabber::~ImageGrabber()
{
    if (mpSLAM)
    {
        mpSLAM->Shutdown();
        // Save trajectory if needed
        // mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        delete mpSLAM;
    }
}

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                            const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
{
    // Copy the ROS2 image message to cv::Mat for RGB
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ROS2 image message to cv::Mat for Depth
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Get object detections
    std::vector<ORB_SLAM2::Detection::Ptr> detections;
    if (mpDetector) {
        // Detect function might accept filename or Mat. 
        // OA-SLAM usually detects on cv::Mat in live mode.
        detections = mpDetector->detect(cv_ptrRGB->image);
    }

    // Convert timestamp
    double timestamp = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;

    // Pass to SLAM with detections
    // [FIX] Truyền thêm detections và force_relocalize=false
    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, timestamp, detections, false);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // Initialize ROS2

    // Check arguments
    // ./ros_oa_slam_rgbd vocab settings rgb_topic depth_topic detect_file ignore_file reloc_mode output
    if(argc != 9)
    {
        cerr << endl << "Usage:\n"
                        " ./ros_oa_slam_rgbd path_to_vocabulary path_to_settings topic_rgb topic_depth path_to_detections path_to_ignore_categories relocalization_mode output_folder\n";
        return 1;
    }

    // Parse arguments
    string vocab_path = argv[1];
    string settings_path = argv[2];
    string topic_rgb = argv[3];
    string topic_depth = argv[4];
    string detections_file = argv[5];
    string categories_to_ignore_file = argv[6];
    string reloc_mode_str = argv[7];
    string output_folder = argv[8]; // Chưa dùng trong code này, nhưng giữ lại để khớp argument

    // Load categories to ignore
    std::ifstream fin(categories_to_ignore_file);
    vector<int> classes_to_ignore;
    if (!fin.is_open()) {
        std::cout << "Warning !! Failed to open the file with ignore classes. No class will be ignore.\n";
    } else {
        int cat;
        while (fin >> cat) {
            std::cout << "Ignore category: " << cat << "\n";
            classes_to_ignore.push_back(cat);
        }
    }

    // Load object detections
    auto extension = get_file_extension(detections_file);
    std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector = nullptr;
    
    if (extension == "onnx") { // load network
        detector = std::make_shared<ORB_SLAM2::ObjectDetector>(detections_file, classes_to_ignore);
        std::cout << "Object detector loaded from ONNX file: " << detections_file << "\n";
    } else if (extension == "json") { // load from external detections file
        // JSON thường dùng cho offline dataset, nhưng vẫn giữ logic này
        detector = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file, classes_to_ignore);
    } else {
        std::cout << "Invalid detection file. It should be .json or .onnx\n";
    }

    // Relocalization mode
    ORB_SLAM2::enumRelocalizationMode relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    if (reloc_mode_str == string("points"))
        relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    else if (reloc_mode_str == std::string("objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
    else if (reloc_mode_str == std::string("points+objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
    else {
        std::cerr << "Error: Invalid parameter for relocalization mode.\n";
        return 1;
    }

    // Initialize Node
    auto node = make_shared<ImageGrabber>(vocab_path, settings_path, 
                                          topic_rgb, topic_depth, 
                                          detector, 
                                          ORB_SLAM2::System::RGBD, 
                                          relocalization_mode);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}