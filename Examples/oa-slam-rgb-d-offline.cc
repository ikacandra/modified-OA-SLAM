#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdlib.h>     /* srand, rand */
#include <csignal>
#include <iomanip>
#include <limits>
#include <execinfo.h>
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include <ImageDetections.h>
#include <TRTDetector.h>
#include <System.h>
#include "Osmap.h"
#include <nlohmann/json.hpp>
#include <experimental/filesystem>
#include "Utils.h"

using json = nlohmann::json;
namespace fs = std::experimental::filesystem; // [FIX 1] Thêm namespace fs

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

// Epoch seconds (same markers as statistics.txt of ORB-SLAM3 / YDM-SLAM)
static double epochNow()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Print a backtrace on crash so unattended benchmark runs leave a trace in their log.
static void crashHandler(int sig)
{
    void* frames[64];
    int n = backtrace(frames, 64);
    const char msg[] = "\n*** Fatal signal, backtrace:\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    signal(sig, SIG_DFL);
    raise(sig);
}

int main(int argc, char **argv)
{
    signal(SIGSEGV, crashHandler);
    signal(SIGABRT, crashHandler);
    srand(time(nullptr));
    const double program_start = epochNow();
    std::chrono::steady_clock::time_point start_init = std::chrono::steady_clock::now();
    std::cout << "C++ version: " << __cplusplus << std::endl;
    if(argc != 9 && argc != 10) // [FIX] Đổi thành 9 tham số cho chắc (tính cả tên chương trình)
    {
        cerr << endl << "Usage:\n"
                        " ./oa-slam_rgbd\n"
                        "      vocabulary_file\n"
                        "      camera_file\n"
                        "      path_to_sequence (root folder)\n"
                        "      path_to_association_file (.txt)\n"
                        "      detections_file (.json, .onnx or .engine)\n"
                        "      categories_to_ignore_file\n"
                        "      relocalization_mode ('points', 'objects' or 'points+objects')\n"
                        "      output_name \n"
                        "      [use_viewer (0: off, 1: on and closed at the end; default: on, waits for Quit)]\n";
        return 1;
    }

    std::string vocabulary_file = string(argv[1]);
    std::string parameters_file = string(argv[2]);
    string path_to_sequence = string(argv[3]);
    string path_to_association = string(argv[4]);
    std::string detections_file(argv[5]);
    std::string categories_to_ignore_file(argv[6]);
    string reloc_mode = string(argv[7]);
    string output_name = string(argv[8]);
    bool use_viewer = argc < 10 || std::string(argv[9]) != "0";
    bool close_viewer_at_end = argc == 10;

    // [FIX 1] Tạo output folder
    string output_folder = output_name;
    if (output_folder.back() != '/')
        output_folder += "/";
    fs::create_directories(output_folder);

    if (path_to_sequence.back() != '/')
        path_to_sequence += "/";

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = path_to_association;
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

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
    bool detect_from_file = false;
    if (extension == "onnx") { // load network
        detector = std::make_shared<ORB_SLAM2::ObjectDetector>(detections_file, classes_to_ignore);
        detect_from_file = false;
        std::cout << "Object detector loaded from ONNX file: " << detections_file << "\n";
    } else if (extension == "engine") { // TensorRT engine
#ifdef USE_TENSORRT
        detector = std::make_shared<ORB_SLAM2::TRTDetector>(detections_file, classes_to_ignore);
        detect_from_file = false;
        std::cout << "Object detector loaded from TensorRT engine: " << detections_file << "\n";
#else
        std::cerr << "This build has no TensorRT support; .engine models cannot be used.\n";
        return 1;
#endif
    } else if (extension == "json") { // load from external detections file
        detector = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file, classes_to_ignore);
        detect_from_file = true;
    } else {
        std::cout << "Invalid detection file. It should be .json, .onnx or .engine\n"
                      "No detections will be obtained.\n";
    }

    // Relocalization mode
    ORB_SLAM2::enumRelocalizationMode relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    if (reloc_mode == string("points"))
        relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    else if (reloc_mode == std::string("objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
    else if (reloc_mode == std::string("points+objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
    else {
        std::cerr << "Error: Invalid parameter for relocalization mode.\n";
        return 1;
    }

    // Create system
    ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::RGBD, use_viewer, false, false);
    std::cout << "System loaded.\n";
    SLAM.SetRelocalizationMode(relocalization_mode);

    vector<float> vTimesTrack;
    vTimesTrack.reserve(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);

    // Main loop
    cv::Mat imRGB, imD;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    poses.reserve(nImages);
    std::vector<std::string> filenames;
    filenames.reserve(nImages);
    std::vector<double> timestamps;
    timestamps.reserve(nImages);

    // Per-frame computation cost (ms): object detection vs. SLAM tracking
    std::ofstream file_times(output_folder + "frame_times_" + output_name + ".csv");
    file_times << "frame,timestamp,wall_clock,n_detections,t_detect_ms,t_track_ms,t_total_ms\n";
    
    // Initialization = program start -> detector + SLAM system ready (vocabulary, engine, threads)
    std::chrono::steady_clock::time_point end_init = std::chrono::steady_clock::now();
    const double init_time = std::chrono::duration<double>(end_init - start_init).count();
    const double loop_start = epochNow();
    // Per-frame processing time = detection + TrackRGBD (image loading excluded, as in rgbd_tum.cc)
    std::vector<double> v_t_frame;
    v_t_frame.reserve(nImages);

    int ni = 0;
    while(1){
        if (ni >= nImages) break; // Check bounds

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        std::string filenameIm = path_to_sequence + vstrImageFilenamesRGB[ni];
        //std::cout << "Processing image: " << filenameIm << "\n";
        std::string filenameD = path_to_sequence + vstrImageFilenamesD[ni];
        //std::cout << "Processing depth: " << filenameD << "\n";

        imRGB = cv::imread(filenameIm, cv::IMREAD_UNCHANGED);
        imD = cv::imread(filenameD, cv::IMREAD_UNCHANGED);
        
        double tframe = vTimestamps[ni];
        timestamps.push_back(tframe);

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: " << filenameIm << endl;
            return 1;
        }
        
        if(imD.empty())
        {
            cerr << endl << "Failed to load depth image at: " << filenameD << endl;
            return 1;
        }

        // [FIX 2] Sử dụng đúng tên biến filenameIm
        filenames.push_back(filenameIm);

        // Get object detections
        std::chrono::steady_clock::time_point t_det0 = std::chrono::steady_clock::now();
        std::vector<ORB_SLAM2::Detection::Ptr> detections;
        if (detector) {
            if (detect_from_file)
                detections = detector->detect(filenameIm); 
            else
                detections = detector->detect(imRGB);
        }

        std::chrono::steady_clock::time_point t_det1 = std::chrono::steady_clock::now();

        // Pass to SLAM
        cv::Mat m = SLAM.TrackRGBD(imRGB, imD, tframe, detections, false);
        std::chrono::steady_clock::time_point t_trk1 = std::chrono::steady_clock::now();
        
        // [FIX 3] Lưu Pose vào vector poses
        if (!m.empty())
            poses.push_back(ORB_SLAM2::cvToEigenMatrix<double, float, 4, 4>(m));
        else
            poses.push_back(Eigen::Matrix4d::Identity());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack.push_back(ttrack);
        v_t_frame.push_back(std::chrono::duration<double>(t_trk1 - t_det0).count());
        auto ms = [](std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        double wall_clock = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        file_times << ni << "," << std::fixed << tframe << "," << wall_clock << "," << detections.size() << ","
                   << ms(t_det0, t_det1) << "," << ms(t_det1, t_trk1) << "," << ms(t1, t2) << "\n";
        std::cout << "time = " << ttrack << "\n";

        if (SLAM.ShouldQuit())
            break;

        ++ni;
    }

    file_times.close();
    const double loop_end = epochNow();
    if (close_viewer_at_end)
        SLAM.RequestViewerFinish();
    SLAM.Shutdown();

    // Save outputs
    std::ofstream file(output_folder + "camera_poses_" + output_name + ".txt");
    std::ofstream file_tum(output_folder + "camera_poses_" + output_name + "_tum.txt");
    json json_data;
    
    for (unsigned int i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d m = poses[i];
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
        pose.block<3, 1>(0, 3) = -m.block<3, 3>(0, 0).transpose() * m.block<3, 1>(0, 3);

        file << i << " " << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                  << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                  << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";

        json R({{m(0, 0), m(0, 1), m(0, 2)}, {m(1, 0), m(1, 1), m(1, 2)}, {m(2, 0), m(2, 1), m(2, 2)}});
        json t({m(0, 3), m(1, 3), m(2, 3)});
        json image_data;
        image_data["file_name"] = filenames[i];
        image_data["R"] = R;
        image_data["t"] = t;
        json_data.push_back(image_data);

        auto q = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
        auto p = pose.block<3, 1>(0, 3);
        file_tum << std::fixed << timestamps[i] << " " << p[0] << " " << p[1] << " " << p[2] << " " 
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n"; 
    }
    file.close();
    file_tum.close();

    std::ofstream json_file(output_folder + "camera_poses_" + output_name + ".json");
    json_file << json_data;
    json_file.close();

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(size_t ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl << endl;

    // statistics.txt in the format of ORB-SLAM3 / YDM-SLAM (tracking time = detection + TrackRGBD)
    {
        std::vector<double> sorted_t(v_t_frame);
        std::sort(sorted_t.begin(), sorted_t.end());
        double sum_t = 0.0, sum_fps = 0.0, max_fps = 0.0, min_fps = std::numeric_limits<double>::max();
        for (double t : v_t_frame) {
            double fps = 1.0 / t;
            sum_t += t;
            sum_fps += fps;
            max_fps = std::max(max_fps, fps);
            min_fps = std::min(min_fps, fps);
        }
        const size_t n = std::max<size_t>(v_t_frame.size(), 1);
        std::ofstream stats_file(output_folder + "statistics.txt");
        stats_file << std::setprecision(10);
        stats_file << "-------\n\n";
        stats_file << "median tracking time: " << (sorted_t.empty() ? 0.0 : sorted_t[sorted_t.size() / 2]) << "\n";
        stats_file << "mean tracking time: " << sum_t / n << "\n";
        stats_file << "Total Time: " << std::chrono::duration<double>(std::chrono::steady_clock::now() - start_init).count() << "\n";
        stats_file << "Initialize time: " << init_time << "\n";
        stats_file << "mean FPS: " << sum_fps / n << "\n";
        stats_file << "max FPS: " << max_fps << "\n";
        stats_file << "min FPS: " << min_fps << "\n";
        stats_file << std::fixed << std::setprecision(6);
        stats_file << "program_start: " << program_start << "\n";
        stats_file << "loop_start: " << loop_start << "\n";
        stats_file << "loop_end: " << loop_end << "\n";
    }

    SLAM.SaveKeyFrameTrajectoryTUM(output_folder + "keyframes_poses_" + output_name + "_tum.txt");
    SLAM.SaveKeyFrameTrajectoryJSON(output_folder + "keyframes_poses_" + output_name + ".json", filenames);
    SLAM.SaveMapPointsOBJ(output_folder + "map_points_" + output_name + ".obj");
    SLAM.SaveMapObjectsOBJ(output_folder + "map_objects_" + output_name + ".obj");
    SLAM.SaveMapObjectsTXT(output_folder + "map_objects_" + output_name + ".txt");
    std::cout << "\n";

    osmap.mapSave(output_folder + "map_" + output_name);

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}