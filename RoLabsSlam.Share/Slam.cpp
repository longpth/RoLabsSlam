#include "Slam.hpp"
#include "helpers.hpp"
#include "Optimizer.hpp"

#if defined(_WIN64) || defined(_WIN32)

#else
#include "pch.h"
#endif


// Define a map to store the start time for each block
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> startTimes;

#define START_MEASURE_TIME(blockName) \
    startTimes[blockName] = std::chrono::high_resolution_clock::now();

#define STOP_MEASURE_TIME(blockName) \
{ \
    auto end = std::chrono::high_resolution_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - startTimes[blockName]).count(); \
    std::cout << "[Cpp] Time taken for " << blockName << ": " << duration << " ms" << std::endl; \
    startTimes.erase(blockName); \
}

Slam::Slam() : _running(false) {
    _cameraInfo.cx = 700;
    _cameraInfo.cy = 700;
    _cameraInfo.fx = 240;
    _cameraInfo.fy = 320;
}

Slam::~Slam() {
    Stop();  // Ensure threads are stopped when the object is destroyed
}

void Slam::Start() {
    if (!_running) {
        _running = true;
        _frameCount = 0;
        _initializationDone = false;
    }
}

void Slam::Stop() {
    _running = false;
}

void Slam::Track() {

    cv::Mat image;
    {
        std::lock_guard<std::mutex> lock(_image_mutex);
        image = _currentImage.clone();
    }

    // Tracking code here (feature extraction, pose estimation, etc.)
    auto new_frame = std::make_shared<Frame>(image);
    {
        //std::lock_guard<std::mutex> lock(_frame_mutex);
        _currentFrame = new_frame;
    }

    if (_frameCount >= 2)
    {
        if (!_initializationDone)
        {
            initialization();
        }
        else
        {
            trackWithMotionModel();
        }
    }
    _previousFrame = _currentFrame;
}

void Slam::GrabImage(const cv::Mat& image) {
    //std::lock_guard<std::mutex> lock(_image_mutex);
    _currentImage = image.clone();
    _frameCount++;
}

void Slam::GetDebugKeyPoints(std::vector<cv::KeyPoint>* keypointsCurrent, std::vector<cv::KeyPoint>* keypointsPrevious) const {
    //std::lock_guard<std::mutex> lock(_frame_mutex);
    if (_currentFrame && keypointsCurrent) {
        *keypointsCurrent = _currentFrame->KeyPoints();
    }
    if (_previousFrame && keypointsPrevious)
    {
        *keypointsPrevious = _previousFrame->KeyPoints();
    }
}

void Slam::initialization()
{
    // Ensure that both frames are valid
    if (!_currentFrame || !_previousFrame || _currentFrame->KeyPoints().empty() || _previousFrame->KeyPoints().empty()) {
        std::cerr << "Error: Invalid frames or no keypoints detected." << std::endl;
        _initializationDone = false;
        return;
    }

    // Match keypoints between frames
    std::vector<cv::DMatch> good_matches = MatchKeyPoints(*_currentFrame, *_previousFrame);
    // Ensure there are at least 8 matches
    if (good_matches.size() < 8) {
        std::cerr << "Error: No matches found between frames." << std::endl;
        _initializationDone = false;
        return;
    }

    // Estimate the Essential Matrix, previous to current frame
    cv::Mat essential_matrix = EstimateEssentialMatrix(*_currentFrame, *_previousFrame, good_matches, _intrinsicCameraMatrix);
    if (essential_matrix.empty()) {
        std::cerr << "Error: Could not estimate the Essential Matrix." << std::endl;
        _initializationDone = false;
        return;
    }
    std::cout << "[Cpp] Essential Matrix = " << essential_matrix << std::endl;

    // Decompose essential matrix, triangulate points, and filter outliers
    FindRtAndTriangulate(
        essential_matrix,
        _intrinsicCameraMatrix,
        good_matches,
        _currentFrame,
        _previousFrame
        );

    _keyFrames.push_back(_previousFrame);
    _keyFrames.push_back(_currentFrame);

    Optimizer::PoseOptimization(_currentFrame, _cameraInfo);


    // velocity = currentPose * previousPose_invert
    _velocity = _currentFrame->Tcw() * _previousFrame->Tcw().inv();

    // add map points
    updateMap(_currentFrame->GetMapPoints());

    // Mark initialization as done
    _initializationDone = true;
}

void Slam::trackWithMotionModel()
{
    // initialize map points vector with the size of keypoints, but currently the map points are nulls
    _currentFrame->InitializeMapPoints();

    START_MEASURE_TIME("MatchKeyPoints")

    // set the current camera pose by the motion model: current_pose = velocity*previous_pose
    _currentFrame->SetTcw(_velocity * _previousFrame->Tcw());

    int trackedPtsCnt = SearchByProjection(_currentFrame, _map, _intrinsicCameraMatrix, 15);

    STOP_MEASURE_TIME("MatchKeyPoints")

    std::cout << "[Cpp] Track point count = " << trackedPtsCnt << std::endl;

    START_MEASURE_TIME("PoseOptimization")
    // do the pose optimization to get the correct transformation from the world coordinate system to the current camera coordinate system
    Optimizer::PoseOptimization(_currentFrame, _cameraInfo);
    STOP_MEASURE_TIME("PoseOptimization")

    _velocity = _currentFrame->Tcw() * _previousFrame->Tcw().inv();
}

void Slam::SetCameraInfo(float fx, float fy, float cx, float cy)
{
    _cameraInfo.fx = fx;
    _cameraInfo.fy = fy;
    _cameraInfo.cx = cx;
    _cameraInfo.cy = cy;

    // Camera intrinsic parameters (assuming fx, fy, cx, cy are known and properly initialized)
    _intrinsicCameraMatrix = (cv::Mat_<double>(3, 3) << _cameraInfo.fx, 0, _cameraInfo.cx, 0, _cameraInfo.fy, _cameraInfo.cy, 0, 0, 1);
}

void Slam::GetCurrentFramePose(cv::Mat *pose)
{
    if (_currentFrame) {
        //std::cout << "[Cpp] get current transformation = " << _currentFrame->Tcw() << std::endl;
        _currentFrame->Tcw().copyTo(*pose);
    }
}

void Slam::updateMap(const std::vector<std::shared_ptr<MapPoint>>& mapPoints) {
    for (const std::shared_ptr<MapPoint>& mapPoint : mapPoints) {
        // Insert the shared pointer into the set
        _map.insert(mapPoint);
    }
    std::cout << "[Cpp] MapPoints created " << _map.size() << std::endl;
}