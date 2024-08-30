#include "Slam.hpp"
#include "Helpers.hpp"
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

    // Step 1: Read the frame from video/camera, do the ORB feature extraction and store in frame
    // Step 2: Do the initialization if the initialization has not been done, else do the tracking with the motion model
    // Step 3: Update map points with the new feature points

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

            // velocity = currentPose * previousPose_invert
            _velocity = _currentFrame->Tcw() * _previousFrame->Tcw().inv();

            // add map points
            updateMap(_currentFrame->GetMapPoints());

        }
        else
        {
            trackWithMotionModel();
        }
    }
    _previousFrame = _currentFrame;
    _previousImage = _currentImage.clone();
}

void Slam::GrabImage(const cv::Mat& image) {
    //std::lock_guard<std::mutex> lock(_image_mutex);
    // Check if the image is already grayscale
    if (image.channels() == 3) {
        // Convert the image to grayscale
        cv::cvtColor(image, _currentImage, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4) {
        // Convert the image from BGRA to grayscale
        cv::cvtColor(image, _currentImage, cv::COLOR_BGRA2GRAY);
    }
    else {
        // If the image is already grayscale, clone it directly
        _currentImage = image.clone();
    }
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
    // The initialization process is for finding the camera pose from the 2 first frames
    // Step 1: finding the orb keypoints from the 2 frames
    // Step 2: do the feature matching to have the good_maches which contains the matched indexes between 2 frames
    // Step 3: find the Essential Matrix
    // Step 4: recover pose from the matches keypoints and essential matrix
    // Step 5: triangulate the 3d points from the recovered pose and the the 2d matched keypoints from 2 consecutive frames 

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

    // Mark initialization as done
    _initializationDone = true;
}

void Slam::trackWithMotionModel()
{
    // Step 1: Set the current camera pose by multiply the velocity and the previous camera pose
    // Step 2: Because we have the current camera pose, so we can do the projection search from 3d map points to match the 3d map points with the current frame keypoints
    // Step 3: Do the pose optimization using g2o with the map points and current frame keypoints
    // Step 4: calculate the camera velocity: velocity = currentPose * previousPose_invert
    // Step 5: Create new 3d points from the current frame keypoint which are not in matched by the above projection search

    // initialize map points vector with the size of keypoints, but currently the map points are nulls
    _currentFrame->InitializeMapPoints();

    START_MEASURE_TIME("MatchKeyPoints")

    // set the current camera pose by the motion model: current_pose = velocity*previous_pose
    _currentFrame->SetTcw(_velocity * _previousFrame->Tcw());

    std::vector<bool> mask;
    std::vector<cv::Point2f> notYetMachedPoints;
    int trackedPtsCnt = SearchByProjection(_currentFrame, _map, _intrinsicCameraMatrix, 15, mask);
    for (int i = 0; i < mask.size(); i++)
    {
        if (!mask[i])
        {
            notYetMachedPoints.push_back(_currentFrame->KeyPoints()[i].pt);
        }
    }

    std::vector<bool> mask1;
    std::vector<cv::Point2f> previousFramePoints = trackKeypointsOpticalFlow(_previousImage, _currentImage, notYetMachedPoints, mask1);
    std::vector<cv::Point2f> notYetMachedPointsFilter; // the notYetMachedPoints which have the previous points calculated by opticalflow
    std::vector<cv::Point2f> previousFramePointsFilter; // the notYetMachedPoints which have the previous points calculated by opticalflow

    //Merge `mask` and `mask1`
    size_t j = 0;
    for (size_t i = 0; i < mask.size(); ++i)
    {
        if (!mask[i])  // Check for points that were not matched in the first pass
        {
            // Update the mask with the combined result of mask1
            mask[i] = mask1[j];
            if (mask1[j]) {
                notYetMachedPointsFilter.push_back(notYetMachedPoints[j]);
                previousFramePointsFilter.push_back(previousFramePoints[j]);
            }
            ++j;
        }
    }

#if 1
    // Make a copy of the current image to draw on
    cv::Mat debugImage = _currentImage.clone();

    // Convert to color if the image is grayscale
    if (debugImage.channels() == 1) {
        cv::cvtColor(debugImage, debugImage, cv::COLOR_GRAY2BGR);
    }

    // Draw the optical flow vectors
    for (size_t i = 0; i < previousFramePointsFilter.size(); ++i) {
        // Draw a line from the previous point to the current point
        cv::line(debugImage, previousFramePointsFilter[i], notYetMachedPointsFilter[i], cv::Scalar(0, 255, 0), 2);

        // Optionally, draw the points as well
        cv::circle(debugImage, notYetMachedPointsFilter[i], 3, cv::Scalar(0, 0, 255), -1);
    }

#endif

    STOP_MEASURE_TIME("MatchKeyPoints")

    std::cout << "[Cpp] Track point count = " << trackedPtsCnt << std::endl;

    START_MEASURE_TIME("PoseOptimization")
    // do the pose optimization to get the correct transformation from the world coordinate system to the current camera coordinate system
    Optimizer::PoseOptimization(_currentFrame, _cameraInfo);
    STOP_MEASURE_TIME("PoseOptimization")

    // Extract the 3x4 projection matrix from the 4x4 transformation matrix Tcw
    cv::Mat Tcw0 = _previousFrame->Tcw();
    cv::Mat Tcw1 = _currentFrame->Tcw();

    //std::cout << "[Cpp] previous tranform " << Tcw0 << std::endl;
    //std::cout << "[Cpp] current tranform " << Tcw1 << std::endl;
    std::cout << "[Cpp] input for MyTriangulatePoints size " << previousFramePointsFilter.size() << " " << notYetMachedPointsFilter.size() << std::endl;

    // Create 3x4 matrices by selecting the first three rows of Tcw (ignoring the last row)
    cv::Mat P0 = _intrinsicCameraMatrix * Tcw0(cv::Rect(0, 0, 4, 3));
    cv::Mat P1 = _intrinsicCameraMatrix * Tcw1(cv::Rect(0, 0, 4, 3));

    // Triangulate notYetMachedPoints in current frame
    // Projection matrices for the two views
    std::vector<cv::Point3d> p3ds = MyTriangulatePoints(P0, P1, previousFramePointsFilter, notYetMachedPointsFilter);

    // calculate the motion velocity
    _velocity = _currentFrame->Tcw() * _previousFrame->Tcw().inv();

    // Create new 3d points from the current frame keypoint which are not in matched by the above projection search
    size_t k = 0;
    int newPoint = 0;
    for (int i = 0; i < _currentFrame->KeyPoints().size(); i++)
    {
        auto mp = _currentFrame->GetMapPoints()[i];
        if (!mp && mask[i]) // not yet and map point, and it is a valid tracked point
        {
            if (p3ds[k].z < 0) // point is behind the camera
                continue;

            //std::cout << "[Cpp] New Map Point at index " << i << std::endl;

            newPoint++;
            _currentFrame->GetMapPoints()[i] = std::make_shared<MapPoint>();
            _currentFrame->GetMapPoints()[i]->SetPosition(p3ds[k]);
            _currentFrame->GetMapPoints()[i]->SetDescriptor(_currentFrame->Descriptors().row(i));
            _currentFrame->GetMapPoints()[i]->AddObservation(_currentFrame);
        }
        else if(mp)
        {
            // For DEBUG
            cv::circle(debugImage, _currentFrame->KeyPoints()[i].pt, 3, cv::Scalar(255, 0, 0), -1);
        }
    }

    std::cout << "[Cpp] New 3d points " << newPoint << std::endl;

    updateMap(_currentFrame->GetMapPoints());

    // so far, I add all the frames
    //_keyFrames.push_back(_currentFrame);

    // TODO: Do Bundle Adjustment here if the keyframe count >= 5

    // For DEBUG
    cv::imshow("Optical Flow", debugImage);
    cv::waitKey(1);  // Wait for a key press to close the window

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
        if (mapPoint) {
            // Insert the shared pointer into the set
            _map.insert(mapPoint);
        }
    }
    std::cout << "[Cpp] map size after updated " << _map.size() << std::endl;
}

// Function to compute optical flow and track keypoints
std::vector<cv::Point2f> Slam::trackKeypointsOpticalFlow(const cv::Mat& prevImg, const cv::Mat& currImg, const std::vector<cv::Point2f>& currKeypoints, std::vector<bool>& mask) {
    mask.resize(currKeypoints.size(), false);
    std::vector<cv::Point2f> prevKeypoints;  // Will hold the keypoints in the previous image
    std::vector<uchar> status;  // Status vector to check if keypoints were successfully tracked
    std::vector<float> err;  // Error vector

    // Calculate optical flow from the current image to the previous image
    cv::calcOpticalFlowPyrLK(currImg, prevImg, currKeypoints, prevKeypoints, status, err);

    // Update mask based on the criteria
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            // Calculate the displacement vector
            cv::Point2f displacement = prevKeypoints[i] - currKeypoints[i];
            float distance = cv::norm(displacement);

            // Calculate the angle of the displacement in degrees
            float angle = std::atan2(displacement.y, displacement.x) * 180.0 / CV_PI;

            // Normalize the angle to the range [-180, 180]
            if (angle > 180.0f) angle -= 360.0f;
            if (angle < -180.0f) angle += 360.0f;

            // Check if the angle is within the ±15 degrees in the x-direction
            bool withinAngleThreshold = (std::abs(angle) <= 15.0f);

            // Check if the distance is within the threshold
            bool withinDistanceThreshold = (distance <= 30.0f);

            // Update mask if both criteria are met
            mask[i] = !withinAngleThreshold && withinDistanceThreshold;
        }
        else {
            mask[i] = false; // Mark as false if tracking failed
        }
    }

    return prevKeypoints;
}

