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
        _previousFrame,
        _invertmedianDepth
        );

    _allFrames.push_back(_previousFrame);
    _allFrames.push_back(_currentFrame);

    //Optimizer::PoseOptimization(_currentFrame, _cameraInfo);

    // Mark initialization as done
    _initializationDone = true;
}

void Slam::trackWithMotionModel()
{
    // Step 1: Set the current camera pose by multiply the velocity and the previous camera pose
    // Step 2: calculate the camera velocity: velocity = currentPose * previousPose_invert
    // Step 3: Because we have the current camera pose, so we can do the projection search from 3d map points to match the 3d map points with the current frame keypoints
    // Step 4: Do the pose optimization using g2o with the map points and current frame keypoints
    // step 5: Check if we need to create new 3D map points, if yes, create new 3d points from the current frame keypoint which are not in matched by the above projection search by using OpticalFlow to find the 2d point in previous frame then do the triangulation, if could not match with optical flow, matching with BFMacher.
    // Step 6: Request do bundle adjustment

    // initialize map points vector with the size of keypoints, but currently the map points are nulls
    _currentFrame->InitializeMapPoints();

    START_MEASURE_TIME("MatchKeyPoints")

    std::cout << "[Cpp] velocity = " << _velocity << std::endl;

    // set the current camera pose by the motion model: current_pose = velocity*previous_pose
    _currentFrame->SetTcw(_velocity * _previousFrame->Tcw());

    std::vector<bool> mask;

    int trackedPtsCnt = SearchByProjection(_currentFrame, _previousFrame, _intrinsicCameraMatrix, 15, mask);

    STOP_MEASURE_TIME("MatchKeyPoints")

    START_MEASURE_TIME("PoseOptimization")
    // do the pose optimization to get the correct transformation from the world coordinate system to the current camera coordinate system
    Optimizer::PoseOptimization(_currentFrame, _cameraInfo);
    STOP_MEASURE_TIME("PoseOptimization")

    std::cout << "[Cpp] Track point count = " << trackedPtsCnt << std::endl;

    bool newMapPointNeeded = needNewMapPoints(trackedPtsCnt);

    _allFrames.push_back(_currentFrame);

    bool createMapPointSuccess = true;

    if (newMapPointNeeded) {
        START_MEASURE_TIME("createMapPoints")
        createMapPointSuccess = createMapPoints(mask);
        STOP_MEASURE_TIME("createMapPoints")

        if (createMapPointSuccess) {
            START_MEASURE_TIME("updateMap")
            updateMap(_currentFrame);

            //_currentFrame->UpdateLocalKeyFrames();

            STOP_MEASURE_TIME("updateMap")

            std::cout << "[Cpp] Created new map points for frame ************** " << _currentFrame->Id() << std::endl;

        }
    }

    if (createMapPointSuccess) {

    }
    else {
        _currentFrame->SetTcw(_previousFrame->Tcw());
    }

    START_MEASURE_TIME("BundleAdjustment2")
    if (_allFrames.size() >= 10 && _allFrames.size() % 5 == 0 && trackedPtsCnt > 80) {
        // Do bundle adjustment
        Optimizer::BundleAdjustment2(_allFrames, _localMap, _cameraInfo);
    }
    STOP_MEASURE_TIME("BundleAdjustment2")

    // calculate the motion velocity

    _velocity = _currentFrame->Tcw() * _previousFrame->Tcw().inv();
}

#define DEBUG 0

bool Slam::createMapPoints(std::vector<bool> mask)
{
    bool ret = true;

    std::vector<cv::Point2f> currentFramePoints; // keypoint that not yet reconstructed into 3d point

    for (int i = 0; i < mask.size(); i++)
    {
        if (!mask[i])
        {
            currentFramePoints.push_back(_currentFrame->KeyPoints()[i].pt);
        }
    }

    std::vector<bool> maskOpticalFlow;
    std::vector<cv::Point2f> previousFramePoints = trackKeypointsOpticalFlow(_previousImage, _currentImage, currentFramePoints, maskOpticalFlow);
    std::vector<cv::Point2f> currentFramePointsFilter; // the currentFramePoints which have the previous points calculated by opticalflow
    std::vector<cv::Point2f> previousFramePointsFilter; // the currentFramePoints which have the previous points calculated by opticalflow

    for (size_t i = 0; i < maskOpticalFlow.size(); ++i)
    {
        if (maskOpticalFlow[i]) {
            currentFramePointsFilter.push_back(currentFramePoints[i]);
            previousFramePointsFilter.push_back(previousFramePoints[i]);
        }
    }

    std::cout << "[Cpp] Number of good points from optical flow = " << currentFramePointsFilter.size() << " " << previousFramePointsFilter.size() << std::endl;
    std::cout << "[Cpp] Percentage of good points from optical flow = " << ((float)currentFramePointsFilter.size() / (float)currentFramePoints.size()) << std::endl;

    float percentage_tracked_optical = (float)currentFramePointsFilter.size() / (float)currentFramePoints.size();

    std::vector<cv::DMatch> good_matches;
    if (percentage_tracked_optical < 0.1)
    {
        currentFramePointsFilter.clear();
        previousFramePointsFilter.clear();
        std::cout << "[Cpp] not good tracking with opticalflow, track with BFMatcher \n";

        // track with BFMatcher
        good_matches = MatchKeyPoints(*_currentFrame, *_previousFrame);
        for (const auto& match : good_matches) {
            currentFramePointsFilter.push_back(_currentFrame->KeyPoints()[match.queryIdx].pt);
            previousFramePointsFilter.push_back(_previousFrame->KeyPoints()[match.trainIdx].pt);
        }
    }

    // Estimate the Essential Matrix, previous to current frame
    cv::Mat essential_matrix = cv::findEssentialMat(previousFramePointsFilter, currentFramePointsFilter, _intrinsicCameraMatrix, cv::RANSAC, 0.999, 1.0);
    std::cout << "[Cpp] Essential Matrix = " << essential_matrix << std::endl;

    // Recover pose
    cv::Mat R, t, mask2;
    int inliersCount = cv::recoverPose(essential_matrix, previousFramePointsFilter, currentFramePointsFilter, _intrinsicCameraMatrix, R, t, mask2); // five-point.cpp opencv

    if (inliersCount < 50)
    {
        std::cout << "inliers from recoverPose are not enough " << inliersCount << "\n";
        ret = false;
    }
    else
    {
        // Filter inliers and prepare for triangulation
        std::vector<cv::Point2f> currentPoints, previousPoints;

        // track with BFMatcher
        std::vector<cv::KeyPoint> inlierPreviousFrameKeypoints; // Output inliers for previousFrame
        std::vector<cv::KeyPoint> inlierCurrentFrameKeypoints;  // Output inliers for currentFrame
        std::vector<int> previousIndices, currentIndices;       // Output valid descriptors

        if (percentage_tracked_optical >= 0.1)
        {
            int j = 0;
            for (size_t i = 0; i < maskOpticalFlow.size(); ++i)
            {
                if (maskOpticalFlow[i]) {
                    maskOpticalFlow[i] = bool(mask2.at<uchar>(j));
                    if (mask2.at<uchar>(j))
                    {
                        // Keep the inliers
                        currentPoints.push_back(currentFramePointsFilter[j]);
                        previousPoints.push_back(previousFramePointsFilter[j]);
                    }
                    j++;
                }
            }
        }
        else
        {
            for (int i = 0; i < mask2.rows; i++)
            {
                if (mask2.at<uchar>(i))
                {
                    // Keep the inliers
                    inlierCurrentFrameKeypoints.push_back(_currentFrame->KeyPoints()[good_matches[i].queryIdx]);
                    inlierPreviousFrameKeypoints.push_back(_previousFrame->KeyPoints()[good_matches[i].trainIdx]);
                    currentPoints.push_back(currentFramePointsFilter[i]);
                    previousPoints.push_back(previousFramePointsFilter[i]);

                    // Keep the descriptors of inliers
                    currentIndices.push_back(good_matches[i].queryIdx);
                    previousIndices.push_back(good_matches[i].trainIdx);
                }
            }
        }

#if DEBUG
        // Make a copy of the current image to draw on
        cv::Mat debugImage = _currentImage.clone();

        // Convert to color if the image is grayscale
        if (debugImage.channels() == 1) {
            cv::cvtColor(debugImage, debugImage, cv::COLOR_GRAY2BGR);
        }

        // Draw the optical flow vectors
        for (size_t i = 0; i < previousFramePointsFilter.size(); ++i) {
            // Draw a line from the previous point to the current point
            cv::line(debugImage, previousFramePointsFilter[i], currentFramePointsFilter[i], cv::Scalar(0, 255, 0), 2);

            // Optionally, draw the points as well
            cv::circle(debugImage, currentFramePointsFilter[i], 3, cv::Scalar(0, 0, 255), -1);

            cv::imshow("Optical Flow", debugImage);
            cv::waitKey(1);  // Wait for a key press to close the window
    }
#endif

        // Create 3x4 matrices by selecting the first three rows of Tcw (ignoring the last row)
        cv::Mat P0 = _intrinsicCameraMatrix * cv::Mat::eye(3, 4, CV_64F); // [I | 0]
        cv::Mat P1(3, 4, CV_64F);
        R.copyTo(P1(cv::Rect(0, 0, 3, 3))); // Copy R into the first 3x3 part of P1
        t.copyTo(P1(cv::Rect(3, 0, 1, 3))); // Copy t into the last column of P1
        P1 = _intrinsicCameraMatrix * P1;

        // Triangulate currentFramePoints in current frame
        // Projection matrices for the two views
        std::vector<bool> mask3d;
        int goodParallaxCnt = 0;
        std::vector<cv::Point3d> p3ds = MyTriangulatePoints(P0, P1, previousPoints, currentPoints, mask3d, goodParallaxCnt);

        Normalize3DPoints(p3ds, _invertmedianDepth);

        if (p3ds.size() > 0) {

            t *= _invertmedianDepth;

            cv::Mat relTransformation = CreateTransformationMatrix(R, t);
            cv::Mat preTcw = _previousFrame->Tcw();

            cv::Mat curTcw = relTransformation * preTcw;

            _currentFrame->SetTcw(curTcw);

            int newPoint = 0;

            if (percentage_tracked_optical >= 0.1) // create new points with optical flow
            {
                // Create new 3d points from the current frame keypoint which are not in matched by the above projection search
                size_t k = 0;
                size_t kk = 0;
                for (int i = 0; i < _currentFrame->KeyPoints().size(); i++)
                {
                    auto mp = _currentFrame->GetMapPoints()[i];
                    if (!mask[i]) // not yet and map point, and it is a valid tracked point
                    {
                        if (maskOpticalFlow[k])
                        {
                            if (p3ds[kk].z > 0 && mask3d[kk] == true)
                            {
                                // Convert the 3D point to homogeneous coordinates (4x1 vector)
                                cv::Mat pointCameraHomogeneous = (cv::Mat_<double>(4, 1) << p3ds[kk].x, p3ds[kk].y, p3ds[kk].z, 1.0);

                                // Transform the point from camera coordinates to world coordinates using Tcw
                                cv::Mat pointWorldHomogeneous = _currentFrame->Tcw().inv() * pointCameraHomogeneous;

                                // Convert back to a 3D point in world coordinates
                                cv::Point3d pointWorld(
                                    pointWorldHomogeneous.at<double>(0) / pointWorldHomogeneous.at<double>(3),
                                    pointWorldHomogeneous.at<double>(1) / pointWorldHomogeneous.at<double>(3),
                                    pointWorldHomogeneous.at<double>(2) / pointWorldHomogeneous.at<double>(3)
                                );

                                newPoint++;
                                _currentFrame->GetMapPoints()[i] = std::make_shared<MapPoint>();
                                _currentFrame->GetMapPoints()[i]->SetPosition(pointWorld);
                                _currentFrame->GetMapPoints()[i]->SetDescriptor(_currentFrame->Descriptors().row(i));
                            }
                            kk++;
                        }
                        k++;
                    }
                }
            }
            else // create new point by BFMacher
            {
                std::vector<std::shared_ptr<MapPoint>>& current_map_points = _currentFrame->GetMapPoints();
                std::vector<std::shared_ptr<MapPoint>>& previous_map_points = _previousFrame->GetMapPoints();

                for (int i = 0; i < p3ds.size(); i++)
                {
                    if (p3ds[i].z < 0 || mask3d[i] == false || current_map_points[currentIndices[i]] != nullptr)
                        continue; // Skip points with Z < 0 or the point already created by reprojection or less parralax

                    if (previous_map_points[previousIndices[i]]) // map point already associated with the keypoint in previous frame
                    {
                        bool existedMapPoint = false;
                        // check if this previous map point has already assigned for the current frame map or not
                        for (auto mp : current_map_points)
                        {
                            if (mp && (mp->Id() == previous_map_points[previousIndices[i]]->Id()))
                            {
                                existedMapPoint = true;
                                break;
                            }
                        }
                        if (!existedMapPoint)
                        {
                            current_map_points[currentIndices[i]] = previous_map_points[previousIndices[i]];
                            current_map_points[currentIndices[i]]->AddObservation(_currentFrame, currentIndices[i]);
                        }
                    }
                    else
                    {
                        // Convert the 3D point to homogeneous coordinates (4x1 vector)
                        cv::Mat pointCameraHomogeneous = (cv::Mat_<double>(4, 1) << p3ds[i].x, p3ds[i].y, p3ds[i].z, 1.0);

                        // Transform the point from camera coordinates to world coordinates using Tcw
                        cv::Mat pointWorldHomogeneous = _currentFrame->Tcw().inv() * pointCameraHomogeneous;

                        // Convert back to a 3D point in world coordinates
                        cv::Point3d pointWorld(
                            pointWorldHomogeneous.at<double>(0) / pointWorldHomogeneous.at<double>(3),
                            pointWorldHomogeneous.at<double>(1) / pointWorldHomogeneous.at<double>(3),
                            pointWorldHomogeneous.at<double>(2) / pointWorldHomogeneous.at<double>(3)
                        );

                        current_map_points[currentIndices[i]] = std::make_shared<MapPoint>();
                        current_map_points[currentIndices[i]]->SetPosition(pointWorld);
                        current_map_points[currentIndices[i]]->SetDescriptor(_currentFrame->Descriptors().row(currentIndices[i]));
                        newPoint++;
                    }
                }
                std::cout << "[Cpp] New 3d points " << newPoint << std::endl;
            }
        }
        // could not create new keypoints
        else
        {
            ret = false;
        }
    }
    return ret;
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

bool Slam::needNewMapPoints(int numTrackPpoints) {
    double translationThreshold = 0.5;
    double rotationThreshold = 0.087;   // 5 degrees in radians
    int trackedPointsThreshold = 80;    // Require at least 80 tracked points
    double parallaxThreshold = 1.0;     // 1 degree of parallax

    // Check number of tracked points
    if (numTrackPpoints < trackedPointsThreshold) return true;

    // Check translation criterion
    double translationDistance = cv::norm(_currentFrame->Tcw().col(3).rowRange(0, 3) - _allFrames.back()->Tcw().col(3).rowRange(0, 3));
    if (translationDistance > translationThreshold) return true;

    cv::Mat currentRotation = _currentFrame->Tcw()(cv::Rect(0, 0, 3, 3));  // Extract the top-left 3x3 part
    cv::Mat lastKeyframeRotation = _allFrames.back()->Tcw()(cv::Rect(0, 0, 3, 3));  // Extract the top-left 3x3 part

    // Compute the relative rotation matrix
    cv::Mat rotationDifference = currentRotation * lastKeyframeRotation.inv();

    // Calculate the rotation angle
    double angle = acos((cv::trace(rotationDifference)[0] - 1.0) / 2.0);  // Angle in radians

    // Check rotation criterion
    if (angle > rotationThreshold) return true;

    //// Check parallax criterion
    //double averageParallax = ComputeAverageParallax(_currentFrame, _allFrames.back());
    //if (averageParallax > parallaxThreshold) return true;

    // Otherwise, do not consider this frame as a keyframe
    return false;
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
            _localMap.insert(mapPoint);
        }
    }
    std::cout << "[Cpp] map size after updated " << _map.size() << std::endl;
}

void Slam::updateMap(std::shared_ptr<Frame> currentKeyFrame) {
    int i = 0;

    for (const std::shared_ptr<MapPoint>& mapPoint : currentKeyFrame->GetMapPoints()) {
        if (mapPoint) {

            bool isMapPointExisted = false;

            if (!isMapPointExisted) {
                mapPoint->AddObservation(currentKeyFrame, i);
                // Insert the shared pointer into the set
                _map.insert(mapPoint);
                _localMap.insert(mapPoint);
            }
        }
        i++;
    }
    std::cout << "[Cpp] ************** map size after updated " << _map.size() << std::endl;
}

void Slam::GetMapPoints(std::vector<cv::Point3f>* mapPoints)
{
    for (const auto mp : _map)
    {
        cv::Point3f pt((float)mp->GetPosition().x, (float)mp->GetPosition().y, (float)mp->GetPosition().z);
        mapPoints->push_back(pt);
    }
}