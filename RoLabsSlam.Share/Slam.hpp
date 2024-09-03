#pragma once

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include "MyTypes.hpp"
#include "Frame.hpp"

class Slam {
public:
    Slam();
    ~Slam();

    void Start();
    void Stop();
    void GrabImage(const cv::Mat& image);
    void GetDebugKeyPoints(std::vector<cv::KeyPoint>* keypointsCurrent, std::vector<cv::KeyPoint>* keypointsPrevious) const;
    void SetCameraInfo(float fx, float fy, float cx, float cy);
    void GetCurrentFramePose(cv::Mat* pose);
    void Track();
    void GetMapPoints(std::vector<cv::Point3f>* mapPoints);

private:
    void initialization();
    void updateMap(const std::vector<std::shared_ptr<MapPoint>>& mapPoints);
    void updateMap(std::shared_ptr<Frame> currentKeyFrame);
    void trackWithMotionModel();
    bool createMapPoints(std::vector<bool> mask);
    std::vector<cv::Point2f> trackKeypointsOpticalFlow(const cv::Mat& prevImg, const cv::Mat& currImg, const std::vector<cv::Point2f>& currKeypoints, std::vector<bool>& mask);
    bool isKeyFrame(int trackPointCnt);
    void trackWithPreviousKeyFrame();

    std::mutex _image_mutex;
    mutable std::mutex _frame_mutex;
    cv::Mat _currentImage;
    cv::Mat _previousImage;
    bool _running;

    int _frameCount;

    bool _initializationDone;

    std::shared_ptr<Frame> _currentFrame;
    std::shared_ptr<Frame> _previousFrame;

    std::set<std::shared_ptr<MapPoint>> _map;
    std::set<std::shared_ptr<MapPoint>> _localMap;

    std::vector<std::shared_ptr<Frame>> _keyFrames;

    CameraInfo _cameraInfo;

    cv::Mat _intrinsicCameraMatrix;

    cv::Mat _velocity;
};
