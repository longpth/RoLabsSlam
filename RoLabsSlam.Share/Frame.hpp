#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include "MapPoint.hpp"

class Frame : public std::enable_shared_from_this<Frame>
{

public:
    Frame(Frame& other) {
        _keypoints = other.KeyPoints();
        _descriptors = other.Descriptors();
        _Tcw = other.Tcw();
    };

    Frame(const cv::Mat& image);

    // overloading < operator 
    bool operator<(const Frame& other) const {
        return _id < other.Id();
    }

    // Getter methods
    const cv::Mat& Descriptors() const { return _descriptors; }

    const std::vector<cv::KeyPoint>& KeyPoints() const {
        std::unique_lock<std::mutex> lock(mtxKeyPoints);
        return _keypoints; 
    }

    std::vector<cv::KeyPoint> KeyPoints() { 
        std::unique_lock<std::mutex> lock(mtxKeyPoints);
        return _keypoints; 
    }

    void SetKeyPoints(std::vector<cv::KeyPoint> updatedKeyPoints)
    {
        std::unique_lock<std::mutex> lock(mtxKeyPoints);
        _keypoints = updatedKeyPoints;
    }

    void SetKeyPoint(int indx, float x, float y)
    {
        _keypoints[indx].pt.x = x;
        _keypoints[indx].pt.y = y;
    }

    void SetDescriptor(cv::Mat descriptor)
    {
        _descriptors = descriptor;
    }

    // Get the transformation matrix Tcw
    cv::Mat Tcw() const { return _Tcw.clone(); }

    // Set the transformation matrix Tcw
    void SetTcw(const cv::Mat& transformationMatrix) {
        std::unique_lock<std::mutex> lock(mtxTcw);
        _Tcw = transformationMatrix.clone();
    }

    void CopyFrom(Frame& other)
    {
        _keypoints = other.KeyPoints();
        _descriptors = other.Descriptors();
        _Tcw = other.Tcw();
    }

    std::shared_ptr<MapPoint> GetMapPoint(int index)
    {
        std::unique_lock<std::mutex> lock(mtxMapPoints);
        return _mapPoints[index];
    }

    std::vector<std::shared_ptr<MapPoint>>& GetMapPoints()
    {
        std::unique_lock<std::mutex> lock(mtxMapPoints);
        return _mapPoints;
    }

    std::vector<bool>& Outliers()
    {
        return _outliers;
    }

    uint64_t Id() const
    {
        return _id;
    }

    void InitializeMapPoints()
    {
        // Ensure _mapPoints vector is sized to match _keypoints vector
        _mapPoints.resize(_keypoints.size(), nullptr);
        _outliers.resize(_keypoints.size(), true);
    }

    std::shared_ptr<Frame> shared_from_this()
    {
        return std::enable_shared_from_this<Frame>::shared_from_this();
    }

    int Width() const {
        return _width;
    }

    int Height() const {
        return _height;
    }

    void UpdateLocalKeyFrames();

    std::vector<std::shared_ptr<Frame>>& GetLocalKeyFrames()
    {
        std::unique_lock<std::mutex> lock(mtxFrames);
        return _localKeyFrames;
    }

    std::vector<std::shared_ptr<Frame>>& GetFixedFrames()
    {
        std::unique_lock<std::mutex> lock(mtxFrames);
        return _fixedKeyFrames;
    }

    void RemoveMapPoint(MapPoint* mapPoint);

private:
    void detectAndCompute(const cv::Mat& image);
    void detectAndCompute2(const cv::Mat& image);

    std::vector<cv::KeyPoint> _keypoints;
    cv::Mat _descriptors;
    cv::Ptr<cv::ORB> _orb;

    // Camera transformation matrix (4x4) from world to camera
    cv::Mat _Tcw;

    std::vector<std::shared_ptr<MapPoint>> _mapPoints;

    std::vector<bool> _outliers;

    int _width;

    int _height;

    uint64_t _id;
    static uint64_t frameCount;

    std::vector<std::shared_ptr<Frame>> _localKeyFrames;
    std::vector<std::shared_ptr<Frame>> _fixedKeyFrames;

    std::mutex mtxFrames;
    std::mutex mtxMapPoints;
    mutable std::mutex mtxKeyPoints;
    std::mutex mtxTcw;
};
