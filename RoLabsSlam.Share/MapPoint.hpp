#pragma once

#include <opencv2/core.hpp>
#include <map>
#include <memory>
#include "my_functions.h"

class Frame;

class MapPoint
{
public:
    MapPoint();

    // Getters
    cv::Point3d GetPosition() const;
    // Setters
    void SetPosition(const cv::Point3d& position);

    // Add an observation
    void AddObservation(std::shared_ptr<Frame> frame, int keyPointIdex);

    // Get Observations
    std::map<std::shared_ptr<Frame>, uint64_t>& GetObservations();

    // Remove an observation
    void RemoveObservation(Frame* frame);

    // Get the map point descriptor
    cv::Mat GetDescriptor();

    void SetDescriptor(cv::Mat descriptor);

    uint64_t Id()
    {
        return _id;
    }

    static void ResetMapPointCount()
    {
        mapPointCount = 0;
    }

private:
    cv::Point3d _position;                                       // 3D position of the map point
    std::map<std::shared_ptr<Frame>, uint64_t> _observers;       // Frames associated with this map point

    cv::Mat _descriptor;

    uint64_t _id;
    static uint64_t mapPointCount;

    std::mutex mtxDescriptor;
    std::mutex mtxObservation;
    mutable std::mutex mtxPosition;
};

