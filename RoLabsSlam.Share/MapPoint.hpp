#pragma once

#include <opencv2/core.hpp>
#include <map>
#include <memory>

class Frame;

class MapPoint
{
public:
    MapPoint() {
        // Constructor implementation (if needed)
    }

    // Constructors
    MapPoint(const cv::Point3d& position);

    // Getters
    cv::Point3d GetPosition() const;
    // Setters
    void SetPosition(const cv::Point3d& position);

    // Add an observation
    void AddObservation(std::shared_ptr<Frame> frame);

    // Get Observations
    std::map<uint64_t, std::shared_ptr<Frame>> GetObservations();

    // Remove an observation
    void RemoveObservation(Frame* frame);

    // Get the map point descriptor
    cv::Mat GetDescriptor();

    void SetDescriptor(cv::Mat descriptor);

private:
    cv::Point3d _position;                                       // 3D position of the map point
    std::map<uint64_t, std::shared_ptr<Frame>> _observers;       // Frames associated with this map point

    cv::Mat _descriptor;
};

