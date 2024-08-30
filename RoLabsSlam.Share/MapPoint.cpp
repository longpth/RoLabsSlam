#include "MapPoint.hpp"
#include "Frame.hpp"
#include "Helpers.hpp"

MapPoint::MapPoint(const cv::Point3d& position)
    : _position(position) {}

// Setters
void MapPoint::SetPosition(const cv::Point3d& position)
{
    _position = position;
}

// Getters
cv::Point3d MapPoint::GetPosition() const
{
    return _position;
}

// Add an observation
void MapPoint::AddObservation(std::shared_ptr<Frame> frame)
{
    _observers[frame->Id()] = frame;
}

// Remove an observation
void MapPoint::RemoveObservation(Frame* frame)
{
    _observers.erase(frame->Id());
}

cv::Mat MapPoint::GetDescriptor()
{
    return _descriptor.clone();
}

void MapPoint::SetDescriptor(cv::Mat descriptor)
{
    _descriptor = descriptor.clone();
}

std::map<uint64_t, std::shared_ptr<Frame>> MapPoint::GetObservations()
{
    return _observers;
}
