#include "MapPoint.hpp"
#include "Frame.hpp"
#include "Helpers.hpp"

uint64_t MapPoint::mapPointCount = 0;

MapPoint::MapPoint()
{
    _id = mapPointCount++;
}

// Setters
void MapPoint::SetPosition(const cv::Point3d& position)
{
    std::unique_lock lock(mtxPosition);
    _position = position;
}

// Getters
cv::Point3d MapPoint::GetPosition() const
{
    std::unique_lock lock(mtxPosition);
    return _position;
}

// Add an observation
void MapPoint::AddObservation(std::shared_ptr<Frame> frame, int keyPointIdex)
{
    std::unique_lock lock(mtxObservation);
    _observers[frame] = keyPointIdex;
}

// Remove an observation
void MapPoint::RemoveObservation(Frame* frame)
{
    std::unique_lock lock(mtxObservation);
    for (auto it = _observers.begin(); it != _observers.end(); ++it) {
        if (it->first->Id() == frame->Id()) {
            //std::cout << "[Cpp] remove observer frame id = " << frame->Id() << " from map point " << this->Id() << "\n";
            _observers.erase(it);
            break;
        }
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    std::unique_lock lock(mtxDescriptor);
    return _descriptor.clone();
}

void MapPoint::SetDescriptor(cv::Mat descriptor)
{
    std::unique_lock lock(mtxDescriptor);
    _descriptor = descriptor.clone();
}

std::map<std::shared_ptr<Frame>, uint64_t> MapPoint::GetObservations()
{
    std::unique_lock lock(mtxObservation);
    return _observers;
}
