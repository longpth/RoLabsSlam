#include "SlamExport.hpp"
#include "Slam.hpp"

CVAPI(Slam*) Slam_create() {
    return new Slam();
}

CVAPI(void) Slam_destroy(Slam* slam) {
    delete slam;
}

CVAPI(void) Slam_grabImage(Slam* slam, cv::Mat* image) {
    if (slam && image) {
        slam->GrabImage(*image);
    }
}

CVAPI(void) Slam_getDebugKeyPoints(Slam* slam, std::vector<cv::KeyPoint>* keypointsCurrent, std::vector<cv::KeyPoint>* keypointsPrevious) {
    if (slam) {
        slam->GetDebugKeyPoints(keypointsCurrent, keypointsPrevious);
    }
}

CVAPI(void) Slam_stop(Slam* slam) {
    if (slam) {
        slam->Stop();
    }
}

CVAPI(void) Slam_start(Slam * slam) {
    if (slam) {
        slam->Start();
    }
}

CVAPI(void) Slam_setIntrinsicsMatrix(Slam* slam, float fx, float fy, float cx, float cy) {
    if (slam) {
        slam->SetCameraInfo(fx, fy, cx, cy);
    }
}

CVAPI(void) Slam_getCurrentPose(Slam* slam, cv::Mat* pose) {
    if (slam && pose)
    {
        slam->GetCurrentFramePose(pose);
    }
}

CVAPI(void) Slam_track(Slam* slam)
{
    if (slam)
    {
        slam->Track();
    }
}

CVAPI(void) Slam_getMapPoints(Slam* slam, std::vector<cv::Point3f>* mapPoint)
{
    if (slam)
    {
        slam->GetMapPoints(mapPoint);
    }
}