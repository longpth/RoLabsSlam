#include "Frame.hpp"
#include "Helpers.hpp"

uint64_t Frame::frameCount = 0;

Frame::Frame(const cv::Mat& image) : _orb(cv::ORB::create()) {
    detectAndCompute2(image);
    // Define the Twc matrix at the origin (identity matrix)
    _Tcw = cv::Mat::eye(4, 4, CV_64F); // 4x4 identity matrix
    _id = frameCount++;

    _width = image.cols;
    _height = image.rows;

}

void Frame::detectAndCompute(const cv::Mat& image) {
    // Minimum size of each cell
    const int minCellSize = 150;

    // Calculate the number of rows and columns to ensure cell size is at least minCellSize
    int rows = std::max(1, image.rows / minCellSize);  // Ensure at least 1 row
    int cols = std::max(1, image.cols / minCellSize);  // Ensure at least 1 column

    // Calculate the size of each cell
    int cellWidth = image.cols / cols;
    int cellHeight = image.rows / rows;

    // Temporary containers for keypoints and descriptors
    std::vector<cv::KeyPoint> allKeypoints;
    cv::Mat allDescriptors;

    // Iterate over each cell
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            // Define the region of interest (ROI) for the current cell
            cv::Rect roi(c * cellWidth, r * cellHeight, cellWidth, cellHeight);
            cv::Mat cellImage = image(roi);

            // Detect and compute keypoints and descriptors for the current cell
            std::vector<cv::KeyPoint> cellKeypoints;
            cv::Mat cellDescriptors;
            _orb->detectAndCompute(cellImage, cv::noArray(), cellKeypoints, cellDescriptors);

            // Offset keypoints to the coordinate system of the full image
            for (auto& kp : cellKeypoints) {
                kp.pt.x += c * cellWidth;
                kp.pt.y += r * cellHeight;
            }

            // Append the cell keypoints and descriptors to the full list
            allKeypoints.insert(allKeypoints.end(), cellKeypoints.begin(), cellKeypoints.end());

            if (!cellDescriptors.empty()) {
                allDescriptors.push_back(cellDescriptors);
            }
        }
    }

    // Set the class member variables to the merged results
    _keypoints = allKeypoints;
    _descriptors = allDescriptors;
}

void Frame::detectAndCompute2(const cv::Mat& image) {

    // Parameters for goodFeaturesToTrack
    int maxCorners = 2000; // Maximum number of good features to retain per cell
    double qualityLevel = 0.01; // Quality level for good features
    double minDistance = 10.0; // Minimum distance between corners
    int blockSize = 3; // Block size for corner detection
    bool useHarrisDetector = false; // Use Harris or Shi-Tomasi corner detector
    double k = 0.04; // Free parameter for Harris detector

    // Detect good features to track in the cell
    std::vector<cv::Point2f> goodFeatures;
    cv::goodFeaturesToTrack(image, goodFeatures, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

    // Convert good features to keypoints
    std::vector<cv::KeyPoint> goodKeypoints;
    cv::KeyPoint::convert(goodFeatures, goodKeypoints);

    // Detect and compute descriptors for the good keypoints in the current cell
    cv::Mat goodFeaturesDescriptors;
    _orb->compute(image, goodKeypoints, goodFeaturesDescriptors);

    // Set the class member variables to the merged results
    _keypoints = goodKeypoints;
    _descriptors = goodFeaturesDescriptors;
}

void Frame::UpdateLocalKeyFrames()
{
    std::unique_lock<std::mutex> lock(mtxFrames); // Lock the mutex

    std::map<std::shared_ptr<Frame>, uint64_t> keyFrames; // map<Frame*, count>

    for (const auto mp : _mapPoints)
    {
        if (!mp)
            continue;
        for (const auto& pair : mp->GetObservations()) {
            if (pair.second != -1) // valid observation
            {
                keyFrames[pair.first]++;
            }
        }
    }

    for (const auto& pair : keyFrames)
    {
        if (pair.second > 20) // if the key frame has more than 20 points observed by this frame, add it to local key frames, else add it as fixed frames, used for bundle adjustment
        {
            _localKeyFrames.push_back(pair.first);
        }
        else
        {
            _fixedKeyFrames.push_back(pair.first);
        }
    }

}

void Frame::RemoveMapPoint(MapPoint* mapPoint)
{
    std::unique_lock<std::mutex> lock(mtxMapPoints);
    // Use std::find_if to search for the MapPoint with the specific ID
    auto it = std::find_if(_mapPoints.begin(), _mapPoints.end(),
        [mapPoint](const std::shared_ptr<MapPoint>& mp) {
            return (mp && mp->Id() == mapPoint->Id());
        });

    if (it != _mapPoints.end()) {
        *it = nullptr;
        //std::cout << "[Cpp] Remove map point id = " << mapPoint->Id() << " from frame " << this->Id() << std::endl;
    }
}