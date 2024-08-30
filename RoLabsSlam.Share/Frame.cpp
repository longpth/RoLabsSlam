#include "Frame.hpp"

uint64_t Frame::frameCount = 0;

Frame::Frame(const cv::Mat& image) : _orb(cv::ORB::create()) {
    detectAndCompute(image);
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
