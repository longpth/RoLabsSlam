
#include "Helpers.hpp"
#include "MapPoint.hpp"

const int ORB_HARMING_DISTANCE_THRESHOLD = 100;

// Function to decompose essential matrix and triangulate points
void FindRtAndTriangulate(
    const cv::Mat& essential_matrix,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::DMatch>& good_matches,
    std::shared_ptr<Frame> currentFrame,
    std::shared_ptr<Frame> previousFrame,
    double& invMedianDepth
)
{
    const std::vector<cv::KeyPoint>& previousFrameKeypoints = previousFrame->KeyPoints(); // Keypoints from previousFrame
    const std::vector<cv::KeyPoint>& currentFrameKeypoints = currentFrame->KeyPoints();  // Keypoints from currentFrame
    const cv::Mat& currentFrameDescriptors = currentFrame->Descriptors();  // Descriptors from currentFrame
    const cv::Mat& previousFrameDescriptors = previousFrame->Descriptors(); // Descriptors from previousFrame

    std::vector<cv::KeyPoint> inlierPreviousFrameKeypoints; // Output inliers for previousFrame
    std::vector<cv::KeyPoint> inlierCurrentFrameKeypoints;  // Output inliers for currentFrame
    cv::Mat validPreviousDescriptors, validCurrentDescriptors; // Output valid descriptors

    // Decompose the essential matrix into R and t
    cv::Mat R, t;
    cv::Mat mask; // mask to distinguish between inliers and outliers
    std::vector<cv::Point2f> points1, points2;

    for (const auto& match : good_matches)
    {
        // Associate keypoints from the matches
        points1.push_back(currentFrameKeypoints[match.queryIdx].pt);
        points2.push_back(previousFrameKeypoints[match.trainIdx].pt);
    }

    // Recover pose
    int inliersCount = cv::recoverPose(essential_matrix, points2, points1, cameraMatrix, R, t, mask); // five-point.cpp opencv

    std::cout << "[Cpp] Initialization R =" << R << " t = " << t << " inliersCount = " << inliersCount << " = " << (float) inliersCount / (float) points1.size() << "\n";

    // Filter inliers and prepare for triangulation
    inlierPreviousFrameKeypoints.clear();
    inlierCurrentFrameKeypoints.clear();
    std::vector<cv::Point2f> currentPoints, previousPoints;

    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<uchar>(i))
        {
            // Keep the inliers
            inlierCurrentFrameKeypoints.push_back(currentFrameKeypoints[good_matches[i].queryIdx]);
            inlierPreviousFrameKeypoints.push_back(previousFrameKeypoints[good_matches[i].trainIdx]);
            currentPoints.push_back(points1[i]);
            previousPoints.push_back(points2[i]);

            // Keep the descriptors of inliers
            validCurrentDescriptors.push_back(currentFrameDescriptors.row(good_matches[i].queryIdx));
            validPreviousDescriptors.push_back(previousFrameDescriptors.row(good_matches[i].trainIdx));
        }
    }

    // Projection matrices for the two views
    cv::Mat P0 = cameraMatrix * cv::Mat::eye(3, 4, CV_64F); // [I | 0]
    cv::Mat P1(3, 4, CV_64F);
    R.copyTo(P1(cv::Rect(0, 0, 3, 3))); // Copy R into the first 3x3 part of P1
    t.copyTo(P1(cv::Rect(3, 0, 1, 3))); // Copy t into the last column of P1
    P1 = cameraMatrix * P1;

    std::vector<bool > mask3d;
    int goodParallaxCnt = 0;
    // Triangulate points
    std::vector<cv::Point3d> point3Ds = MyTriangulatePoints(P1, P0, currentPoints, previousPoints, mask3d, goodParallaxCnt);

    invMedianDepth = Normalize3DPoints(point3Ds);

    t *= invMedianDepth;

    std::vector<std::shared_ptr<MapPoint>>& current_map_points = currentFrame->GetMapPoints();
    std::vector<std::shared_ptr<MapPoint>>& previous_map_points = previousFrame->GetMapPoints();

    current_map_points.clear();
    std::vector<cv::KeyPoint> validKeyPointCurrent, validKeyPointPrevious;
    cv::Mat validCurrentDescriptorsFiltered, validPreviousDescriptorsFiltered;

    for (int i = 0; i < point3Ds.size(); i++)
    {
        //std::cout << "[Cpp] point3D = " << point3Ds[i] << std::endl;

        if (point3Ds[i].z < 0)
            continue; // Skip points with Z < 0

        std::shared_ptr<MapPoint> mapPoint = std::make_shared<MapPoint>();
        mapPoint->AddObservation(previousFrame, i);
        mapPoint->AddObservation(currentFrame, i);
        mapPoint->SetPosition(point3Ds[i]);
        mapPoint->SetDescriptor(validCurrentDescriptors.row(i));

        //std::cout << "[Cpp] mappoint descriptor " << mapPoint->GetDescriptor() << std::endl;

        current_map_points.push_back(mapPoint);
        previous_map_points.push_back(mapPoint);

        // Keep only the valid 2D points and corresponding descriptors
        validKeyPointCurrent.push_back(inlierCurrentFrameKeypoints[i]);
        validKeyPointPrevious.push_back(inlierPreviousFrameKeypoints[i]);

        validCurrentDescriptorsFiltered.push_back(validCurrentDescriptors.row(i));
        validPreviousDescriptorsFiltered.push_back(validPreviousDescriptors.row(i));
    }

    // Update the keypoints and descriptors in the current and previous frames with only inliers
    previousFrame->SetKeyPoints(validKeyPointPrevious);
    currentFrame->SetKeyPoints(validKeyPointCurrent);
    previousFrame->SetDescriptor(validPreviousDescriptorsFiltered);
    currentFrame->SetDescriptor(validCurrentDescriptorsFiltered);

    cv::Mat transformation = CreateTransformationMatrix(R, t);

    // Update the current frame's transformation matrix (camera to world)
    currentFrame->SetTcw(transformation);
}

double FindMedianDepth(std::vector<cv::Point3d>& point3Ds, cv::Mat Tcw)
{
    std::vector<double> depths;

    // Output the sorted points and their depths
    for (const auto& point : point3Ds) {
        // Convert the 3D point to homogeneous coordinates (4x1 vector)
        cv::Mat pointWorldHomogeneous = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);

        // Transform the point from world coordinates to camera coordinates using Tcw
        cv::Mat pointCameraHomogeneous = Tcw * pointWorldHomogeneous;

        // The depth (z value) is the third element of the resulting point in camera coordinates
        double depth = pointCameraHomogeneous.at<double>(2);

        if (depth >= 0) {
            depths.push_back(depth);
        }
    }

    if (depths.empty())
    {
        std::cerr << "No valid 3D points found for normalization." << std::endl;
        return -1.0; // Indicate an error with a special value
    }

    // Sort depths to find the median
    std::sort(depths.begin(), depths.end());
    double medianDepth = depths[depths.size() / 2];

    return medianDepth;
}

void Normalize3DPoints(std::vector<cv::Point3d>& point3Ds, double invMedianDepth)
{
    for (auto& point : point3Ds)
    {
        point.x *= invMedianDepth;
        point.y *= invMedianDepth;
        point.z *= invMedianDepth;
    }
}

double Normalize3DPoints(std::vector<cv::Point3d>& point3Ds)
{
    // Step 1: Compute the Median Depth of the Triangulated Points
    double medianDepth = FindMedianDepth(point3Ds);
    double invMedianDepth = 1.0 / medianDepth;
    //double invMedianDepth = 1.0;

    // Step 2: Normalize the 3D Points
    for (auto& point : point3Ds)
    {
        point.x *= invMedianDepth;
        point.y *= invMedianDepth;
        point.z *= invMedianDepth;
    }

    std::cout << "[Cpp] medianDepth = " << medianDepth << "\n";

    return invMedianDepth;
}

double Normalize3DPoints(std::vector<cv::Point3d>& point3Ds, std::vector<double>& medianDepthVector) {
    // Step 1: Compute the Median Depth of the Triangulated Points
    double medianDepth = FindMedianDepth(point3Ds);

    if (medianDepth != -1) {

        medianDepthVector.push_back(medianDepth);

        medianDepth = 0;
        for (auto med : medianDepthVector)
        {
            medianDepth += med;
        }
        medianDepth = medianDepth / medianDepthVector.size();
    }

    double invMedianDepth = 1.0 / medianDepth;

    // Step 2: Normalize the 3D Points
    for (auto& point : point3Ds)
    {
        point.x *= invMedianDepth;
        point.y *= invMedianDepth;
        point.z *= invMedianDepth;
    }

    std::cout << "[Cpp] medianDepth = " << medianDepth << "\n";

    return invMedianDepth;
}


cv::Mat CreateTransformationMatrix(const cv::Mat& R, const cv::Mat& t)
{
    // Ensure R is 3x3 and t is 3x1
    CV_Assert(R.rows == 3 && R.cols == 3);
    CV_Assert(t.rows == 3 && t.cols == 1);

    // Create a 4x4 identity matrix
    cv::Mat T = cv::Mat::eye(4, 4, R.type());

    // Copy the rotation matrix into the top-left 3x3 part of T
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));

    // Copy the translation std::vector into the top-right 3x1 part of T
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
}

std::vector<cv::DMatch> MatchKeyPoints(const Frame& frame1, const Frame& frame2) {
    // Use BFMatcher to match descriptors
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher.knnMatch(frame1.Descriptors(), frame2.Descriptors(), knnMatches, 2);

    std::vector<cv::DMatch> goodMatches;
    std::vector<int> idx1, idx2;
    std::set<int> idx1s, idx2s;

    for (const auto& matchPair : knnMatches) {
        if (matchPair[0].distance < 0.75f * matchPair[1].distance) {
            const cv::KeyPoint& p1 = frame1.KeyPoints()[matchPair[0].queryIdx];
            const cv::KeyPoint& p2 = frame2.KeyPoints()[matchPair[0].trainIdx];

            // Be within ORB distance 32
            if (matchPair[0].distance < 32) {
                // Check for unique matches
                if (idx1s.find(matchPair[0].queryIdx) == idx1s.end() &&
                    idx2s.find(matchPair[0].trainIdx) == idx2s.end()) {

                    idx1.push_back(matchPair[0].queryIdx);
                    idx2.push_back(matchPair[0].trainIdx);
                    idx1s.insert(matchPair[0].queryIdx);
                    idx2s.insert(matchPair[0].trainIdx);
                    goodMatches.push_back(matchPair[0]);
                }
            }
        }
    }

    // Ensure no duplicates (assert statements)
    assert(std::set<int>(idx1.begin(), idx1.end()).size() == idx1.size());
    assert(std::set<int>(idx2.begin(), idx2.end()).size() == idx2.size());

    return goodMatches;
}

cv::Mat EstimateEssentialMatrix(const Frame& frame1, const Frame& frame2, const std::vector<cv::DMatch>& good_matches, const cv::Mat& cameraMatrix) {
    // Extract the matched keypoints
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : good_matches) {
        points1.push_back(frame1.KeyPoints()[match.queryIdx].pt);
        points2.push_back(frame2.KeyPoints()[match.trainIdx].pt);
    }

    // Compute the Essential Matrix using RANSAC to handle outliers
    cv::Mat essential_matrix = cv::findEssentialMat(points2, points1, cameraMatrix, cv::RANSAC, 0.999, 1.0);
    return essential_matrix;
}

std::vector<cv::Point3d> MyTriangulatePoints(
    const cv::Mat& P1, const cv::Mat& P2,
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2,
    std::vector<bool>& mask,
    int& goodParralaxCnt)
{
    assert(points1.size() == points2.size()); // size of poth point vectors must equal

    std::vector<cv::Point3d> points3D;

    for (size_t i = 0; i < points1.size(); i++)
    {
        // Construct matrix A
        cv::Mat A(4, 4, CV_64F);
        A.row(0) = points1[i].x * P1.row(2) - P1.row(0);
        A.row(1) = points1[i].y * P1.row(2) - P1.row(1);
        A.row(2) = points2[i].x * P2.row(2) - P2.row(0);
        A.row(3) = points2[i].y * P2.row(2) - P2.row(1);

        // Perform SVD decomposition
        cv::Mat u, w, vt;
        cv::SVD::compute(A, w, u, vt);

        // The 3D point in homogeneous coordinates is the last column of V^T (or row of V in OpenCV)
        cv::Mat X = vt.row(3).t();

        // Convert homogeneous coordinates to 3D
        X /= X.at<double>(3);  // Normalize by W to get (X, Y, Z, 1)

        cv::Point3d pt3d = cv::Point3d(X.at<double>(0), X.at<double>(1), X.at<double>(2));

        points3D.emplace_back(X.at<double>(0), X.at<double>(1), X.at<double>(2));

        if (CalculateCosParallax(P1, P2, pt3d) < 0.9998)
        {
            mask.push_back(true);
            goodParralaxCnt++;
        }
        else
        {
            mask.push_back(false);
        }
    }

    return points3D;
}

// Function to compute optical flow and track keypoints
std::vector<cv::Point2f> TrackKeypointsOpticalFlow(const cv::Mat& prevImg, const cv::Mat& currImg, const std::vector<cv::Point2f>& currKeypoints, std::vector<bool>& mask) {
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

int SearchInLocalMap(std::shared_ptr<Frame> currentFrame,const std::set<std::shared_ptr<MapPoint>>& localMap)
{
    int trackedPtsCnt = 0;

    std::vector<cv::KeyPoint> currentFrameKps = currentFrame->KeyPoints();

    std::vector<std::shared_ptr<MapPoint>> currentFrameMap = currentFrame->GetMapPoints();
    if (currentFrameMap.size() == 0)
    {
        currentFrame->InitializeMapPoints();
    }

    cv::Mat currentFrameDesc = currentFrame->Descriptors(); // Assuming Descriptors() returns a cv::Mat

    int width = currentFrame->Width();
    int height = currentFrame->Height();

    int outPointCnt = 0;
    int descriptorDistanceOutCnt = 0;

    std::vector<std::shared_ptr<MapPoint>> localMapVec;

    // Step 1: Initialize an empty matrix to hold all the descriptors
    cv::Mat localMapDescriptors;

    // Step 2: Iterate over the localMap and get the descriptors
    for (const auto& mp_ptr : localMap) {

        cv::Mat descriptor = mp_ptr->GetDescriptor();

        if (descriptor.empty()) {
            continue;  // Skip map points with empty descriptors
        }

        // Step 3: Append the descriptor to the localMapDescriptors matrix
        if (localMapDescriptors.empty()) {
            // If localMapDescriptors is empty, just assign the first descriptor
            localMapDescriptors = descriptor.clone();
        }
        else {
            // Append the current descriptor as a new row
            localMapDescriptors.push_back(descriptor);
        }

        localMapVec.push_back(mp_ptr);
    }

    // Use BFMatcher to match descriptors
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher.knnMatch(currentFrameDesc, localMapDescriptors, knnMatches, 2);

    std::vector<cv::DMatch> goodMatches;
    std::vector<int> idx1, idx2;
    std::set<int> idx1s, idx2s;

    for (const auto& matchPair : knnMatches) {
        if (matchPair[0].distance < 0.75f * matchPair[1].distance) {
            int currentFrameMapPointIndex = matchPair[0].queryIdx;
            int localMapPointIndex = matchPair[0].trainIdx;

            // Be within ORB distance 32
            if (matchPair[0].distance < 32) {
                if (!currentFrameMap[currentFrameMapPointIndex]) // not yet assigned a map point
                {
                    currentFrameMap[currentFrameMapPointIndex] = localMapVec[localMapPointIndex];
                    trackedPtsCnt++;
                }
            }
        }
    }

    //std::cout << "[Cpp] search by projection " << mapPoints.size() << " 2d out point count " << outPointCnt << std::endl;
    //std::cout << "[Cpp] search by projection " << mapPoints.size() << " descriptorDistanceOutCnt " << descriptorDistanceOutCnt << std::endl;

    return trackedPtsCnt;
}

int SearchByProjection(std::shared_ptr<Frame> currentFrame, std::shared_ptr<Frame> previousFrame, const cv::Mat& intrinsicCameraMatrix, int searchRadius, std::vector<bool>& mask)
{
    const std::vector<std::shared_ptr<MapPoint>>& mapPoints = previousFrame->GetMapPoints();
    mask.resize(currentFrame->KeyPoints().size(), false);
    cv::Matx33d K = intrinsicCameraMatrix;
    cv::Matx44d Tcw = currentFrame->Tcw(); // 4x4 transformation matrix
    int trackedPtsCnt = 0;

    std::vector<cv::KeyPoint> currentFrameKps = currentFrame->KeyPoints();
    cv::Mat currentFrameDesc = currentFrame->Descriptors(); // Assuming Descriptors() returns a cv::Mat

    int width = currentFrame->Width();
    int height = currentFrame->Height();

    int outPointCnt = 0;
    int descriptorDistanceOutCnt = 0;

    for (const auto& mp_ptr : mapPoints) {

        if (!mp_ptr) {
            continue; // Skip null map points
        }

        cv::Point3d pos = mp_ptr->GetPosition();
        cv::Matx41d posMat(pos.x, pos.y, pos.z, 1.0);
        cv::Matx41d posProj = Tcw * posMat;

        if (posProj(2) <= 0)
        {
            continue; // Skip points that are behind the camera
        }

        // Project to 2D
        cv::Mat point3dMat = (cv::Mat_<double>(3, 1) << posProj(0) / posProj(2), posProj(1) / posProj(2), 1.0);
        cv::Mat uvMat = K * point3dMat;
        cv::Point2d uv(uvMat.at<double>(0), uvMat.at<double>(1));

        // Ensure uv is within image bounds
        if (uv.x < 0 || uv.x >= width || uv.y < 0 || uv.y >= height) {
            outPointCnt++;
            continue; // Skip points outside image bounds
        }

        // search all the point within the radius

        std::vector<int> indicesWithinRadius;

        for (size_t j = 0; j < currentFrameKps.size(); ++j) {
            const cv::KeyPoint& kp = currentFrameKps[j];
            cv::Point2f kp_pt_float = kp.pt;
            cv::Point2d kp_pt_double(kp_pt_float.x, kp_pt_float.y);

            double distance = cv::norm(uv - kp_pt_double);

            // Add the index to the vector if the distance is within the search radius
            if (distance <= searchRadius) {
                indicesWithinRadius.push_back(j);
            }
        }

        cv::Mat mapPointDescriptor = mp_ptr->GetDescriptor(); // Retrieve the descriptor from MapPoint
        double bestDescriptorDistance = std::numeric_limits<double>::max();
        int bestIndex = -1;

        for (int idx : indicesWithinRadius) {
            // Retrieve the descriptor of the current keypoint
            cv::Mat queryDescriptor = currentFrameDesc.row(idx);

            // Calculate the Hamming distance between the current descriptor and the MapPoint descriptor
            double descriptorDistance = cv::norm(queryDescriptor, mapPointDescriptor, cv::NORM_HAMMING);

            // Check if this is the best (shortest) distance
            if (descriptorDistance < bestDescriptorDistance) {
                bestDescriptorDistance = descriptorDistance;
                bestIndex = idx;
            }
        }

        if (bestDescriptorDistance < ORB_HARMING_DISTANCE_THRESHOLD)
        {
            if (currentFrame->GetMapPoints()[bestIndex] == nullptr) {
                currentFrame->GetMapPoints()[bestIndex] = mp_ptr; // Store map point in current frame
            }
            //mp_ptr->AddObservation(currentFrame, bestIndex); // Add observer
            mask[bestIndex] = true; //mark that this index of keypoint has an associated map point
            trackedPtsCnt++;
        }
        else
        {
            descriptorDistanceOutCnt++;
        }
    }

    //std::cout << "[Cpp] search by projection " << mapPoints.size() << " 2d out point count " << outPointCnt << std::endl;
    //std::cout << "[Cpp] search by projection " << mapPoints.size() << " descriptorDistanceOutCnt " << descriptorDistanceOutCnt << std::endl;

    return trackedPtsCnt;
}

float Reprojection(const cv::Point3f& point3world, const cv::Point2f& point2f, const cv::Mat& Tcw, const cv::Mat& cameraMatrix) {
    // Step 1: Convert 3D world point to homogeneous coordinates (4x1 matrix)
    cv::Mat point3d_homogeneous = (cv::Mat_<double>(4, 1) << point3world.x, point3world.y, point3world.z, 1.0);

    // Step 2: Transform the 3D point from world coordinates to camera coordinates using Tcw
    cv::Mat point3d_camera = Tcw * point3d_homogeneous;

    // Step 3: Normalize the camera coordinates (i.e., divide by z to project to the image plane)
    float X = point3d_camera.at<double>(0);
    float Y = point3d_camera.at<double>(1);
    float Z = point3d_camera.at<double>(2);

    if (Z <= 0) {
        std::cerr << "Point is behind the camera, cannot project!" << std::endl;
        return -1.0f; // Invalid projection
    }

    // Step 4: Apply the camera intrinsic matrix (cameraMatrix) to project to 2D image coordinates
    cv::Mat projected_point2d_homogeneous = cameraMatrix * (cv::Mat_<float>(3, 1) << X / Z, Y / Z, 1.0);

    // Extract the projected 2D point
    float u_projected = projected_point2d_homogeneous.at<float>(0) / projected_point2d_homogeneous.at<float>(2);
    float v_projected = projected_point2d_homogeneous.at<float>(1) / projected_point2d_homogeneous.at<float>(2);

    // Step 5: Calculate the reprojection error as the Euclidean distance between the observed 2D point and the projected point
    float reprojection_error = sqrt(pow(point2f.x - u_projected, 2) + pow(point2f.y - v_projected, 2));

    return reprojection_error;
}

// Function to calculate the parallax between two rays in two views
double CalculateCosParallax(const cv::Mat& P1, const cv::Mat& P2, const cv::Point3d& point3D) {
    // Extract camera centers from projection matrices P1 and P2
    // The camera center is the null space of the projection matrix (P * C = 0)
    cv::Mat C1 = -P1.colRange(0, 3).inv() * P1.col(3);
    cv::Mat C2 = -P2.colRange(0, 3).inv() * P2.col(3);

    // Create the ray vectors from the camera centers to the 3D point
    cv::Mat ray1 = (cv::Mat_<double>(3, 1) << point3D.x - C1.at<double>(0), point3D.y - C1.at<double>(1), point3D.z - C1.at<double>(2));
    cv::Mat ray2 = (cv::Mat_<double>(3, 1) << point3D.x - C2.at<double>(0), point3D.y - C2.at<double>(1), point3D.z - C2.at<double>(2));

    // Normalize the rays (we only care about the direction)
    ray1 /= cv::norm(ray1);
    ray2 /= cv::norm(ray2);

    // Calculate the dot product between the two rays
    double cosTheta = ray1.dot(ray2);

    return cosTheta;
}