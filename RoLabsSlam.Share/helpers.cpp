
#include "Helpers.hpp"
#include "MapPoint.hpp"

const int ORB_HARMING_DISTANCE_THRESHOLD = 100;

std::vector<cv::Point3d> MyTriangulatePoints(
    const cv::Mat& P1, const cv::Mat& P2,
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2);

// Function to decompose essential matrix and triangulate points
void FindRtAndTriangulate(
    const cv::Mat& essential_matrix,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::DMatch>& good_matches,
    std::shared_ptr<Frame> currentFrame,
    std::shared_ptr<Frame> previousFrame
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

    // Triangulate points
    std::vector<cv::Point3d> point3Ds = MyTriangulatePoints(P1, P0, currentPoints, previousPoints);

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
        mapPoint->AddObservation(previousFrame);
        mapPoint->AddObservation(currentFrame);
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

    // Update the outliers in the current frame
    currentFrame->Outliers().clear(); // Clear previous outliers
    for (size_t i = 0; i < validKeyPointCurrent.size(); ++i)
    {
        // Mark inliers as non-outliers (-1 or some similar value)
        currentFrame->Outliers().push_back(false);
    }

    cv::Mat transformation = CreateTransformationMatrix(R, t);

    // Update the current frame's transformation matrix (camera to world)
    currentFrame->SetTcw(transformation);
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
    const std::vector<cv::Point2f>& points2)
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

        points3D.emplace_back(X.at<double>(0), X.at<double>(1), X.at<double>(2));
    }

    return points3D;
}

int SearchByProjection(std::shared_ptr<Frame> currentFrame, const std::set<std::shared_ptr<MapPoint>>& mapPoints, const cv::Mat& intrinsicCameraMatrix, int searchRadius, std::vector<bool>& mask)
{
    mask.resize(currentFrame->KeyPoints().size(), false);
    cv::Matx33d K = intrinsicCameraMatrix;
    cv::Matx44d Tcw = currentFrame->Tcw(); // 4x4 transformation matrix
    int trackedPtsCnt = 0;

    std::vector<cv::KeyPoint> currentFrameKps = currentFrame->KeyPoints();
    cv::Mat currentFrameDesc = currentFrame->Descriptors(); // Assuming Descriptors() returns a cv::Mat

    int width = currentFrame->Width(); // Assuming you have a method to get frame width
    int height = currentFrame->Height(); // Assuming you have a method to get frame height

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
            currentFrame->GetMapPoints()[bestIndex] = mp_ptr; // Store map point in current frame
            mp_ptr->AddObservation(currentFrame); // Add observer
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
