#include "Optimizer.hpp"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "Eigen/StdVector"
#include "Helpers.hpp"

cv::Mat g2oToCvMat(const g2o::SE3Quat& SE3quat) {
    // Extract the rotation matrix (3x3) from the SE3Quat
    Eigen::Matrix3d rotationMatrix = SE3quat.rotation().toRotationMatrix();

    // Extract the translation vector (3x1) from the SE3Quat
    Eigen::Vector3d translationVector = SE3quat.translation();

    // Convert the Eigen matrix and vector to OpenCV Mat
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2),
        rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2),
        rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2));

    cv::Mat t = (cv::Mat_<double>(3, 1) <<
        translationVector(0), translationVector(1), translationVector(2));

    // Combine R and t into a 4x4 transformation matrix
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_64F);  // Initialize as an identity matrix
    R.copyTo(Tcw(cv::Rect(0, 0, 3, 3)));       // Copy R into the top-left 3x3 part
    t.copyTo(Tcw(cv::Rect(3, 0, 1, 3)));       // Copy t into the top-right 3x1 part

    return Tcw;
}

// This function I refered from ORB-SLAM2 
int Optimizer::PoseOptimization(std::shared_ptr<Frame> frame, const CameraInfo& camInfo)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set the Frame vertex (camera pose)
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    cv::Mat Tcw = frame->Tcw();
    Eigen::Matrix<double, 3, 3> R;
    R << Tcw.at<double>(0, 0), Tcw.at<double>(0, 1), Tcw.at<double>(0, 2),
        Tcw.at<double>(1, 0), Tcw.at<double>(1, 1), Tcw.at<double>(1, 2),
        Tcw.at<double>(2, 0), Tcw.at<double>(2, 1), Tcw.at<double>(2, 2);
    Eigen::Matrix<double, 3, 1> t(Tcw.at<double>(0, 3), Tcw.at<double>(1, 3), Tcw.at<double>(2, 3));
    vSE3->setEstimate(g2o::SE3Quat(R, t));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    const int N = frame->KeyPoints().size();

    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    std::vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    //std::cout << "[Cpp][PoseOptimization] ptimizer.indexMapping().size = " << optimizer.indexMapping().size() << std::endl;

    const float deltaMono = sqrt(5.991);

    for (int i = 0; i < N; i++)
    {
        std::shared_ptr<MapPoint> p3D = frame->GetMapPoints()[i];
        const cv::KeyPoint& kp = frame->KeyPoints()[i];

        if (p3D) // Check if map point is valid
        {
            frame->Outliers()[i] = false;

            nInitialCorrespondences++;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kp.pt.x, kp.pt.y;

            g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());

            e->fx = camInfo.fx;
            e->fy = camInfo.fy;
            e->cx = camInfo.cx;
            e->cy = camInfo.cy;
            e->Xw[0] = (float)p3D->GetPosition().x;
            e->Xw[1] = (float)p3D->GetPosition().y;
            e->Xw[2] = (float)p3D->GetPosition().z;

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
        }
    }

    if (nInitialCorrespondences < 3)
        return 0;

    //std::cout << "[Cpp][PoseOptimization] ptimizer.indexMapping().size = " << optimizer.indexMapping().size() << std::endl;

    // Optimize the pose using multiple iterations, handling outliers
    const float chi2Mono[4] = { 5.991, 5.991, 5.991, 5.991 };
    const int its[4] = { 10, 10, 10, 10 };

    int nBad = 0;
    for (size_t it = 0; it < 4; it++)
    {
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        optimizer.initializeOptimization();
        //std::cout << "[Cpp][PoseOptimization] ptimizer.indexMapping().size = " << optimizer.indexMapping().size() << std::endl;
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];
            const size_t idx = vnIndexEdgeMono[i];

            if (frame->Outliers()[i]) // Check if it's not an outlier
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
                frame->Outliers()[i] = true; // Mark as outlier
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                frame->Outliers()[i] = false; // Mark as inlier
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    Eigen::Matrix4d optimizedPose = SE3quat_recov.to_homogeneous_matrix();

    cv::Mat optimizedPoseCV(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            optimizedPoseCV.at<double>(i, j) = optimizedPose(i, j);

    frame->SetTcw(optimizedPoseCV);

    return nInitialCorrespondences - nBad;
}

void Optimizer::BundleAdjustment2(std::vector<std::shared_ptr<Frame>>& frames, std::set<std::shared_ptr<MapPoint>>& mapPoints, const CameraInfo& camInfo, int windowSize)
{
    assert(frames.size() > 0);

    int localFrameSize = std::min((int)frames.size(), windowSize);
    int lastIndex = frames.size() - 1;
    int lastLocalFrameId = frames[lastIndex]->Id();
    int firstLocalFrameId = frames[lastIndex]->Id() - localFrameSize + 1;
    int checkedFrameSize = std::min((int)frames.size(), windowSize * 2);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    unsigned long maxKFid = 0;

    std::vector<std::shared_ptr<Frame>>::iterator lit = (frames.end() - 1) - (checkedFrameSize - 1);

    std::vector<std::shared_ptr<Frame>>::iterator lend = frames.end();

    // Set frame vertices
    for (; lit != lend; lit++)
    {
        Frame* pKFi = lit->get();
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = pKFi->Tcw();
        Eigen::Matrix<double, 3, 3> R;
        R << Tcw.at<double>(0, 0), Tcw.at<double>(0, 1), Tcw.at<double>(0, 2),
            Tcw.at<double>(1, 0), Tcw.at<double>(1, 1), Tcw.at<double>(1, 2),
            Tcw.at<double>(2, 0), Tcw.at<double>(2, 1), Tcw.at<double>(2, 2);
        Eigen::Matrix<double, 3, 1> t(Tcw.at<double>(0, 3), Tcw.at<double>(1, 3), Tcw.at<double>(2, 3));
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        vSE3->setId(pKFi->Id());
        vSE3->setFixed(pKFi->Id() <= 1 || pKFi->Id() < firstLocalFrameId);
        optimizer.addVertex(vSE3);
        //std::cout << "[Cpp] localKeyFrame id = " << pKFi->Id() << "\n";
        if (pKFi->Id() > maxKFid)
            maxKFid = pKFi->Id();
    }

    // Set Map point vertices
    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    std::vector<Frame*> vpEdgeKFMono;
    std::vector<MapPoint*> vpMapPointEdgeMono;

    int edgeCount = 0;
    std::map <int, int> mpVertexIds;

    for (std::set<std::shared_ptr<MapPoint>>::iterator lit = mapPoints.begin(), lend = mapPoints.end(); lit != lend; lit++)
    {
        MapPoint* pMP = lit->get();
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3d pointEstimate(pMP->GetPosition().x, pMP->GetPosition().y, pMP->GetPosition().z);
        vPoint->setEstimate(pointEstimate);
        int id = pMP->Id() + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        mpVertexIds[id]++;

        const std::map<std::shared_ptr<Frame>, uint64_t> observations = pMP->GetObservations();
        const float thHuberMono = sqrt(5.991);

        // Set edges
        for (std::map<std::shared_ptr<Frame>, uint64_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            //std::cout << "[Cpp] Edge count " << edgeCount << std::endl;
            edgeCount++;
            Frame* pKFi = mit->first.get();

            int frameId = pKFi->Id();

            if (frameId <= maxKFid - checkedFrameSize)
                continue;

            //std::cout << "[Cpp] BA Added frame id as edge " << frameId << std::endl;
            //std::cout << "[Cpp] maxKFid = " << maxKFid << std::endl;
            //std::cout << "[Cpp] checkedFrameSize = " << checkedFrameSize << std::endl;

            const cv::KeyPoint& kp = pKFi->KeyPoints()[mit->second];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kp.pt.x, kp.pt.y;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->Id())));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(thHuberMono);

            e->setRobustKernel(rk);

            e->fx = camInfo.fx;
            e->fy = camInfo.fy;
            e->cx = camInfo.cx;
            e->cy = camInfo.cy;

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];

        // this edge might be an outlier, so set level to 1 to exclude from from the main optimization process (level 0)
        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    std::vector<std::pair<Frame*, MapPoint*>> outliers;

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        // Prepare outliers for removing
        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            Frame* pKFi = vpEdgeKFMono[i];
            outliers.push_back(make_pair(pKFi, pMP));
        }
    }

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        // Prepare outliers for removing
        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            Frame* pKFi = vpEdgeKFMono[i];
            outliers.push_back(make_pair(pKFi, pMP));
        }
    }

    // Remove outlier
    for (int i = 0; i < outliers.size(); i++)
    {
        MapPoint* mp = outliers[i].second;
        Frame* kframe = outliers[i].first;

        //std::cout << "[Cpp] Remove map point outlier " << mp->Id() << std::endl;
        //std::cout << "[Cpp] Remove key frame outlier " << kframe->Id() << std::endl;

        kframe->RemoveMapPoint(mp);
        mp->RemoveObservation(kframe);
    }

    std::cout << "[Cpp] Bundle Adjustment remove " << outliers.size() << " outliers " << std::endl;

    // Recover optimized data

    std::cout << "[Cpp] Bundle Adjustment Map point size = " << mapPoints.size() << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";

    // Frames update
    for (std::vector<std::shared_ptr<Frame>>::iterator lit = frames.begin() + firstLocalFrameId, lend = frames.begin() + lastLocalFrameId; lit != lend; lit++)
    {
        Frame* pKF = lit->get();
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->Id()));
        g2o::SE3Quat SE3quat = vSE3->estimate();

        std::cout << "[Cpp] Update " << pKF->Id() << g2oToCvMat(SE3quat) << "\n";

        pKF->SetTcw(g2oToCvMat(SE3quat));
    }

    // Points update
    for (std::set<std::shared_ptr<MapPoint>>::iterator lit = mapPoints.begin(), lend = mapPoints.end(); lit != lend; lit++)
    {
        std::shared_ptr<MapPoint> pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->Id() + maxKFid + 1));
        Eigen::Vector3d position = vPoint->estimate();
        pMP->SetPosition(cv::Point3d(position[0], position[1], position[2]));

        bool old = false;

        // remove old points from local map if the reproject error is large or far from the current frame
        std::map<std::shared_ptr<Frame>, uint64> frames = pMP->GetObservations();

        int biggestId = 0;
        float error = 0;

        for (const auto& pair : frames) {
            biggestId = biggestId < pair.first->Id() ? pair.first->Id() : biggestId;
            Frame* frame = pair.first.get();
            cv::Point2f point2d = frame->KeyPoints()[pair.second].pt;
            cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) <<
                camInfo.fx, 0, camInfo.cx,  // fx, 0, cx
                0, camInfo.fy, camInfo.cy,  // 0, fy, cy
                0, 0, 1);                   // 0, 0, 1
            error += Reprojection(pMP->GetPosition(), point2d, frame->Tcw(), cameraMatrix);
        }

        error /= frames.size();
        bool oldPoint = frames.size() <= 3 && biggestId < frames.size() - 7;

        if (oldPoint || error >= 0.5)
        {
            old = true;
            mapPoints.erase(pMP);
        }
    }
    std::cout << "[Cpp] Bundle Adjustment mapPoints size = " << mapPoints.size() << " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";

}

void Optimizer::BundleAdjustment(std::shared_ptr<Frame> keyFrame, const CameraInfo& camInfo)
{

    // Get Local Key Frame
    std::vector<std::shared_ptr<Frame>> localKeyFrames = keyFrame->GetLocalKeyFrames();

    // Get Local Map
    std::vector<std::shared_ptr<MapPoint>> localMapPoints;
    for (auto mp : keyFrame->GetMapPoints())
    {
        if (mp) {
            localMapPoints.push_back(mp);
        }
    }

    // Get Fixed Frame
    std::vector<std::shared_ptr<Frame>> fixedKeyFrames = keyFrame->GetFixedFrames();

    std::cout << "[Cpp] BundleAdjustment localKeyFrames " << localKeyFrames.size() << " "
              << "fixedKeyFrames " << fixedKeyFrames.size() << " "
              << "localMapPoints " << localMapPoints.size() << std::endl;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (std::vector<std::shared_ptr<Frame>>::iterator lit = localKeyFrames.begin(), lend = localKeyFrames.end(); lit != lend; lit++)
    {
        Frame* pKFi = lit->get();
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = pKFi->Tcw();
        Eigen::Matrix<double, 3, 3> R;
        R << Tcw.at<double>(0, 0), Tcw.at<double>(0, 1), Tcw.at<double>(0, 2),
            Tcw.at<double>(1, 0), Tcw.at<double>(1, 1), Tcw.at<double>(1, 2),
            Tcw.at<double>(2, 0), Tcw.at<double>(2, 1), Tcw.at<double>(2, 2);
        Eigen::Matrix<double, 3, 1> t(Tcw.at<double>(0, 3), Tcw.at<double>(1, 3), Tcw.at<double>(2, 3));
        vSE3->setEstimate(g2o::SE3Quat(R,t));
        vSE3->setId(pKFi->Id());
        vSE3->setFixed(pKFi->Id() <= 1);
        optimizer.addVertex(vSE3);
        std::cout << "[Cpp] localKeyFrame id = " << pKFi->Id() << "\n";
        if (pKFi->Id() > maxKFid)
            maxKFid = pKFi->Id();
    }

    // Set Fixed KeyFrame vertices
    for (std::vector<std::shared_ptr<Frame>>::iterator lit = fixedKeyFrames.begin(), lend = fixedKeyFrames.end(); lit != lend; lit++)
    {
        Frame* pKFi = lit->get();
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = pKFi->Tcw();
        Eigen::Matrix<double, 3, 3> R;
        R << Tcw.at<double>(0, 0), Tcw.at<double>(0, 1), Tcw.at<double>(0, 2),
            Tcw.at<double>(1, 0), Tcw.at<double>(1, 1), Tcw.at<double>(1, 2),
            Tcw.at<double>(2, 0), Tcw.at<double>(2, 1), Tcw.at<double>(2, 2);
        Eigen::Matrix<double, 3, 1> t(Tcw.at<double>(0, 3), Tcw.at<double>(1, 3), Tcw.at<double>(2, 3));
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        vSE3->setId(pKFi->Id());
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        std::cout << "[Cpp] fixedKeyFrames id = " << pKFi->Id() << "\n";
        if (pKFi->Id() > maxKFid)
            maxKFid = pKFi->Id();
    }

    std::cout << "[Cpp] localMapPoints size " << localMapPoints.size() << std::endl;
    std::cout << "[Cpp] Max KF id " << maxKFid << std::endl;

    std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    std::vector<Frame*> vpEdgeKFMono;
    std::vector<MapPoint*> vpMapPointEdgeMono;

    int edgeCount = 0;
    std::map <int, int> mpVertexIds;

    for (std::vector<std::shared_ptr<MapPoint>>::iterator lit = localMapPoints.begin(), lend = localMapPoints.end(); lit != lend; lit++)
    {
        MapPoint* pMP = lit->get();
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3d pointEstimate(pMP->GetPosition().x, pMP->GetPosition().y, pMP->GetPosition().z);
        vPoint->setEstimate(pointEstimate);
        int id = pMP->Id() + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        mpVertexIds[id]++;

        const std::map<std::shared_ptr<Frame>, uint64_t> observations = pMP->GetObservations();
        const float thHuberMono = sqrt(5.991);

        // Set edges
        for (std::map<std::shared_ptr<Frame>, uint64_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            //std::cout << "[Cpp] Edge count " << edgeCount << std::endl;
            edgeCount++;
            Frame* pKFi = mit->first.get();

            const cv::KeyPoint& kp = pKFi->KeyPoints()[mit->second];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kp.pt.x, kp.pt.y;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->Id())));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            e->fx = camInfo.fx;
            e->fy = camInfo.fy;
            e->cx = camInfo.cx;
            e->cy = camInfo.cy;

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
        }
    }

    // Iterate through the map to check values
    for (const auto& pair : mpVertexIds) {
        if (pair.second > 1) {
            std::cout << " Map vertex id same " << pair.first << "  " << pair.second << "\n";
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];

        // this edge might be an outlier, so set level to 1 to exclude from from the main optimization process (level 0)
        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    std::vector<std::pair<Frame*, MapPoint*>> outliers;

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        // Prepare outliers for removing
        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            Frame* pKFi = vpEdgeKFMono[i];
            outliers.push_back(make_pair(pKFi, pMP));
        }
    }

    // Remove outlier
    for (int i = 0; i < outliers.size(); i++)
    {
        MapPoint* mp = outliers[i].second;
        Frame* kframe = outliers[i].first;

        //std::cout << "[Cpp] Remove map point outlier " << mp->Id() << std::endl;
        //std::cout << "[Cpp] Remove key frame outlier " << kframe->Id() << std::endl;

        kframe->RemoveMapPoint(mp);
        mp->RemoveObservation(kframe);
    }

    std::cout << "[Cpp] Bundle Adjustment remove " << outliers.size() << " outliers " << std::endl;

    // Recover optimized data

    std::cout << "[Cpp] Bundle Adjustment >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";

    // Keyframes
    for (std::vector<std::shared_ptr<Frame>>::iterator lit = localKeyFrames.begin(), lend = localKeyFrames.end(); lit != lend; lit++)
    {
        Frame* pKF = lit->get();
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->Id()));
        g2o::SE3Quat SE3quat = vSE3->estimate();

        std::cout << "[Cpp] Update " << pKF->Id() << g2oToCvMat(SE3quat) << "\n";

        pKF->SetTcw(g2oToCvMat(SE3quat));
    }

    // Points
    for (std::vector<std::shared_ptr<MapPoint>>::iterator lit = localMapPoints.begin(), lend = localMapPoints.end(); lit != lend; lit++)
    {
        MapPoint* pMP = lit->get();
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->Id() + maxKFid + 1));
        Eigen::Vector3d position = vPoint->estimate();
        pMP->SetPosition(cv::Point3d(position[0], position[1], position[2]));
    }
    std::cout << "[Cpp] Bundle Adjustment <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
}