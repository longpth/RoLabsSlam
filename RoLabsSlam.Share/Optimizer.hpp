#pragma once
#include "Frame.hpp"
#include "MyTypes.hpp"

class Optimizer
{
public:
	static int PoseOptimization(std::shared_ptr<Frame> frame, const CameraInfo& camInfo);
	static void BundleAdjustment(std::shared_ptr<Frame> keyFrame, const CameraInfo& camInfo);
	static void BundleAdjustment2(std::vector<std::shared_ptr<Frame>>& frame, std::set<std::shared_ptr<MapPoint>>& mapPoints, const CameraInfo& camInfo, int windowSize = 10);
};