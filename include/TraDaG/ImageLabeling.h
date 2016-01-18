#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

#include <limits>
#include <random>
#include <string>

namespace TraDaG {
    class ImageLabeling;
}

class TraDaG::ImageLabeling
{
public:
    ImageLabeling(const cv::Mat& depthImage, const cv::Mat& labelImage, const LabelMap& labelMap, const CameraManager& cameraParams);
    virtual ~ImageLabeling();

    bool containsLabel(const std::string& label);

    LabelVec findValidLabelValues(const std::string& label);

    virtual PlaneFitStatus findPlaneForLabel(const std::string& label, GroundPlane& result,
                                             const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0,
                                             unsigned short minDistance = 0,
                                             unsigned short maxDistance = std::numeric_limits<unsigned short>::max());

    cv::Mat getDepthImage() const;
    cv::Mat getLabelImage() const;
    LabelMap getLabelMap() const;
    CameraManager getCameraManager() const;

protected:
    cv::Mat mDepthImage;
    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    CameraManager mCameraManager;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGELABELING_H
