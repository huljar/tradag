#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/PlaneInfo.h>
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
    typedef std::map<cv::Point, std::pair<Ogre::Vector3, int>, bool(*)(const cv::Point&, const cv::Point&)> PixelWorldMap;

    ImageLabeling(const cv::Mat& depthImage, const cv::Mat& labelImage, const LabelMap& labelMap, const CameraManager& cameraParams);
    virtual ~ImageLabeling();

    bool containsLabel(const std::string& label) const;

    LabelVec findValidLabelValues(const std::string& label) const;

    virtual PlaneFitStatus findPlaneForLabel(const std::string& label, GroundPlane& result,
                                             const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0,
                                             unsigned short minDistance = 0,
                                             unsigned short maxDistance = std::numeric_limits<unsigned short>::max());

    virtual PlaneFitStatus findPlaneForLabel(const std::string& label, PlaneInfo& result,
                                             const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    cv::Mat getDepthImage() const;
    cv::Mat getLabelImage() const;
    LabelMap getLabelMap() const;
    CameraManager getCameraManager() const;

protected:
    std::vector<unsigned int> doRegionGrowing(PixelWorldMap& inliers) const;

    cv::Mat mDepthImage;
    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    CameraManager mCameraManager;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGELABELING_H
