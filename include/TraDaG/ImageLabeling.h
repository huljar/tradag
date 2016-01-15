#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <array>
#include <random>

namespace TraDaG {
    class ImageLabeling;
}

class TraDaG::ImageLabeling
{
public:
    ImageLabeling(const cv::Mat& depthImage, const cv::Mat& labelImage, const LabelMap& labelMap, const CameraManager& cameraParams);
    virtual ~ImageLabeling();

    std::vector<unsigned short> findValidLabelValues(const std::string& label);

    virtual PlaneFitStatus computePlaneForLabel(const std::string& label, GroundPlane& result);

    cv::Mat getDepthImage() const;
    cv::Mat getLabelImage() const;
    LabelMap getLabelMap() const;
    CameraManager getCameraManager() const;

protected:
    typedef std::map<cv::Vec2i, Ogre::Vector3, bool(*)(const cv::Vec2i&, const cv::Vec2i&)> InlierMap;

    cv::Mat mDepthImage;
    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    CameraManager mCameraManager;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGELABELING_H
