#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/rgbdobject.h>
#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <array>

namespace TraDaG {
    class ImageLabeling;
}

class TraDaG::ImageLabeling
{
public:
    ImageLabeling(const cv::Mat& labelImage, const LabelMap& labelMap, LabelMode labelMode);
    virtual ~ImageLabeling();

    virtual PlaneFittingResult getPlaneForLabel(const std::string& label, const RgbdObject* scene) const;

    virtual cv::Mat getLabelImage();
    virtual const cv::Mat getLabelImage() const;

    virtual LabelMap getLabelMap() const;
    virtual void setLabelMap(const LabelMap& labelMap);

    virtual LabelMode getLabelMode() const;
    virtual void setLabelMode(LabelMode mode);

    // model construction and evaluation functions for RANSAC
    static Ogre::Plane createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points);
    static float pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane);

protected:
    typedef std::map<cv::Vec2i, Ogre::Vector3, bool(*)(const cv::Vec2i&, const cv::Vec2i&)> InlierMap;

    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    LabelMode mLabelMode;
};

#endif // IMAGELABELING_H
