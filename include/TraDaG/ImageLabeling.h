#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/RGBDScene.h>
#include <TraDaG/GroundPlane.h>
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

    virtual PlaneFitStatus computePlaneForLabel(const std::string& label, const RGBDScene* scene, GroundPlane& result) const;

    virtual cv::Mat getLabelImage();
    virtual const cv::Mat getLabelImage() const;

    virtual LabelMap getLabelMap() const;
    virtual void setLabelMap(const LabelMap& labelMap);

    virtual LabelMode getLabelMode() const;
    virtual void setLabelMode(LabelMode mode);

protected:
    typedef std::map<cv::Vec2i, Ogre::Vector3, bool(*)(const cv::Vec2i&, const cv::Vec2i&)> InlierMap;

    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    LabelMode mLabelMode;
};

#endif // IMAGELABELING_H
