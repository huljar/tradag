#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/util.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace TraDaG {
    class ImageLabeling;
}

class TraDaG::ImageLabeling
{
public:
    ImageLabeling(const cv::Mat& labelImage, LabelMode labelMode);
    virtual ~ImageLabeling();

    virtual cv::Mat getLabelImage();
    virtual const cv::Mat getLabelImage() const;

    virtual LabelMode getLabelMode() const;
    virtual void setLabelMode(LabelMode mode);

protected:
    cv::Mat mLabelImage;
    LabelMode mLabelMode;
};

#endif // IMAGELABELING_H
