#include <TraDaG/imagelabeling.h>

using namespace TraDaG;

ImageLabeling::ImageLabeling(const cv::Mat& labelImage, LabelMode labelMode)
    : mLabelImage(labelImage)
    , mLabelMode(labelMode)
{
}

ImageLabeling::~ImageLabeling() {

}

cv::Mat ImageLabeling::getLabelImage() {
    return mLabelImage;
}

const cv::Mat ImageLabeling::getLabelImage() const {
    return mLabelImage;
}

LabelMode ImageLabeling::getLabelMode() const {
    return mLabelMode;
}

void ImageLabeling::setLabelMode(LabelMode mode) {
    mLabelMode = mode;
}
