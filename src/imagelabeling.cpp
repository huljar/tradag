#include <TraDaG/imagelabeling.h>
#include <TraDaG/ransac.h>

#include <random>

using namespace TraDaG;

ImageLabeling::ImageLabeling(const cv::Mat& labelImage, const LabelMap& labelMap, LabelMode labelMode)
    : mLabelImage(labelImage)
    , mLabelMap(labelMap)
    , mLabelMode(labelMode)
{
}

ImageLabeling::~ImageLabeling() {

}

PlaneFittingResult ImageLabeling::getPlaneForLabel(const std::string& label, const RgbdObject* scene) const {
    // Get depth image from scene
    const cv::Mat depthImage = scene->getDepthImage();

    // Check if the depth image has the same dimensions as the label image
    // TODO: what to do if the labels are defined on RGB image?
    if(depthImage.rows != mLabelImage.rows || depthImage.cols != mLabelImage.cols)
        return PlaneFittingResult(DEPTH_LABEL_DIFFERENT_DIMENSIONS);

    // Check if the label is contained in the label map
    LabelMap::const_iterator entry = mLabelMap.find(label);
    if(entry == mLabelMap.end())
        return PlaneFittingResult(INVALID_LABEL);

    LabelVec labelValues = entry->second;

    // Check which actual labels are contained within the image
    LabelVec actualLabels;
    for(LabelVec::iterator it = labelValues.begin(); it != labelValues.end(); ++it) {
        // Iterate over all pixels
        bool next = false;
        for(int y = 0; y < mLabelImage.rows && !next; ++y) {
            for(int x = 0; x < mLabelImage.cols && !next; ++x) {
                // If the label is found at a pixel, save it and continue with next label
                if(mLabelImage.at<unsigned short>(y, x) == *it) {
                    actualLabels.push_back(*it);
                    next = true;
                }
            }
        }
    }

    // If none of the labels is contained in the image, abort
    if(actualLabels.size() == 0)
        return PlaneFittingResult(LABEL_NOT_IN_IMAGE);

    // Select a random valid label
    // TODO: random?
    std::default_random_engine generator;
    std::uniform_int_distribution<size_t> distribution(0, actualLabels.size() - 1);

    unsigned short actualLabel = actualLabels[distribution(generator)];

    // Gather vertices underneath the label
    std::vector<Ogre::Vector3> points;
    for(int y = 0; y < mLabelImage.rows; ++y) {
        for(int x = 0; x < mLabelImage.cols; ++x) {
            if(mLabelImage.at<unsigned short>(y, x) == actualLabel) {
                points.push_back(scene->depthToWorld(x, y, depthImage.at<unsigned short>(y, x)));
            }
        }
    }

    // Perform RANSAC with the points
    Ransac<Ogre::Vector3, Ogre::Plane, 3> ransac(createPlaneFromPoints, pointEvaluation);
    std::pair<Ogre::Plane, std::vector<const Ogre::Vector3*>> result = ransac(points);

    // If the plane does not face the camera, invert its normal
    if(result.first.normal.dotProduct(Ogre::Vector3::UNIT_Z) < 0) {
        result.first.normal = -result.first.normal;
        result.first.d = -result.first.d;
    }

    // TODO: after ransac do region growing (4/8-connected pixel neighborhood) with x random starting pixels, select one of the largest regions

    return PlaneFittingResult(SUCCESS_FIT, result.first);
}

cv::Mat ImageLabeling::getLabelImage() {
    return mLabelImage;
}

const cv::Mat ImageLabeling::getLabelImage() const {
    return mLabelImage;
}

LabelMap ImageLabeling::getLabelMap() const {
    return mLabelMap;
}

void ImageLabeling::setLabelMap(const LabelMap& labelMap) {
    mLabelMap = labelMap;
}

LabelMode ImageLabeling::getLabelMode() const {
    return mLabelMode;
}

void ImageLabeling::setLabelMode(LabelMode mode) {
    mLabelMode = mode;
}

Ogre::Plane ImageLabeling::createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points) {
    return Ogre::Plane(points[0], points[1], points[2]);
}

float ImageLabeling::pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane) {
    if(plane.getDistance(point) < Constants::RansacConfidenceInterval)
        return 0.0;

    return 1.0;
}
