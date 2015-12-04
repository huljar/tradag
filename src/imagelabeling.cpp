#include <TraDaG/imagelabeling.h>
#include <TraDaG/ransac.h>

#include <chrono>
#include <random>
#include <queue>
#include <map>

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
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<size_t> distribution(0, actualLabels.size() - 1);

    unsigned short actualLabel = actualLabels[distribution(generator)];

    // Gather vertices underneath the label
    // TODO: consider labels defined on rgb image (unmapped)
    std::vector<Ogre::Vector3> points;
    std::vector<cv::Vec2i> pixels;
    for(int y = 0; y < mLabelImage.rows; ++y) {
        for(int x = 0; x < mLabelImage.cols; ++x) {
            if(mLabelImage.at<unsigned short>(y, x) == actualLabel) {
                points.push_back(scene->depthToWorld(x, y, depthImage.at<unsigned short>(y, x)));
                pixels.push_back(cv::Vec2i(y, x));
            }
        }
    }

    // Perform RANSAC with the points
    Ransac<Ogre::Vector3, Ogre::Plane, 3> ransac(createPlaneFromPoints, pointEvaluation);
    Ransac<Ogre::Vector3, Ogre::Plane, 3>::result_type result = ransac(points);

    // Retrieve all inliers
    InlierMap inliers(
        [] (const cv::Vec2i& lhs, const cv::Vec2i& rhs) -> bool {
            return lhs[0] == rhs[0] ? lhs[1] < rhs[1] : lhs[0] < rhs[0];
        }
    );

    for(std::vector<Ransac<Ogre::Vector3, Ogre::Plane, 3>::const_point_iterator>::const_iterator it = result.second.cbegin(); it != result.second.cend(); ++it) {
        size_t idx = std::distance(points.cbegin(), *it);
        inliers.insert(std::make_pair(pixels[idx], points[idx]));
    }

    // Perform region growing to find the largest connected areas in the inlier set
    std::vector<int> regionIds(inliers.size(), -1);
    std::vector<int> regionPointCounts;
    int currentId = 0;

    std::queue<InlierMap::const_iterator> queue;

    // Iterate over all inliers
    for(InlierMap::const_iterator it = inliers.cbegin(); it != inliers.cend(); ++it) {
        // Check if this point was already processed
        if(regionIds[std::distance(inliers.cbegin(), it)] != -1)
            continue;

        // Add point to the queue
        queue.push(it);

        regionPointCounts.push_back(0);

        // Iterate until the whole region is marked
        while(!queue.empty()) {
            InlierMap::const_iterator current = queue.front();
            queue.pop();

            if(regionIds[std::distance(inliers.cbegin(), current)] != -1)
                continue;

            // This point is unmarked, so mark it with the current region ID
            regionIds[std::distance(inliers.cbegin(), current)] = currentId;
            ++regionPointCounts[currentId];

            // Add all inliers to the queue that are direct neighbors (8-connected grid) of the current inlier
            InlierMap::key_type pixel = current->first;
            for(int y = pixel[0] - 1; y <= pixel[0] + 1; ++y) {
                for(int x = pixel[1] - 1; x <= pixel[1] + 1; ++x) {
                    InlierMap::const_iterator neighbor = inliers.find(cv::Vec2i(y, x));
                    // The current point itself will not be added since it is no longer marked with -1
                    if(neighbor != inliers.cend() && regionIds[std::distance(inliers.cbegin(), neighbor)] == -1) {
                        queue.push(neighbor);
                    }
                }
            }
        }

        ++currentId;
    }

    // Select a random large region to use for the plane
    // TODO: define probability distribution (logarithmic?) over region size, select a random region
    std::pair<int, int> selectedRegion = std::make_pair(0, regionPointCounts[0]);
    for(size_t i = 1; i < regionPointCounts.size(); ++i) {
        if(regionPointCounts[i] > selectedRegion.second) {
            selectedRegion.first = (int)i;
            selectedRegion.second = regionPointCounts[i];
        }
    }

    // Gather all 3D points from the region
    std::vector<Ogre::Vector3> planePoints;
    planePoints.reserve(selectedRegion.second);

    for(InlierMap::const_iterator it = inliers.cbegin(); it != inliers.cend(); ++it) {
        if(regionIds[std::distance(inliers.cbegin(), it)] == selectedRegion.first) {
            planePoints.push_back(it->second);
        }
    }

    // TODO: apply least-squares-fit using only the remaining inliers?

    return PlaneFittingResult(SUCCESS_FIT, result.first, planePoints);
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
    Ogre::Plane ret(points[0], points[1], points[2]);
    ret.normalise(); // required so the getDistance member function returns the correct distance
    return ret;
}

float ImageLabeling::pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane) {
    // Check for invalid plane first (this can occur, probably when the RANSAC sampled points
    // lie on a line and no unique plane can be created)
    if(plane.normal == Ogre::Vector3::ZERO)
        return 1.0; // treat every point as outlier so this model will be discarded

    // Ogre::Plane::getDistance returns positive/negative values depending on which side of the plane the point lies
    if(std::abs(plane.getDistance(point)) < Constants::RansacConfidenceInterval)
        return 0.0;

    return 1.0;
}
