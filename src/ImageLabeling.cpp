#include <TraDaG/ImageLabeling.h>
#include <TraDaG/Ransac.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <algorithm>
#include <chrono>
#include <queue>
#include <map>
#include <stdexcept>

using namespace TraDaG;

ImageLabeling::ImageLabeling(const cv::Mat& depthImage, const cv::Mat& labelImage, const LabelMap& labelMap, const CameraManager& cameraParams)
    : mDepthImage(depthImage)
    , mLabelImage(labelImage)
    , mLabelMap(labelMap)
    , mCameraManager(cameraParams)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
    if(depthImage.rows != labelImage.rows || depthImage.cols != labelImage.cols)
        throw std::invalid_argument("Depth image and label image do not have the same dimensions");
}

ImageLabeling::~ImageLabeling() {
}

bool ImageLabeling::containsLabel(const std::string& label) {
    DEBUG_OUT("Checking if image contains label \"" << label << "\"");

    return !findValidLabelValues(label).empty();
}

LabelVec ImageLabeling::findValidLabelValues(const std::string& label) {
    DEBUG_OUT("Finding valid label values for label \"" << label << "\"");

    // Check if the label is contained in the label map
    LabelMap::const_iterator entry = mLabelMap.find(label);
    if(entry == mLabelMap.end()) {
        DEBUG_OUT("Label \"" << label << "\" is invalid (not in label map)");
        return LabelVec();
    }
    LabelVec labelValues = entry->second;

    // Check which actual labels are contained within the image
    LabelVec actualLabels;
    for(LabelVec::iterator it = labelValues.begin(); it != labelValues.end(); ++it) {
        // We need at least 3 pixels with this label for it to be considered
        unsigned int numPx = 0;

        // Iterate over all pixels
        bool next = false;
        for(int y = 0; y < mLabelImage.rows && !next; ++y) {
            for(int x = 0; x < mLabelImage.cols && !next; ++x) {
                // If the label is found at a pixel, save it
                if(mLabelImage.at<unsigned short>(y, x) == *it) {
                    if(++numPx >= Constants::MinLabelPixelsToBeValid) {
                        actualLabels.push_back(*it);
                        next = true;
                    }
                }
            }
        }
    }

    DEBUG_OUT(entry->second.size() << " possible label values, " << actualLabels.size() << " present in the image");

    return actualLabels;
}

PlaneFitStatus ImageLabeling::findPlaneForLabel(const std::string& label, GroundPlane& result,
                                                const cv::Vec3f& normal, float tolerance,
                                                unsigned short minDistance, unsigned short maxDistance) {

    DEBUG_OUT("Computing plane for label \"" << label << "\" with the following constraints:");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance << "°");
    DEBUG_OUT("    Distance to camera: [" << minDistance << ", " << maxDistance << "]");

    // Retrieve valid label values
    LabelVec labelValues = findValidLabelValues(label);

    // If none of the labels is contained (with at least a certain amount pixels) in the image, abort
    if(labelValues.size() == 0) {
        DEBUG_OUT("Label \"" << label << "\" is not present in the image");
        return PF_INVALID_LABEL;
    }

    // Scramble label values
    std::shuffle(labelValues.begin(), labelValues.end(), mRandomEngine);

    // Convert normal and tolerance
    Ogre::Vector3 normalOgre = cvToOgre(normal);
    Ogre::Radian toleranceOgre(Ogre::Degree(tolerance).valueRadians());

    // Iterate over label values
    Ogre::Plane finalPlane;
    PixelWorldMap finalInliers;
    bool goodPlaneFound = false;

    for(LabelVec::iterator it = labelValues.begin(); it != labelValues.end(); ++it) {
        DEBUG_OUT("Trying to fit plane for label value " << *it);

        // Gather vertices underneath the label
        std::vector<Ogre::Vector3> points;
        std::vector<cv::Vec2i> pixels;
        for(int y = 0; y < mLabelImage.rows; ++y) {
            for(int x = 0; x < mLabelImage.cols; ++x) {
                if(mLabelImage.at<unsigned short>(y, x) == *it) {
                    points.push_back(cvToOgre(mCameraManager.getWorldForDepth(x, y, mDepthImage.at<unsigned short>(y, x))));
                    pixels.push_back(cv::Vec2i(y, x));
                }
            }
        }

        // Perform RANSAC with the points
        Ransac<Ogre::Vector3, Ogre::Plane, 3> ransac(GroundPlane::createPlaneFromPoints, GroundPlane::pointEvaluation);
        Ransac<Ogre::Vector3, Ogre::Plane, 3>::result_type ransacRes = ransac(points);

        // Check plane normal for validity
        if(!normalOgre.isZeroLength() && ransacRes.first.normal.angleBetween(normalOgre) > toleranceOgre) {
            DEBUG_OUT("Plane normal " << ransacRes.first.normal << "is not within " << tolerance << " degrees of " << normalOgre);
            continue;
        }

        // Retrieve all inliers
        PixelWorldMap inliers(
            [] (const cv::Vec2i& lhs, const cv::Vec2i& rhs) -> bool {
                return lhs[0] == rhs[0] ? lhs[1] < rhs[1] : lhs[0] < rhs[0];
            }
        );
        for(std::vector<Ransac<Ogre::Vector3, Ogre::Plane, 3>::const_point_iterator>::const_iterator it = ransacRes.second.cbegin(); it != ransacRes.second.cend(); ++it) {
            size_t idx = std::distance(points.cbegin(), *it);

            // Check if this point lies within the given distance interval
            cv::Vec2i depthPx = pixels[idx];
            unsigned short depth = mDepthImage.at<unsigned short>(depthPx[0], depthPx[1]);

            if(depth >= minDistance && depth <= maxDistance) {
                // Insert point as inlier
                inliers.insert(std::make_pair(depthPx, points[idx]));
            }
        }

        // Check if we didn't find any inliers
        if(inliers.empty()) {
            DEBUG_OUT("Didn't find any points that lie within [" << minDistance << ", " << maxDistance << "] distance to the camera");
            continue;
        }

        // If we reach this point, the plane meets the requirements
        goodPlaneFound = true;
        finalPlane = std::move(ransacRes.first);
        finalInliers = std::move(inliers);
        break;
    }

    // Abort if no good plane was found
    if(!goodPlaneFound) {
        DEBUG_OUT("Did not find any suitable planes that meet the requirements");
        return PF_NO_GOOD_PLANE;
    }

    // Perform region growing to find the largest connected areas in the inlier set
    DEBUG_OUT("Found a suitable plane, performing region growing on inliers");

    std::vector<int> regionIds(finalInliers.size(), -1);
    std::vector<int> regionPointCounts;
    int currentId = 0;

    std::queue<PixelWorldMap::const_iterator> queue;

    // Iterate over all inliers
    for(PixelWorldMap::const_iterator it = finalInliers.cbegin(); it != finalInliers.cend(); ++it) {
        // Check if this point was already processed
        if(regionIds[std::distance(finalInliers.cbegin(), it)] != -1)
            continue;

        // Add point to the queue
        queue.push(it);

        regionPointCounts.push_back(0);

        // Iterate until the whole region is marked
        while(!queue.empty()) {
            PixelWorldMap::const_iterator current = queue.front();
            queue.pop();

            if(regionIds[std::distance(finalInliers.cbegin(), current)] != -1)
                continue;

            // This point is unmarked, so mark it with the current region ID
            regionIds[std::distance(finalInliers.cbegin(), current)] = currentId;
            ++regionPointCounts[currentId];

            // Add all inliers to the queue that are direct neighbors (8-connected grid) of the current inlier
            PixelWorldMap::key_type pixel = current->first;
            for(int y = pixel[0] - 1; y <= pixel[0] + 1; ++y) {
                for(int x = pixel[1] - 1; x <= pixel[1] + 1; ++x) {
                    PixelWorldMap::const_iterator neighbor = finalInliers.find(cv::Vec2i(y, x));
                    // The current point itself will not be added since it is no longer marked with -1
                    if(neighbor != finalInliers.cend() && regionIds[std::distance(finalInliers.cbegin(), neighbor)] == -1) {
                        queue.push(neighbor);
                    }
                }
            }
        }

        ++currentId;
    }

    DEBUG_OUT("Found " << regionPointCounts.size() << " connected regions, selecting one");

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

    for(PixelWorldMap::const_iterator it = finalInliers.cbegin(); it != finalInliers.cend(); ++it) {
        if(regionIds[std::distance(finalInliers.cbegin(), it)] == selectedRegion.first) {
            planePoints.push_back(it->second);
        }
    }

    // Set output parameter and return
    DEBUG_OUT("Returning plane with normal " << finalPlane.normal << " and " << planePoints.size() << " vertices");

    result = GroundPlane(finalPlane, planePoints, label);
    return PF_SUCCESS;
}

cv::Mat ImageLabeling::getDepthImage() const {
    return mDepthImage;
}

cv::Mat ImageLabeling::getLabelImage() const {
    return mLabelImage;
}

LabelMap ImageLabeling::getLabelMap() const {
    return mLabelMap;
}

CameraManager ImageLabeling::getCameraManager() const {
    return mCameraManager;
}
