#include <TraDaG/ImageLabeling.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>
#include <TraDaG/Ransac.h>

#include <OGRE/OgreMath.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreVector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

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

bool ImageLabeling::containsLabel(const std::string& label) const {
    DEBUG_OUT("Checking if image contains label \"" << label << "\"");

    return !findValidLabelValues(label).empty();
}

LabelVec ImageLabeling::findValidLabelValues(const std::string& label) const {
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

ImageLabeling::PlaneFitStatus ImageLabeling::findPlaneForLabel(const std::string& label, GroundPlane& result,
                                                               const cv::Vec3f& normal, float tolerance,
                                                               unsigned short minDistance, unsigned short maxDistance,
                                                               PlaneInfo::PickMode regionMode) {

    DEBUG_OUT("Computing plane for label \"" << label << "\" with the following constraints:");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance << "°");
    DEBUG_OUT("    Distance to camera: [" << minDistance << ", " << maxDistance << "]");

    // Retrieve valid label values
    LabelVec labelValues = findValidLabelValues(label);

    // If none of the labels is contained (with at least a certain amount pixels) in the image, abort
    if(labelValues.size() == 0) {
        DEBUG_OUT("Label \"" << label << "\" is not present in the image");
        return PlaneFitStatus::INVALID_LABEL;
    }

    // Shuffle label values
    std::shuffle(labelValues.begin(), labelValues.end(), mRandomEngine);

    // Convert normal and tolerance
    Ogre::Vector3 normalOgre = cvToOgre(normal);
    Ogre::Degree toleranceOgre(tolerance);

    // Iterate over label values
    Ogre::Plane finalPlane;
    PixelWorldMap finalInliers;
    bool goodPlaneFound = false;

    for(LabelVec::iterator it = labelValues.begin(); it != labelValues.end(); ++it) {
        DEBUG_OUT("Trying to fit plane for label value " << *it);

        // Gather vertices underneath the label
        std::vector<Ogre::Vector3> points;
        std::vector<cv::Point> pixels;
        for(cv::Mat_<unsigned short>::iterator jt = mLabelImage.begin<unsigned short>(); jt != mLabelImage.end<unsigned short>(); ++jt) {
            if(*jt == *it) {
                points.push_back(cvToOgre(mCameraManager.getWorldForDepth(jt.pos(), mDepthImage.at<unsigned short>(jt.pos()))));
                pixels.push_back(jt.pos());
            }
        }

        // Perform RANSAC with the points
        Ransac<Ogre::Vector3, Ogre::Plane, 3> ransac(GroundPlane::createPlaneFromPoints, GroundPlane::pointEvaluation);
        Ransac<Ogre::Vector3, Ogre::Plane, 3>::result_type ransacRes = ransac(points);

        // Check if RANSAC found a plane
        if(ransacRes.first.normal.isZeroLength()) {
            DEBUG_OUT("RANSAC did not find a plane (" << points.size() << " input vertices)");
            continue;
        }

        // Check plane normal for validity
        if(!normalOgre.isZeroLength() && ransacRes.first.normal.angleBetween(normalOgre) > toleranceOgre
                && (-ransacRes.first.normal).angleBetween(normalOgre) > toleranceOgre) {
            DEBUG_OUT("Plane normal " << ransacRes.first.normal << " is not within " << tolerance << "° of " << normalOgre <<
                      " (" << std::min(ransacRes.first.normal.angleBetween(normalOgre).valueDegrees(),
                                       (-ransacRes.first.normal).angleBetween(normalOgre).valueDegrees()) << "°)");
            continue;
        }

        // Retrieve all inliers
        PixelWorldMap inliers(
            [] (const cv::Point& lhs, const cv::Point& rhs) -> bool {
                return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
            }
        );
        for(std::vector<Ransac<Ogre::Vector3, Ogre::Plane, 3>::const_point_iterator>::const_iterator it = ransacRes.second.cbegin(); it != ransacRes.second.cend(); ++it) {
            size_t idx = std::distance(points.cbegin(), *it);

            // Check if this point lies within the given distance interval
            unsigned short depth = mDepthImage.at<unsigned short>(pixels[idx]);

            if(depth >= minDistance && depth <= maxDistance) {
                // Insert point as inlier
                inliers.insert(std::make_pair(pixels[idx], std::make_pair(points[idx], -1)));
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
        return PlaneFitStatus::NO_GOOD_PLANE;
    }

    // Perform region growing to find the largest connected areas in the inlier set
    DEBUG_OUT("Found a suitable plane, performing region growing on inliers");

    std::vector<unsigned int> regionPointCounts = doRegionGrowing(finalInliers);

    // Filter out all regions with too little vertices
    std::vector<size_t> validRegions;
    for(size_t i = 0; i < regionPointCounts.size(); ++i) {
        if(regionPointCounts[i] >= Constants::MinRegionPixelsToBeValid) {
            DEBUG_OUT("Adding region with " << regionPointCounts[i] << " vertices");
            validRegions.push_back(i);
        }
        else {
            DEBUG_OUT("Skipping region with less than " << Constants::MinRegionPixelsToBeValid << " vertices");
        }
    }

    DEBUG_OUT("Found " << validRegions.size() << " connected regions");

    // Check if any regions are left
    if(validRegions.size() == 0) {
        DEBUG_OUT("Unable to create a ground plane - no suitable regions exist");
        return PlaneFitStatus::NO_GOOD_PLANE;
    }

    // Pick a region according to the regionMode
    int pick = -1;
    if(regionMode == PlaneInfo::PickMode::LARGEST) {
        DEBUG_OUT("Picking the largest region for plane construction");

        unsigned int largestNumVertices = 0;
        for(std::vector<size_t>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            if(regionPointCounts[*it] > largestNumVertices) {
                pick = static_cast<int>(*it);
                largestNumVertices = regionPointCounts[*it];
            }
        }
    }
    else if(regionMode == PlaneInfo::PickMode::UNIFORM_RANDOM) {
        DEBUG_OUT("Picking a random region for plane construction");

        std::uniform_int_distribution<size_t> distribution(0, validRegions.size() - 1);
        pick = static_cast<int>(validRegions[distribution(mRandomEngine)]);
    }
    else if(regionMode == PlaneInfo::PickMode::WEIGHTED_RANDOM) {
        DEBUG_OUT("Picking a weighted random region for plane construction");

        // Using the algorithm as described here: https://stackoverflow.com/questions/1761626/weighted-random-numbers
        // Weights are the number of vertices in each region

        // Calculate sum of weights
        unsigned long sumOfWeights = 0;
        for(std::vector<size_t>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            sumOfWeights += static_cast<unsigned long>(regionPointCounts[*it]);
        }

        // Pick a random number between 0 and sum of weights
        std::uniform_int_distribution<unsigned long> distribution(0, sumOfWeights - 1);
        unsigned long rnd = distribution(mRandomEngine);

        // Select the corresponding region
        bool pickFound = false;
        for(std::vector<size_t>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            unsigned long weight = static_cast<unsigned long>(regionPointCounts[*it]);
            if(rnd < weight) {
                pick = static_cast<int>(*it);
                pickFound = true;
                break;
            }
            rnd -= weight;
        }
        if(!pickFound)
            throw std::logic_error("Unable to pick a weighted random region - the function should never reach this point");
    }
    else {
        throw std::invalid_argument("Invalid region mode specified");
    }

    DEBUG_OUT("Picked region " << pick << " with " << regionPointCounts[pick] << " valid vertices");

    // Gather all 3D points from the region
    std::vector<Ogre::Vector3> planePoints;
    planePoints.reserve(regionPointCounts[pick]);

    for(PixelWorldMap::const_iterator it = finalInliers.cbegin(); it != finalInliers.cend(); ++it) {
        if(it->second.second == pick) {
            planePoints.push_back(it->second.first);
        }
    }

    // Set output parameter and return
    DEBUG_OUT("Returning plane with normal " << finalPlane.normal << " and " << planePoints.size() << " vertices");

    result = GroundPlane(finalPlane, planePoints, label);
    return PlaneFitStatus::SUCCESS;
}

ImageLabeling::PlaneFitStatus ImageLabeling::findPlaneForLabel(const std::string& label, PlaneInfo& result, const cv::Vec3f& normal, float tolerance) {
    DEBUG_OUT("Computing plane info for label \"" << label << "\" with the following constraints:");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance << "°");

    // Retrieve valid label values
    LabelVec labelValues = findValidLabelValues(label);

    // If none of the labels is contained (with at least a certain amount pixels) in the image, abort
    if(labelValues.size() == 0) {
        DEBUG_OUT("Label \"" << label << "\" is not present in the image");
        return PlaneFitStatus::INVALID_LABEL;
    }

    // Shuffle label values
    std::shuffle(labelValues.begin(), labelValues.end(), mRandomEngine);

    // Convert normal and tolerance
    Ogre::Vector3 normalOgre = cvToOgre(normal);
    Ogre::Degree toleranceOgre(tolerance);

    // Iterate over label values
    Ogre::Plane finalPlane;
    PixelWorldMap finalInliers;
    bool goodPlaneFound = false;

    for(LabelVec::iterator it = labelValues.begin(); it != labelValues.end(); ++it) {
        DEBUG_OUT("Trying to fit plane for label value " << *it);

        // Gather vertices underneath the label
        std::vector<Ogre::Vector3> points;
        std::vector<cv::Point> pixels;
        for(cv::Mat_<unsigned short>::iterator jt = mLabelImage.begin<unsigned short>(); jt != mLabelImage.end<unsigned short>(); ++jt) {
            if(*jt == *it) {
                points.push_back(cvToOgre(mCameraManager.getWorldForDepth(jt.pos(), mDepthImage.at<unsigned short>(jt.pos()))));
                pixels.push_back(jt.pos());
            }
        }

        // Perform RANSAC with the points
        Ransac<Ogre::Vector3, Ogre::Plane, 3> ransac(GroundPlane::createPlaneFromPoints, GroundPlane::pointEvaluation);
        Ransac<Ogre::Vector3, Ogre::Plane, 3>::result_type ransacRes = ransac(points);

        // Check if RANSAC found a plane
        if(ransacRes.first.normal.isZeroLength()) {
            DEBUG_OUT("RANSAC did not find a plane (" << points.size() << " input vertices)");
            continue;
        }

        // Check plane normal for validity
        if(!normalOgre.isZeroLength() && ransacRes.first.normal.angleBetween(normalOgre) > toleranceOgre
                && (-ransacRes.first.normal).angleBetween(normalOgre) > toleranceOgre) {
            DEBUG_OUT("Plane normal " << ransacRes.first.normal << " is not within " << tolerance << "° of " << normalOgre <<
                      " (" << std::min(ransacRes.first.normal.angleBetween(normalOgre).valueDegrees(),
                                       (-ransacRes.first.normal).angleBetween(normalOgre).valueDegrees()) << "°)");
            continue;
        }

        // Retrieve all inliers
        PixelWorldMap inliers(
            [] (const cv::Point& lhs, const cv::Point& rhs) -> bool {
                return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
            }
        );
        for(std::vector<Ransac<Ogre::Vector3, Ogre::Plane, 3>::const_point_iterator>::const_iterator it = ransacRes.second.cbegin(); it != ransacRes.second.cend(); ++it) {
            size_t idx = std::distance(points.cbegin(), *it);
            inliers.insert(std::make_pair(pixels[idx], std::make_pair(points[idx], -1)));
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
        return PlaneFitStatus::NO_GOOD_PLANE;
    }

    // Perform region growing to find the largest connected areas in the inlier set
    DEBUG_OUT("Found a suitable plane, performing region growing on inliers");

    std::vector<unsigned int> regionPointCounts = doRegionGrowing(finalInliers);

    DEBUG_OUT("Found " << regionPointCounts.size() << " connected regions");

    // Cluster vertices by region
    PlaneInfo::RegionVec finalRegions(regionPointCounts.size(), std::make_tuple(std::vector<Ogre::Vector3>(),
                                                                                std::numeric_limits<unsigned short>::max(),
                                                                                0));

    // Reserve space to prevent reallocation
    for(size_t i = 0; i < finalRegions.size(); ++i) {
        std::get<0>(finalRegions[i]).reserve(regionPointCounts[i]);
    }

    // Iterate over all points
    for(PixelWorldMap::iterator it = finalInliers.begin(); it != finalInliers.end(); ++it) {
        PlaneInfo::Region& currentRegion = finalRegions[it->second.second];

        // Insert point
        const Ogre::Vector3& currentPoint = it->second.first;
        std::get<0>(currentRegion).push_back(currentPoint);

        // Extend region min/max distance if necessary
        unsigned short currentDepth = mDepthImage.at<unsigned short>(it->first);
        unsigned short& currentMinDepth = std::get<1>(currentRegion);
        unsigned short& currentMaxDepth = std::get<2>(currentRegion);

        currentMinDepth = std::min(currentDepth, currentMinDepth);
        currentMaxDepth = std::max(currentDepth, currentMaxDepth);
    }

    // Set output parameter and return
    DEBUG_OUT("Returning plane info with normal " << finalPlane.normal << " and " << finalRegions.size() << " regions");

    result = PlaneInfo(finalPlane, label);
    result.regions() = std::move(finalRegions);
    return PlaneFitStatus::SUCCESS;
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

std::vector<unsigned int> ImageLabeling::doRegionGrowing(PixelWorldMap& inliers) const {
    std::vector<unsigned int> regionPointCounts;

    int currentId = 0;
    std::queue<PixelWorldMap::iterator> queue;

    // Iterate over all inliers
    for(PixelWorldMap::iterator it = inliers.begin(); it != inliers.end(); ++it) {
        // Check if this point was already processed
        if(it->second.second != -1)
            continue;

        // Add point to the queue
        queue.push(it);

        regionPointCounts.push_back(0);

        // Iterate until the whole region is marked
        while(!queue.empty()) {
            PixelWorldMap::iterator current = queue.front();
            queue.pop();

            if(current->second.second != -1)
                continue;

            // This point is unmarked, so mark it with the current region ID
            current->second.second = currentId;
            ++regionPointCounts[currentId];

            // Add all inliers to the queue that are direct neighbors (8-connected grid) of the current inlier
            cv::Point pixel = current->first;
            for(int y = pixel.y - 1; y <= pixel.y + 1; ++y) {
                for(int x = pixel.x - 1; x <= pixel.x + 1; ++x) {
                    PixelWorldMap::iterator neighbor = inliers.find(cv::Point(x, y));
                    // The current point itself will not be added since it is no longer marked with -1
                    if(neighbor != inliers.cend() && neighbor->second.second == -1) {
                        queue.push(neighbor);
                    }
                }
            }
        }

        ++currentId;
    }

    return regionPointCounts;
}
