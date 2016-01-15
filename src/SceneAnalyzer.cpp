#include <TraDaG/SceneAnalyzer.h>
#include <TraDaG/ImageLabeling.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <OGRE/OgreMath.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;

SceneAnalyzer::SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                             const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxImages)
    : mDepthPath(depthDirPath)
    , mRGBPath(rgbDirPath)
    , mLabelPath(labelDirPath)
    , mLabelMap(labelMap)
    , mCameraManager(cameraParams)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
    DEBUG_OUT("Constructing ImageAnalyzer:");
    DEBUG_OUT("    Depth path: " << depthDirPath);
    DEBUG_OUT("    RGB path:   " << rgbDirPath);
    DEBUG_OUT("    Label path: " << labelDirPath);

    // Check if the paths all exist
    if(!fs::exists(mDepthPath) || !fs::exists(mRGBPath) || !fs::exists(mLabelPath))
        throw std::invalid_argument("One or more of the specified paths do not exist");

    // Collect directory entries
    std::vector<fs::path> depthImgs, rgbImgs, labelImgs;
    fs::directory_iterator dirEnd;

    for(fs::directory_iterator it(mDepthPath); it != dirEnd; ++it) {
        if(fs::is_regular_file(it->status()) && boost::iequals(it->path().extension().string(), ".png")) {
            depthImgs.push_back(it->path().filename());
        }
    }

    for(fs::directory_iterator it(mRGBPath); it != dirEnd; ++it) {
        if(fs::is_regular_file(it->status()) && boost::iequals(it->path().extension().string(), ".png")) {
            rgbImgs.push_back(it->path().filename());
        }
    }

    for(fs::directory_iterator it(mLabelPath); it != dirEnd; ++it) {
        if(fs::is_regular_file(it->status()) && boost::iequals(it->path().extension().string(), ".png")) {
            labelImgs.push_back(it->path().filename());
        }
    }

    DEBUG_OUT("Found " << depthImgs.size() << " depth images, " << rgbImgs.size() << " RGB images and " << labelImgs.size() << " label images");

    // Sort entries
    std::sort(depthImgs.begin(), depthImgs.end());
    std::sort(rgbImgs.begin(), rgbImgs.end());
    std::sort(labelImgs.begin(), labelImgs.end());

    // Temporary vector to hold map keys for fast lookup
    std::vector<unsigned int> imageIDs;

    // Iterate over entries
    std::vector<fs::path>::iterator dit = depthImgs.begin(), dend = depthImgs.end(),
                                    cit = rgbImgs.begin(),   cend = rgbImgs.end(),
                                    lit = labelImgs.begin(), lend = labelImgs.end();

    while(dit != dend && cit != cend && lit != lend) {
        // Check if the file names are equal
        if(*dit == *cit && *dit == *lit) {
            // Add image
            unsigned int id = std::distance(depthImgs.begin(), dit) + 1;
            mImages.insert(std::make_pair(id, dit->string()));
            imageIDs.push_back(id);

            // Step to the next image
            ++dit;
            ++cit;
            ++lit;
        }
        else {
            // File names are not equal, so increment the iterator pointing to the lowest name (lexicographic comparison)
            ++tripleMin<std::vector<fs::path>>(dit, cit, lit);
        }
    }

    DEBUG_OUT("Found " << mImages.size() << " matching file names");

    // If we added more images than the maximum, remove random ones until reaching the maximum
    if(maxImages > 0 && maxImages < mImages.size()) {
        DEBUG_OUT("Selecting " << maxImages << " random images");

        // Scramble the image IDs
        std::shuffle(imageIDs.begin(), imageIDs.end(), mRandomEngine);

        // Iterate over shuffled IDs
        for(std::vector<unsigned int>::iterator it = imageIDs.begin(); it != imageIDs.end() && mImages.size() > static_cast<size_t>(maxImages); ++it) {
            // Erase the image ID from the map
            mImages.erase(*it);
        }
    }
}

std::vector<unsigned int> SceneAnalyzer::findScenesByLabel(const std::vector<std::string>& labels) {

}

std::vector<unsigned int> SceneAnalyzer::findScenesByLabel(const std::string& label) {
    return findScenesByLabel(std::vector<std::string>({label}));
}

std::vector<unsigned int> SceneAnalyzer::findScenesByLabel(unsigned short labelValue) {
    // TODO: implement
}

std::map<unsigned int, GroundPlane> SceneAnalyzer::findScenesByPlane(const std::vector<std::string>& labels,
                                                                     const cv::Vec3f& normal, float tolerance,
                                                                     unsigned short minDistance, unsigned short maxDistance) {

    DEBUG_OUT("Searching for scenes with valid labels and");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance);
    DEBUG_OUT("    Distance to camera: [" << minDistance << ", " << maxDistance << "]");

    std::map<unsigned int, GroundPlane> ret;

    Ogre::Vector3 normalOgre = cvToOgre(normal);
    Ogre::Radian toleranceOgre(Ogre::Degree(tolerance).valueRadians());

    // Iterate over all scenes
    for(FileMap::iterator it = mImages.begin(); it != mImages.end(); ++it) {
        // Get images
        cv::Mat depthImg, rgbImg, labelImg;
        if(!readImages(it->first, depthImg, rgbImg, labelImg))
            continue;

        // Create image labeling for this scene
        ImageLabeling labeling(depthImg, labelImg, mLabelMap, mCameraManager);
        GroundPlane plane;

        // Iterate over given labels
        bool sceneIsGood = false;
        for(std::vector<std::string>::const_iterator jt = labels.cbegin(); jt != labels.cend() && !sceneIsGood; ++jt) {
            // Try to get a plane for this label
            PlaneFitStatus result = labeling.computePlaneForLabel(*jt, plane);

            if(result == PF_SUCCESS) {
                // Check if the plane normal is within the given tolerance
                if(plane.ogrePlane().normal.angleBetween(normalOgre) <= toleranceOgre) {

                    // Check if any plane vertices lie within the distance interval
                    // Use reverse iterator so it stays valid when elements are removed
                    std::vector<Ogre::Vector3>& vertices = plane.vertices();
                    for(std::vector<Ogre::Vector3>::reverse_iterator kt = vertices.rbegin(); kt != vertices.rend(); ++kt) {
                        // Get depth to camera
                        cv::Vec2i depthPx = mCameraManager.getDepthForWorld(kt->x, kt->y, kt->z);
                        unsigned short depth = depthImg.at<unsigned short>(depthPx[1], depthPx[0]);

                        // Check if depth is within interval
                        if(depth >= minDistance && depth <= maxDistance) {
                            // We have found a plane meeting the criteria
                            sceneIsGood = true;
                        }
                        else {
                            // Remove this vertex efficiently
                            std::swap(*kt, vertices.back());
                            vertices.pop_back();
                        }
                    }
                }
            }
        }

        if(sceneIsGood) {
            DEBUG_OUT("Found plane for scene \"" << it->second << "\" and label \"" << plane.getLabel() << "\"");
            DEBUG_OUT("    Plane normal: " << plane.ogrePlane().normal);
            DEBUG_OUT("    Valid plane vertices: " << plane.vertices().size());

            ret.insert(std::make_pair(it->first, plane));
        }
    }

    return ret;
}

std::map<unsigned int, GroundPlane> SceneAnalyzer::findScenesByPlane(const std::string& label,
                                                                     const cv::Vec3f& normal, float tolerance,
                                                                     unsigned short minDistance, unsigned short maxDistance) {

    return findScenesByPlane(std::vector<std::string>({label}), normal, tolerance, minDistance, maxDistance);
}

bool SceneAnalyzer::readImages(unsigned int imageID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage) {
    DEBUG_OUT("Retrieving images for ID " << imageID);

    MatMap::iterator matIter = mMats.find(imageID);
    if(matIter != mMats.end()) {
        DEBUG_OUT("Found images in cache");

        depthImage = matIter->second[0];
        rgbImage = matIter->second[1];
        labelImage = matIter->second[2];
        return true;
    }

    std::string fileName = getFileName(imageID);
    if(fileName.empty()) {
        DEBUG_OUT("Invalid image ID: " << imageID);
        return false;
    }

    DEBUG_OUT("Didn't find images in cache, reading from disk");

    fs::path fileNamePath(fileName);
    depthImage = cv::imread((mDepthPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    rgbImage = cv::imread((mRGBPath / fileNamePath).string(), CV_LOAD_IMAGE_COLOR);
    labelImage = cv::imread((mLabelPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    if(depthImage.data && rgbImage.data && labelImage.data) {
        mMats.insert(std::make_pair(imageID, std::array<cv::Mat, 3>({depthImage, rgbImage, labelImage})));
        return true;
    }

    DEBUG_OUT("Unable to read one or more images from disk");
    return false;
}

TradagMain SceneAnalyzer::createSimulator(unsigned int imageID) {
    cv::Mat depthImg, rgbImg, labelImg;
    if(!readImages(imageID, depthImg, rgbImg, labelImg))
        throw std::runtime_error("Couldn't create simulator, one or more images for ID " + boost::lexical_cast<std::string>(imageID) + " are unreadable");

    return TradagMain(depthImg, rgbImg, labelImg, mLabelMap, mCameraManager);
}

std::string SceneAnalyzer::getFileName(unsigned int imageID) const {
    FileMap::const_iterator fileIter = mImages.find(imageID);
    if(fileIter != mImages.cend())
        return fileIter->second;
    return std::string();
}

std::string SceneAnalyzer::getDepthPath() const {
    return mDepthPath.string();
}

std::string SceneAnalyzer::getRGBPath() const {
    return mRGBPath.string();
}

std::string SceneAnalyzer::getLabelPath() const {
    return mLabelPath.string();
}

CameraManager SceneAnalyzer::getCameraManager() const {
    return mCameraManager;
}

LabelMap SceneAnalyzer::getLabelMap() const {
    return mLabelMap;
}

template<typename T>
typename T::iterator& SceneAnalyzer::tripleMin(typename T::iterator& first,
                                               typename T::iterator& second,
                                               typename T::iterator& third) const {
    if(*first < *second) {
        if(*first < *third)
            return first;
        else
            return third;
    }
    else {
        if(*second < *third)
            return second;
        else
            return third;
    }
}
