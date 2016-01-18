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
                             const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes)
    : mDepthPath(depthDirPath)
    , mRGBPath(rgbDirPath)
    , mLabelPath(labelDirPath)
    , mLabelMap(labelMap)
    , mCameraManager(cameraParams)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
    DEBUG_OUT("Constructing SceneAnalyzer:");
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
    std::vector<unsigned int> sceneIDs;

    // Iterate over entries
    std::vector<fs::path>::iterator dit = depthImgs.begin(), dend = depthImgs.end(),
                                    cit = rgbImgs.begin(),   cend = rgbImgs.end(),
                                    lit = labelImgs.begin(), lend = labelImgs.end();

    while(dit != dend && cit != cend && lit != lend) {
        // Check if the file names are equal
        if(*dit == *cit && *dit == *lit) {
            // Add scene
            unsigned int id = std::distance(depthImgs.begin(), dit) + 1;
            mScenes.insert(std::make_pair(id, dit->string()));
            sceneIDs.push_back(id);

            // Step to the next scene
            ++dit;
            ++cit;
            ++lit;
        }
        else {
            // File names are not equal, so increment the iterator pointing to the lowest name (lexicographic comparison)
            ++tripleMin<std::vector<fs::path>>(dit, cit, lit);
        }
    }

    DEBUG_OUT("Found " << mScenes.size() << " matching file names");

    // If we added more scenes than the maximum, remove random ones until reaching the maximum
    if(maxScenes > 0 && maxScenes < mScenes.size()) {
        DEBUG_OUT("Selecting " << maxScenes << " random scenes");

        // Scramble the scene IDs
        std::shuffle(sceneIDs.begin(), sceneIDs.end(), mRandomEngine);

        // Iterate over shuffled IDs
        for(std::vector<unsigned int>::iterator it = sceneIDs.begin(); it != sceneIDs.end() && mScenes.size() > static_cast<size_t>(maxScenes); ++it) {
            // Erase the scene ID from the map
            mScenes.erase(*it);
        }
    }
}

std::vector<unsigned int> SceneAnalyzer::findScenesByLabel(const std::vector<std::string>& labels) {
    DEBUG_OUT("Searching for scenes with the given labels");
    DEBUG_OUT("    Number of input labels: " << labels.size());

    std::vector<unsigned int> ret;

    // Iterate over all scenes
    for(FileMap::iterator it = mScenes.begin(); it != mScenes.end(); ++it) {
        // Get images
        cv::Mat depthImg, rgbImg, labelImg;
        if(!readImages(it->first, depthImg, rgbImg, labelImg))
            continue;

        // Create image labeling for this scene
        ImageLabeling labeling(depthImg, labelImg, mLabelMap, mCameraManager);

        // Iterate over given labels
        for(std::vector<std::string>::const_iterator jt = labels.cbegin(); jt != labels.cend(); ++jt) {
            // Check if it contains the label
            if(labeling.containsLabel(*jt)) {
                DEBUG_OUT("Scene \"" << it->second << "\" contains label \"" << *jt << "\"");

                ret.push_back(it->first);
                break;
            }
        }
    }

    return ret;
}

std::vector<unsigned int> SceneAnalyzer::findScenesByLabel(const std::string& label) {
    return findScenesByLabel(std::vector<std::string>({label}));
}

std::map<unsigned int, GroundPlane> SceneAnalyzer::findScenesByPlane(const std::vector<std::string>& labels,
                                                                     const cv::Vec3f& normal, float tolerance,
                                                                     unsigned short minDistance, unsigned short maxDistance) {

    DEBUG_OUT("Searching for scenes with valid labels that meet the following constraints:");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance << "°");
    DEBUG_OUT("    Distance to camera: [" << minDistance << ", " << maxDistance << "]");

    std::map<unsigned int, GroundPlane> ret;

    // Iterate over all scenes
    for(FileMap::iterator it = mScenes.begin(); it != mScenes.end(); ++it) {
        // Get images
        cv::Mat depthImg, rgbImg, labelImg;
        if(!readImages(it->first, depthImg, rgbImg, labelImg))
            continue;

        // Create image labeling for this scene
        ImageLabeling labeling(depthImg, labelImg, mLabelMap, mCameraManager);
        GroundPlane plane;

        // Iterate over given labels
        bool sceneIsGood = false;
        for(std::vector<std::string>::const_iterator jt = labels.cbegin(); jt != labels.cend(); ++jt) {
            // Try to get a plane for this label
            PlaneFitStatus result = labeling.findPlaneForLabel(*jt, plane, normal, tolerance, minDistance, maxDistance);

            if(result == PF_SUCCESS) {
                sceneIsGood = true;
                break;
            }
        }

        if(sceneIsGood) {
            DEBUG_OUT("Found plane for scene \"" << it->second << "\" and label \"" << plane.getLabel() << "\"");
            DEBUG_OUT("    Plane normal: " << plane.ogrePlane().normal);
            DEBUG_OUT("    Plane vertices: " << plane.vertices().size());

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

bool SceneAnalyzer::readImages(unsigned int sceneID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage) {
    DEBUG_OUT("Retrieving images for scene ID " << sceneID);

    MatMap::iterator matIter = mMats.find(sceneID);
    if(matIter != mMats.end()) {
        DEBUG_OUT("Found images in cache");

        depthImage = matIter->second[0];
        rgbImage = matIter->second[1];
        labelImage = matIter->second[2];
        return true;
    }

    std::string fileName = getFileName(sceneID);
    if(fileName.empty()) {
        DEBUG_OUT("Invalid scene ID: " << sceneID);
        return false;
    }

    DEBUG_OUT("Didn't find images in cache, reading from disk");

    fs::path fileNamePath(fileName);
    depthImage = cv::imread((mDepthPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    rgbImage = cv::imread((mRGBPath / fileNamePath).string(), CV_LOAD_IMAGE_COLOR);
    labelImage = cv::imread((mLabelPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    if(depthImage.data && rgbImage.data && labelImage.data) {
        mMats.insert(std::make_pair(sceneID, std::array<cv::Mat, 3>({depthImage, rgbImage, labelImage})));
        return true;
    }

    DEBUG_OUT("Unable to read one or more images from disk");
    return false;
}

Simulator SceneAnalyzer::createSimulator(unsigned int sceneID) {
    DEBUG_OUT("Creating Simulator for scene with ID " << sceneID);

    cv::Mat depthImg, rgbImg, labelImg;
    if(!readImages(sceneID, depthImg, rgbImg, labelImg))
        throw std::runtime_error("Unable to load images for ID " + boost::lexical_cast<std::string>(sceneID)
                                 + " (" + getFileName(sceneID) + ")");

    return Simulator(depthImg, rgbImg, mCameraManager);
}

std::string SceneAnalyzer::getFileName(unsigned int sceneID) const {
    FileMap::const_iterator fileIter = mScenes.find(sceneID);
    if(fileIter != mScenes.cend())
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
