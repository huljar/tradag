#include <TraDaG/SceneAnalyzer.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <OGRE/OgreMath.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

SceneAnalyzer::SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                             const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes)
    : mDepthPath(depthDirPath)
    , mRGBPath(rgbDirPath)
    , mLabelPath(labelDirPath)
    , mCameraManager(cameraParams)
    , mLabelMap(labelMap)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
    DEBUG_OUT("Constructing SceneAnalyzer:");
    DEBUG_OUT("    Depth path: " << depthDirPath);
    DEBUG_OUT("    RGB path:   " << rgbDirPath);
    DEBUG_OUT("    Label path: " << labelDirPath);

    // Check if the paths all exist
    if(!fs::exists(mDepthPath) || !fs::exists(mRGBPath) || !fs::exists(mLabelPath))
        throw std::invalid_argument("One or more of the specified paths do not exist");

    // Check if all paths are directories
    if(!fs::is_directory(mDepthPath) || !fs::is_directory(mRGBPath) || !fs::is_directory(mLabelPath))
        throw std::invalid_argument("One or more of the specified paths are not directories");

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

        // Shuffle the scene IDs
        std::shuffle(sceneIDs.begin(), sceneIDs.end(), mRandomEngine);

        // Iterate over shuffled IDs
        for(std::vector<unsigned int>::iterator it = sceneIDs.begin(); it != sceneIDs.end() && mScenes.size() > static_cast<size_t>(maxScenes); ++it) {
            // Erase the scene ID from the map
            mScenes.erase(*it);
        }
    }
}

SceneAnalyzer::SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                             const std::string& planeDirPath,
                             const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes)
    : SceneAnalyzer(depthDirPath, rgbDirPath, labelDirPath, cameraParams, labelMap, maxScenes)
{
    DEBUG_OUT("Registering plane path: " << planeDirPath);
    mPlanePath = fs::path(planeDirPath);

    // Check if the path exists
    if(!fs::exists(mPlanePath))
        throw std::invalid_argument("Plane path does not exist");

    // Check if the path is a directory
    if(!fs::is_directory(mPlanePath))
        throw std::invalid_argument("Plane path is not a directory");

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

    DEBUG_OUT("Searching for scenes with the given labels that meet the following constraints:");
    DEBUG_OUT("    Plane normal: " << normal << ", tolerance: " << tolerance << "°");
    DEBUG_OUT("    Distance to camera: [" << minDistance << ", " << maxDistance << "]");

    std::map<unsigned int, GroundPlane> ret;

    // Convert normal and tolerance
    Ogre::Vector3 normalOgre = cvToOgre(normal);
    Ogre::Degree toleranceOgre(tolerance);

    // Iterate over all scenes
    for(FileMap::iterator it = mScenes.begin(); it != mScenes.end(); ++it) {
        // Temporary result storage variables
        bool sceneIsGood = false;
        GroundPlane plane;

        // Find available .planeinfo files for this scene
        std::vector<std::string> planes = getPlaneInfoFileNames(it->first);

        // Shuffle file names
        std::shuffle(planes.begin(), planes.end(), mRandomEngine);

        // Iterate over files
        for(std::vector<std::string>::iterator it = planes.begin(); it != planes.end(); ++it) {
            // Get path to file
            std::string filePath = (mPlanePath / fs::path(*it)).string();

            // Read plane info headers
            PlaneInfo::Headers headers = PlaneInfo::readHeadersFromFile(filePath);

            // Check parsed headers
            Ogre::Vector3& fileNormal = std::get<1>(headers);
            if(fileNormal.isZeroLength())
                continue;

            // Check if the label is one of the specified ones
            if(std::find(labels.begin(), labels.end(), std::get<0>(headers)) == labels.end()) {
                DEBUG_OUT("Label \"" << std::get<0>(headers) << "\" does not match any of the given labels");
                continue;
            }

            // Check if normal is within tolerance
            if(!normalOgre.isZeroLength() && fileNormal.angleBetween(normalOgre) > toleranceOgre
                    && (-fileNormal).angleBetween(normalOgre) > toleranceOgre) {
                DEBUG_OUT("Plane normal " << fileNormal << " is not within " << tolerance << "° of " << normalOgre <<
                          " (" << std::min(fileNormal.angleBetween(normalOgre).valueDegrees(),
                                           (-fileNormal).angleBetween(normalOgre).valueDegrees()) << "°)");
                continue;
            }

            // Plane is good, read whole file and check distance to camera
            DEBUG_OUT("Label and normal OK, parsing the rest of the file");
            PlaneInfo filePlaneInfo = PlaneInfo::readFromFile(filePath);

            // Check parsed file
            if(!filePlaneInfo.isPlaneDefined())
                continue;

            // Try to create a GroundPlane with the given distance
            plane = filePlaneInfo.createGroundPlane(minDistance, maxDistance);

            // Check if a plane was successfully created
            if(plane.isPlaneDefined()) {
                sceneIsGood = true;
                break;
            }
        }

        // If we didn't find a good plane yet, try to fit one now
        if(!sceneIsGood) {
            // Create image labeling for this scene
            ImageLabeling labeling = createImageLabeling(it->first);

            // Iterate over given labels
            for(std::vector<std::string>::const_iterator jt = labels.cbegin(); jt != labels.cend(); ++jt) {
                // Try to get a plane for this label
                PlaneFitStatus result = labeling.findPlaneForLabel(*jt, plane, normal, tolerance, minDistance, maxDistance);

                if(result == PF_SUCCESS) {
                    sceneIsGood = true;
                    break;
                }
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

bool SceneAnalyzer::precomputePlaneInfoForScene(unsigned int sceneID, const std::string& label, const cv::Vec3f& normal, float tolerance) {
    // TODO: implement
    return false;
}

bool SceneAnalyzer::precomputePlaneInfoForAllScenes(const std::string& label, const cv::Vec3f& normal, float tolerance) {
    bool ret = true;
    for(FileMap::const_iterator it = mScenes.cbegin(); it != mScenes.cend(); ++it) {
        if(!precomputePlaneInfoForScene(it->first, label, normal, tolerance))
            ret = false;
    }
    return ret;
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

void SceneAnalyzer::clearCache() {
    DEBUG_OUT("Clearing image cache");
    mMats.clear();
}

ImageLabeling SceneAnalyzer::createImageLabeling(unsigned int sceneID) {
    DEBUG_OUT("Creating ImageLabeling for scene with ID " << sceneID);

    cv::Mat depthImg, rgbImg, labelImg;
    if(!readImages(sceneID, depthImg, rgbImg, labelImg))
        throw std::runtime_error("Unable to load images for ID " + boost::lexical_cast<std::string>(sceneID)
                                 + " (" + getFileName(sceneID) + ")");

    return ImageLabeling(depthImg, labelImg, mLabelMap, mCameraManager);
}

Simulator SceneAnalyzer::createSimulator(unsigned int sceneID) {
    DEBUG_OUT("Creating Simulator for scene with ID " << sceneID);

    cv::Mat depthImg, rgbImg, labelImg;
    if(!readImages(sceneID, depthImg, rgbImg, labelImg))
        throw std::runtime_error("Unable to load images for ID " + boost::lexical_cast<std::string>(sceneID)
                                 + " (" + getFileName(sceneID) + ")");

    return Simulator(depthImg, rgbImg, mCameraManager);
}

Simulator SceneAnalyzer::createSimulator(unsigned int sceneID, const GroundPlane& plane) {
    Simulator ret = createSimulator(sceneID);
    ret.setGroundPlane(plane);
    return ret;
}

std::vector<std::string> SceneAnalyzer::getPlaneInfoFileNames(unsigned int sceneID) {
    DEBUG_OUT("Searching existing files for scene ID " << sceneID << " (" << getFileName(sceneID) << ")");

    // Check if a plane path was provided
    if(mPlanePath.empty()) {
        DEBUG_OUT("Unable to search for plane info files - no plane path specified");
        return std::vector<std::string>();
    }

    // Get file name pattern
    std::string pattern = Strings::FileNamePatternPlaneInfo + Strings::FileExtensionPlaneInfo;

    // Build regex
    ba::replace_all(pattern, "%s", getFileName(sceneID));
    ba::replace_all(pattern, "%n", "\\d+");
    ba::replace_all(pattern, ".", "\\.");
    boost::regex fileRegex('^' + pattern + '$');

    DEBUG_OUT("Constructed file name regular expression: " << fileRegex);

    // Iterate over plane directory
    fs::directory_iterator dirEnd;
    std::vector<std::string> ret;

    for(fs::directory_iterator it(mPlanePath); it != dirEnd; ++it) {
        if(fs::is_regular_file(it->status())) {
            // Check file with regex
            fs::path fileName = it->path().filename();
            if(boost::regex_match(fileName.string(), fileRegex)) {
                // File name matches
                DEBUG_OUT("File " << fileName << " matches regular expression");
                ret.push_back(fileName.string().substr(0, fileName.string().length() - Strings::FileExtensionPlaneInfo.length()));
            }
        }
    }

    DEBUG_OUT("Found " << ret.size() << " matching files");
    return ret;
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

std::string SceneAnalyzer::getPlanePath() const {
    return mPlanePath.string();
}

void SceneAnalyzer::setPlanePath(const std::string& planePath) {
    fs::path path(planePath);
    if(!fs::exists(path))
        throw std::invalid_argument("Specified path does not exist");
    if(!fs::is_directory(path))
        throw std::invalid_argument("Specified path is not a directory");
    mPlanePath = path;
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
