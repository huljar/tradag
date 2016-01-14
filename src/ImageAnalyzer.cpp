#include <TraDaG/ImageAnalyzer.h>
#include <TraDaG/debug.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;

ImageAnalyzer::ImageAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                             unsigned int maxImages)
    : mDepthPath(depthDirPath)
    , mRGBPath(rgbDirPath)
    , mLabelPath(labelDirPath)
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
            ++tripleMin<fs::path>(dit, cit, lit);
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

std::vector<unsigned int> ImageAnalyzer::findScenesByLabel(const std::string& label) {
    // TODO: implement
}

std::vector<unsigned int> ImageAnalyzer::findScenesByLabel(unsigned short labelValue) {
    // TODO: implement
}

std::vector<unsigned int> ImageAnalyzer::findScenesByPlane(const std::vector<std::string>& validLabels, const cv::Vec3f& normal, float tolerance) {

}

bool ImageAnalyzer::readImages(unsigned int imageID, cv::Mat& depthImg, cv::Mat& rgbImg, cv::Mat& labelImg) {
    DEBUG_OUT("Retrieving images for ID " << imageID);

    MatMap::iterator matIter = mMats.find(imageID);
    if(matIter != mMats.end()) {
        DEBUG_OUT("Found images in cache");

        depthImg = matIter->second[0];
        rgbImg = matIter->second[1];
        labelImg = matIter->second[2];
        return true;
    }

    std::string fileName = getFileName(imageID);
    if(fileName.empty()) {
        DEBUG_OUT("Invalid image ID: " << imageID);
        return false;
    }

    DEBUG_OUT("Didn't find images in cache, reading from disk");

    fs::path fileNamePath(fileName);
    depthImg = cv::imread((mDepthPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    rgbImg = cv::imread((mRGBPath / fileNamePath).string(), CV_LOAD_IMAGE_COLOR);
    labelImg = cv::imread((mLabelPath / fileNamePath).string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    if(depthImg.data && rgbImg.data && labelImg.data) {
        mMats.insert(std::make_pair(imageID, std::array<cv::Mat, 3>({depthImg, rgbImg, labelImg})));
        return true;
    }

    DEBUG_OUT("Unable to read one or more images from disk");
    return false;
}

std::string ImageAnalyzer::getFileName(unsigned int imageID) const {
    FileMap::const_iterator fileIter = mImages.find(imageID);
    if(fileIter != mImages.cend())
        return fileIter->second;
    return std::string();
}

std::string ImageAnalyzer::getDepthPath() const {
    return mDepthPath.string();
}

std::string ImageAnalyzer::getRGBPath() const {
    return mRGBPath.string();
}

std::string ImageAnalyzer::getLabelPath() const {
    return mLabelPath.string();
}

template<typename T>
typename std::vector<T>::iterator& ImageAnalyzer::tripleMin(typename std::vector<T>::iterator& first,
                                                            typename std::vector<T>::iterator& second,
                                                            typename std::vector<T>::iterator& third) const {
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
