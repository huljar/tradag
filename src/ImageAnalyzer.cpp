#include <TraDaG/ImageAnalyzer.h>

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
{
    // Check if the paths all exist
    if(!fs::exists(mDepthPath) || !fs::exists(mRGBPath) || !fs::exists(mLabelPath))
        throw std::invalid_argument("One or more of the specified paths do not exist");

    // Collect directory entries
    std::vector<fs::path> depthImgs, rgbImgs, labelImgs;
    fs::directory_iterator dirEnd;

    for(fs::directory_iterator it(mDepthPath); it != dirEnd; ++it) {
        if(fs::is_regular_file(it->status()) && boost::iequals(it->path().extension().string(), ".png")) {
            depthImgs.push_back(it->path().filename()); // TODO: test if filename() is necessary
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

    // Sort entries
    std::sort(depthImgs.begin(), depthImgs.end());
    std::sort(rgbImgs.begin(), rgbImgs.end());
    std::sort(labelImgs.begin(), labelImgs.end());

    // Iterate over entries
    std::vector<fs::path>::iterator dit = depthImgs.begin(), dend = depthImgs.end(),
                                    cit = rgbImgs.begin(),   cend = rgbImgs.end(),
                                    lit = labelImgs.begin(), lend = labelImgs.end();

    while(dit != dend && cit != cend && lit != lend) {
        // Check if the file names are equal
        if(*dit == *cit && *dit == *lit) {
            // Add image
            mImages.insert(std::make_pair(std::distance(depthImgs.begin(), dit) + 1, dit->string()));
            ++dit;
            ++cit;
            ++lit;
        }
        else {
            // File names are not equal, so increment the lowest iterator
            ++tripleMin<fs::path>(dit, cit, lit);
        }
    }
}

std::vector<std::string> ImageAnalyzer::findScenesByLabel(const std::string& label) const {
    // TODO: implement
}

std::vector<std::string> ImageAnalyzer::findScenesByLabel(unsigned short labelValue) const {
    // TODO: implement
}

std::vector<std::string> ImageAnalyzer::findScenesByPlane(const std::vector<std::string>& validLabels, const cv::Vec3f& normal, float tolerance) const {

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
