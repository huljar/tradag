#ifndef IMAGEANALYZER_H
#define IMAGEANALYZER_H

#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

#include <array>
#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace TraDaG {
    class ImageAnalyzer;
}

class TraDaG::ImageAnalyzer
{
public:
    typedef std::map<unsigned int, std::string> FileMap;
    typedef std::map<unsigned int, std::array<cv::Mat, 3>> MatMap;

    ImageAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  unsigned int maxImages = 0);

    std::vector<unsigned int> findScenesByLabel(const std::string& label);
    std::vector<unsigned int> findScenesByLabel(unsigned short labelValue);

    std::vector<unsigned int> findScenesByPlane(const std::vector<std::string>& validLabels,
                                               const cv::Vec3f& normal, float tolerance);

    bool readImages(unsigned int imageID, cv::Mat& depthImg, cv::Mat& rgbImg, cv::Mat& labelImg);

    std::string getFileName(unsigned int imageID) const;

    std::string getDepthPath() const;
    std::string getRGBPath() const;
    std::string getLabelPath() const;

protected:
    template<typename T>
    typename std::vector<T>::iterator& tripleMin(typename std::vector<T>::iterator& first,
                                                 typename std::vector<T>::iterator& second,
                                                 typename std::vector<T>::iterator& third) const;

    boost::filesystem::path mDepthPath;
    boost::filesystem::path mRGBPath;
    boost::filesystem::path mLabelPath;

    FileMap mImages;
    MatMap mMats;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGEANALYZER_H
