#ifndef IMAGEANALYZER_H
#define IMAGEANALYZER_H

#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

#include <map>
#include <string>
#include <vector>

namespace TraDaG {
    class ImageAnalyzer;
}

class TraDaG::ImageAnalyzer
{
public:
    ImageAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  unsigned int maxImages = 0);

    std::vector<std::string> findScenesByLabel(const std::string& label) const;
    std::vector<std::string> findScenesByLabel(unsigned short labelValue) const;

    std::vector<std::string> findScenesByPlane(const std::vector<std::string>& validLabels,
                                               const cv::Vec3f& normal, float tolerance) const;

    bool readImages(const std::string& fileName, cv::Mat& depthImg, cv::Mat& rgbImg, cv::Mat& labelImg) const;

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

    std::map<unsigned int, std::string> mImages;
};

#endif // IMAGEANALYZER_H
