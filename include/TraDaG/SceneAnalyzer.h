#ifndef IMAGEANALYZER_H
#define IMAGEANALYZER_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/ImageLabeling.h>
#include <TraDaG/Simulator.h>
#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

#include <array>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace TraDaG {
    class SceneAnalyzer;
}

class TraDaG::SceneAnalyzer
{
public:
    typedef std::map<unsigned int, std::string> FileMap;
    typedef std::map<unsigned int, std::array<cv::Mat, 3>> MatMap;

    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const std::string& planeDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    std::vector<unsigned int> findScenesByLabel(const std::vector<std::string>& labels);
    std::vector<unsigned int> findScenesByLabel(const std::string& label);

    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::vector<std::string>& labels,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max());

    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::string& label,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max());

    bool readImages(unsigned int sceneID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage);
    void clearCache();

    ImageLabeling createImageLabeling(unsigned int sceneID);
    Simulator createSimulator(unsigned int sceneID);
    Simulator createSimulator(unsigned int sceneID, const GroundPlane& plane);

    std::string getFileName(unsigned int sceneID) const;

    std::string getDepthPath() const;
    std::string getRGBPath() const;
    std::string getLabelPath() const;

    CameraManager getCameraManager() const;

    LabelMap getLabelMap() const;

protected:
    template<typename T>
    typename T::iterator& tripleMin(typename T::iterator& first,
                                    typename T::iterator& second,
                                    typename T::iterator& third) const;

    boost::filesystem::path mDepthPath;
    boost::filesystem::path mRGBPath;
    boost::filesystem::path mLabelPath;
    boost::filesystem::path mPlanePath;

    CameraManager mCameraManager;

    LabelMap mLabelMap;

    FileMap mScenes;
    MatMap mMats;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGEANALYZER_H
