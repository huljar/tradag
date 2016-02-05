#ifndef IMAGEANALYZER_H
#define IMAGEANALYZER_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/ImageLabeling.h>
#include <TraDaG/Simulator.h>
#include <TraDaG/util.h>

#include <OGRE/OgreMath.h>
#include <OGRE/OgreVector3.h>

#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

#include <array>
#include <iterator>
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

    // Iterators
    class LabelIterator {
    public:
        typedef LabelIterator self_type;
        typedef std::pair<unsigned int, std::string> value_type;
        typedef value_type& reference;
        typedef value_type* pointer;
        typedef std::input_iterator_tag iterator_category;
        typedef int difference_type;

        LabelIterator(FileMap::const_iterator begin, FileMap::const_iterator end, SceneAnalyzer* parent,
                      const std::vector<std::string>& labels, bool randomize);
        LabelIterator(FileMap::const_iterator end, SceneAnalyzer* parent);
        LabelIterator(const LabelIterator& other);
        LabelIterator& operator=(const LabelIterator& other);

        self_type operator++() { step(); return *this; }
        self_type operator++(int) { self_type tmp = *this; step(); return tmp; }
        const value_type& operator*() { return mCurrentElem; }
        const value_type* operator->() { return &mCurrentElem; }
        bool operator==(const self_type& rhs) { return mFileIter == rhs.mFileIter; }
        bool operator!=(const self_type& rhs) { return mFileIter != rhs.mFileIter; }

    private:
        void step();
        bool updateCurrent();

        FileMap::const_iterator mFileIter;
        FileMap::const_iterator mFileEnd;
        SceneAnalyzer* mParent;
        std::vector<std::string> mLabels;

        bool mRandomize;
        std::vector<unsigned int> mAllIDs;
        std::vector<unsigned int>::const_iterator mIDIter;
        std::vector<unsigned int>::const_iterator mIDEnd;

        value_type mCurrentElem;
    };

    class PlaneIterator {
    public:
        typedef PlaneIterator self_type;
        typedef std::pair<unsigned int, GroundPlane> value_type;
        typedef value_type& reference;
        typedef value_type* pointer;
        typedef std::input_iterator_tag iterator_category;
        typedef int difference_type;

        PlaneIterator(FileMap::const_iterator begin, FileMap::const_iterator end, SceneAnalyzer* parent,
                      const std::vector<std::string>& labels, const cv::Vec3f& normal, float tolerance,
                      unsigned short minDistance, unsigned short maxDistance,
                      bool computeNoFile, bool randomize);
        PlaneIterator(FileMap::const_iterator end, SceneAnalyzer* parent);
        PlaneIterator(const PlaneIterator& other);
        PlaneIterator& operator=(const PlaneIterator& other);

        self_type operator++() { step(); return *this; }
        self_type operator++(int) { self_type tmp = *this; step(); return tmp; }
        const value_type& operator*() { return mCurrentElem; }
        const value_type* operator->() { return &mCurrentElem; }
        bool operator==(const self_type& rhs) { return mFileIter == rhs.mFileIter; }
        bool operator!=(const self_type& rhs) { return mFileIter != rhs.mFileIter; }

    private:
        void step();
        bool updateCurrent();

        FileMap::const_iterator mFileIter;
        FileMap::const_iterator mFileEnd;
        SceneAnalyzer* mParent;
        std::vector<std::string> mLabels;
        Ogre::Vector3 mNormal;
        Ogre::Degree mTolerance;
        unsigned short mMinDistance;
        unsigned short mMaxDistance;
        bool mComputeNoFile;

        bool mRandomize;
        std::vector<unsigned int> mAllIDs;
        std::vector<unsigned int>::const_iterator mIDIter;
        std::vector<unsigned int>::const_iterator mIDEnd;

        value_type mCurrentElem;
    };

    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const std::string& planeDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    std::map<unsigned int, std::string> findScenesByLabel(const std::vector<std::string>& labels);
    std::map<unsigned int, std::string> findScenesByLabel(const std::string& label);

    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::vector<std::string>& labels,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                                          bool computePlaneIfNoPlaneInfoFile = false);

    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::string& label,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                                          bool computePlaneIfNoPlaneInfoFile = false);

    LabelIterator beginByLabel(const std::vector<std::string>& labels, bool randomizeOrder = false);
    LabelIterator beginByLabel(const std::string& label, bool randomizeOrder = false);
    LabelIterator endByLabel();

    PlaneIterator beginByPlane(const std::vector<std::string>& labels,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    PlaneIterator beginByPlane(const std::string& label,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    PlaneIterator endByPlane();

    bool precomputePlaneInfoForScene(unsigned int sceneID, const std::string& label,
                                     const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);
    bool precomputePlaneInfoForAllScenes(const std::string& label,
                                         const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    bool readImages(unsigned int sceneID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage);
    void clearCache();

    ImageLabeling createImageLabeling(unsigned int sceneID);
    Simulator createSimulator(unsigned int sceneID);
    Simulator createSimulator(unsigned int sceneID, const GroundPlane& plane);

    std::vector<std::string> getPlaneInfoFileNames(unsigned int sceneID);

    FileMap::const_iterator getSceneIterator(unsigned int sceneID) const;
    std::string getFileName(unsigned int sceneID) const;

    size_t getNumScenes() const;

    std::string getDepthPath() const;
    std::string getRGBPath() const;
    std::string getLabelPath() const;

    std::string getPlanePath() const;
    void setPlanePath(const std::string& planePath);

    CameraManager getCameraManager() const;
    const CameraManager& cameraManager() const;

    LabelMap getLabelMap() const;
    const LabelMap& labelMap() const;

    std::default_random_engine& randomEngine();

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
