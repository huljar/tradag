/************************************************************//**
 * @file
 *
 * @brief SceneAnalyzer class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef SCENEANALYZER_H
#define SCENEANALYZER_H

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
#include <queue>
#include <random>
#include <string>
#include <vector>

namespace TraDaG {
    class SceneAnalyzer;
}

/**
 * @brief Manages a collection of scenes consisting of depth, RGB and label images.
 *
 * This class manages your data set consisting of depth, RGB and label images and
 * can be used to search and iterate through the scenes with specific constraints.
 * It also provides functions to quickly create the other necessary classes when
 * you have found a scene that you want to use for a simulation.
 *
 * %SceneAnalyzer assigns numerical @a IDs to the scenes from the data set, in
 * ascending order of the alphabetically sorted depth images. These IDs can be used
 * with various member functions to identify a specific scene.
 */
class TraDaG::SceneAnalyzer
{
public:
    /// Map of scene IDs to file names.
    typedef std::map<unsigned int, std::string> FileMap;
    /// Map of scene IDs to the depth, RGB and label images of the scenes.
    typedef std::map<unsigned int, std::array<cv::Mat, 3>> MatMap;
    /// Queue of scene IDs used to manage the image cache of this class.
    typedef std::queue<unsigned int> MatQueue;

    /**
     * @brief <a href="http://www.cplusplus.com/reference/iterator/InputIterator/">Input iterator</a>
     * for iterating over scenes containing specific labels.
     */
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

    /**
     * @brief <a href="http://www.cplusplus.com/reference/iterator/InputIterator/">Input iterator</a>
     * for iterating over scenes containing specific planes.
     */
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

    /**
     * @brief beginByLabel
     * @param labels
     * @param randomizeOrder
     * @return
     */
    LabelIterator beginByLabel(const std::vector<std::string>& labels, bool randomizeOrder = false);
    /**
     * @brief beginByLabel
     * @param label
     * @param randomizeOrder
     * @return
     */
    LabelIterator beginByLabel(const std::string& label, bool randomizeOrder = false);
    /**
     * @brief endByLabel
     * @return
     */
    LabelIterator endByLabel();

    /**
     * @brief beginByPlane
     * @param labels
     * @param normal
     * @param tolerance
     * @param minDistance
     * @param maxDistance
     * @param computePlaneIfNoPlaneInfoFile
     * @param randomizeOrder
     * @return
     */
    PlaneIterator beginByPlane(const std::vector<std::string>& labels,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    /**
     * @brief beginByPlane
     * @param label
     * @param normal
     * @param tolerance
     * @param minDistance
     * @param maxDistance
     * @param computePlaneIfNoPlaneInfoFile
     * @param randomizeOrder
     * @return
     */
    PlaneIterator beginByPlane(const std::string& label,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    /**
     * @brief endByPlane
     * @return
     */
    PlaneIterator endByPlane();

    bool precomputePlaneInfoForScene(unsigned int sceneID, const std::string& label,
                                     const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);
    bool precomputePlaneInfoForAllScenes(const std::string& label,
                                         const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    bool readImages(unsigned int sceneID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage);
    void clearCache();

    ImageLabeling createImageLabeling(unsigned int sceneID);

    /**
     * @brief Construct a Simulator instance for a specific scene.
     * @param sceneID The ID of the scene, as returned by the search results of this class.
     * @return A Simulator instance for the specified scene.
     *
     * This function is a shortcut for comfortable construction of a Simulator instance for a scene.
     *
     * @remarks Alternatively, you can always construct the Simulator yourself.
     */
    Simulator createSimulator(unsigned int sceneID);
    /**
     * @brief Construct a Simulator instance for a specific scene and set its ground plane.
     * @param sceneID The ID of the scene, as returned by the search results of this class.
     * @param plane The plane to register with the constructed Simulator.
     * @return A Simulator instance for the specified scene, with the ground plane already set.
     *
     * This function is a shortcut for comfortable construction of a Simulator instance for a scene.
     *
     * @remarks Alternatively, you can always construct the Simulator yourself and set the plane manually using Simulator::setGroundPlane.
     */
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

    size_t getCacheSize() const;
    void setCacheSize(size_t cacheSize);

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
    MatQueue mMatQueue;
    size_t mCacheSize;

    std::default_random_engine mRandomEngine;
};

#endif // SCENEANALYZER_H
