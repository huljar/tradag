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
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref LabelIterator::operator++() "operator++").
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

        /// Prefix increment operator.
        self_type operator++() { step(); return *this; }
        /// Postfix increment operator.
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
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref PlaneIterator::operator++() "operator++").
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

        /// Prefix increment operator.
        self_type operator++() { step(); return *this; }
        /// Postfix increment operator.
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

    /**
     * @brief Constructor (without plane directory).
     * @param depthDirPath Path to the directory containing the depth images of the data set.
     * @param rgbDirPath Path to the directory containing the RGB images of the data set.
     * @param labelDirPath Path to the directory containing the label images of the data set.
     * @param cameraParams A CameraManager instance containing all the camera parameters which were used to record the data set.
     * @param labelMap A mapping of string labels to 16 bit unsigned label values. For the <em>NYU Depth V1</em> and <em>NYU Depth
     * V2</em> data sets, sensible label maps are predefined in @c util.h which can be used here.
     * @param maxScenes The maximum number of scenes to load from the data set. If this number is smaller than the number of
     * scenes in the data set, some scenes will not be loaded, and the loaded scenes are selected randomly from the data set.
     * Set this to 0 to specify no limit.
     *
     * @note Depth, color and label images are matched to the same scene if they have the same file name in their respective
     * directories. Images that do not have a match in the other directories are ignored (note that ignored depth images still
     * affect the ID assignments, though).
     *
     * @remarks This constructor does not specify a plane directory. If you plan on using precomputed
     * planes or precomputing new planes, you will need to specify the plane directory first with
     * \ref SceneAnalyzer::setPlanePath "setPlanePath" (or just use the
     * \ref SceneAnalyzer(const std::string&, const std::string&, const std::string&, const std::string&, const CameraManager&, const LabelMap&, unsigned int) "other constructor").
     */
    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    /**
     * @brief Constructor (with plane directory).
     * @param depthDirPath Path to the directory containing the depth images of the data set.
     * @param rgbDirPath Path to the directory containing the RGB images of the data set.
     * @param labelDirPath Path to the directory containing the label images of the data set.
     * @param planeDirPath Path to the directory that will be used to load/store precomputed planes from/to.
     * @param cameraParams A CameraManager instance containing all the camera parameters which were used to record the data set.
     * @param labelMap A mapping of string labels to 16 bit unsigned label values. For the <em>NYU Depth V1</em> and <em>NYU Depth
     * V2</em> data sets, sensible label maps are predefined in @c util.h which can be used here.
     * @param maxScenes The maximum number of scenes to load from the data set. If this number is smaller than the number of
     * scenes in the data set, some scenes will not be loaded, and the loaded scenes are selected randomly from the data set.
     * Set this to 0 to specify no limit.
     *
     * @note Depth, color and label images are matched to the same scene if they have the same file name in their respective
     * directories. Images that do not have a match in the other directories are ignored (note that ignored depth images still
     * affect the ID assignments, though).
     *
     * @remarks If you don't want to precompute planes or make use of existing precomputed planes, you can use the
     * \ref SceneAnalyzer(const std::string&, const std::string&, const std::string&, const CameraManager&, const LabelMap&, unsigned int) "other constructor".
     */
    SceneAnalyzer(const std::string& depthDirPath, const std::string& rgbDirPath, const std::string& labelDirPath,
                  const std::string& planeDirPath,
                  const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxScenes = 0);

    /**
     * @brief Search for scenes containing a specific label (with enough pixels).
     * @param labels Vector of allowed labels (a scene will be returned if it contains any of those labels).
     * @return Scene ID and the actual found label for each resulting scene.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @sa beginByLabel
     */
    std::map<unsigned int, std::string> findScenesByLabel(const std::vector<std::string>& labels);
    /**
     * @brief Search for scenes containing a specific label (with enough pixels).
     * @param label Label to search for.
     * @return Scene ID and the actual found label for each resulting scene. These labels will always be the same as the
     * the specified label, and are only returned for consistency with the
     * \ref findScenesByLabel(const std::vector<std::string>&) "other overload".
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @sa beginByLabel
     */
    std::map<unsigned int, std::string> findScenesByLabel(const std::string& label);

    /**
     * @brief Search for scenes containing a specific plane (within a tolerance).
     * @param labels Vector of allowed labels (a scene will be considered if it contains any of those labels).
     * @param normal Normal vector of the searched plane. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @param minDistance Minimum distance of the plane to the camera.
     * @param maxDistance Maximum distance of the plane to the camera.
     * @param computePlaneIfNoPlaneInfoFile Specify if the search algorithm should attempt to fit a new plane if none of the
     * precomputed planes match the given constraints.
     * @return Scene ID and the actual computed plane for each resulting scene.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @warning This function will return a plane for each result in the data set, which means that
     * for big data sets, this can fill up a lot of memory, depending on the number of results. It
     * is advised to use the iterator interface instead.
     *
     * @sa beginByPlane
     */
    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::vector<std::string>& labels,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                                          bool computePlaneIfNoPlaneInfoFile = false);

    /**
     * @brief Search for scenes containing a specific plane (within a tolerance).
     * @param label Label that scenes must contain to be considered.
     * @param normal Normal vector of the searched plane. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @param minDistance Minimum distance of the plane to the camera.
     * @param maxDistance Maximum distance of the plane to the camera.
     * @param computePlaneIfNoPlaneInfoFile Specify if the search algorithm should attempt to fit a new plane if none of the
     * precomputed planes match the given constraints.
     * @return Scene ID and the actual computed plane for each resulting scene.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @warning This function will return a plane for each result in the data set, which means that
     * for big data sets, this can fill up a lot of memory, depending on the number of results. It
     * is advised to use the iterator interface instead.
     *
     * @sa beginByPlane
     */
    std::map<unsigned int, GroundPlane> findScenesByPlane(const std::string& label,
                                                          const cv::Vec3f& normal, float tolerance,
                                                          unsigned short minDistance = 0,
                                                          unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                                          bool computePlaneIfNoPlaneInfoFile = false);

    /**
     * @brief Get a LabelIterator to iterate through scenes containing specific labels (with enough pixels).
     * @param labels Vector of allowed labels (a scene will be considered if it contains any of those labels).
     * @param randomizeOrder Specify if the iterator should iterate through the scenes in a random order.
     * @return The created iterator.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref LabelIterator::operator++() "operator++").
     */
    LabelIterator beginByLabel(const std::vector<std::string>& labels, bool randomizeOrder = false);
    /**
     * @brief Get a LabelIterator to iterate through scenes containing specific labels (with enough pixels).
     * @param label Label that scenes must contain to be considered.
     * @param randomizeOrder Specify if the iterator should iterate through the scenes in a random order.
     * @return The created iterator.
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref LabelIterator::operator++() "operator++").
     */
    LabelIterator beginByLabel(const std::string& label, bool randomizeOrder = false);
    /**
     * @brief Get a LabelIterator pointing to the @a past-the-end element.
     * @return The created iterator.
     */
    LabelIterator endByLabel();

    /**
     * @brief Get a PlaneIterator to iterate through scenes containing a specific plane (within a tolerance).
     * @param labels Vector of allowed labels (a scene will be considered if it contains any of those labels).
     * @param normal Normal vector of the searched plane. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @param minDistance Minimum distance of the plane to the camera.
     * @param maxDistance Maximum distance of the plane to the camera.
     * @param computePlaneIfNoPlaneInfoFile Specify if the search algorithm should attempt to fit a new plane if none of the
     * precomputed planes match the given constraints.
     * @param randomizeOrder Specify if the iterator should iterate through the scenes in a random order.
     * @return The created iterator.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref PlaneIterator::operator++() "operator++").
     */
    PlaneIterator beginByPlane(const std::vector<std::string>& labels,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    /**
     * @brief Get a PlaneIterator to iterate through scenes containing a specific plane (within a tolerance).
     * @param label Label that scenes must contain to be considered.
     * @param normal Normal vector of the searched plane. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @param minDistance Minimum distance of the plane to the camera.
     * @param maxDistance Maximum distance of the plane to the camera.
     * @param computePlaneIfNoPlaneInfoFile Specify if the search algorithm should attempt to fit a new plane if none of the
     * precomputed planes match the given constraints.
     * @param randomizeOrder Specify if the iterator should iterate through the scenes in a random order.
     * @return The created iterator.
     *
     * @remarks Currently, a label is considered to be contained in a scene if at least 10 pixels have this label assigned.
     *
     * @remarks The scene search is performed @a lazily, i.e. the next element is only searched
     * when the iterator is incremented (using \ref PlaneIterator::operator++() "operator++").
     */
    PlaneIterator beginByPlane(const std::string& label,
                               const cv::Vec3f& normal, float tolerance,
                               unsigned short minDistance = 0,
                               unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                               bool computePlaneIfNoPlaneInfoFile = false, bool randomizeOrder = false);
    /**
     * @brief Get a PlaneIterator pointing to the @a past-the-end element.
     * @return The created iterator.
     */
    PlaneIterator endByPlane();

    /**
     * @brief Precompute plane information for a specific scene and store it to <em>.planeinfo</em> files.
     * @param sceneID ID of the scene to find planes in.
     * @param label Label for which planes shall be searched.
     * @param normal Plane normal to search for. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @return Status indicating success/failure/...
     */
    bool precomputePlaneInfoForScene(unsigned int sceneID, const std::string& label,
                                     const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);
    /**
     * @brief Precompute plane information for all scenes and store it to <em>.planeinfo</em> files.
     * @param label Label for which planes shall be searched.
     * @param normal Plane normal to search for. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance (in degrees) of the normal vector.
     * @return Status indicating success/failure/...
     */
    bool precomputePlaneInfoForAllScenes(const std::string& label,
                                         const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    /**
     * @brief Get the depth, RGB and label images of a specific scene.
     * @param sceneID ID of the scene.
     * @param depthImage Output parameter for the depth image.
     * @param rgbImage Output parameter for the RGB image.
     * @param labelImage Output parameter for the label image.
     * @return True if the images were read correctly, otherwise false.
     *
     * @remarks @a %SceneAnalyzer has an internal cache if the recently used images. If this scene is in the
     * cache, the images from the cache will be returned.
     */
    bool readImages(unsigned int sceneID, cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& labelImage);
    /// Clear the internal image cache.
    void clearCache();

    /**
     * @brief Construct an ImageLabeling instance for a specific scene.
     * @param sceneID The ID of the scene, as returned by the search results of this class.
     * @return An ImageLabeling instance for the specified scene.
     *
     * This function is a shortcut for easy construction of an ImageLabeling instance for a scene.
     *
     * @remarks Alternatively, you can always construct the ImageLabeling yourself.
     */
    ImageLabeling createImageLabeling(unsigned int sceneID);

    /**
     * @brief Construct a Simulator instance for a specific scene.
     * @param sceneID The ID of the scene, as returned by the search results of this class.
     * @return A Simulator instance for the specified scene.
     *
     * This function is a shortcut for easy construction of a Simulator instance for a scene.
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
     * This function is a shortcut for easy construction of a Simulator instance for a scene.
     *
     * @remarks Alternatively, you can always construct the Simulator yourself and set the plane manually using Simulator::setGroundPlane.
     */
    Simulator createSimulator(unsigned int sceneID, const GroundPlane& plane);

    /**
     * @brief Get the names of all <em>.planeinfo</em> files found for a specific scene.
     * @note In order to find previously stored files, the file names should not be modified.
     */
    std::vector<std::string> getPlaneInfoFileNames(unsigned int sceneID);

    /// Get an iterator to the internal scene map for a specific scene.
    FileMap::const_iterator getSceneIterator(unsigned int sceneID) const;
    /// Get the file names of the images for a specific scene.
    std::string getFileName(unsigned int sceneID) const;

    /// Get the number of scenes loaded by this %SceneAnalyzer.
    size_t getNumScenes() const;

    /// Get path of the depth image directory, as specified in the constructor.
    std::string getDepthPath() const;
    /// Get path of the RGB image directory, as specified in the constructor.
    std::string getRGBPath() const;
    /// Get path of the label image directory, as specified in the constructor.
    std::string getLabelPath() const;

    /// Get plane path.
    std::string getPlanePath() const;
    /// Set plane path.
    void setPlanePath(const std::string& planePath);

    /// Get the camera manager used by this %SceneAnalyzer.
    CameraManager getCameraManager() const;
    /// Get the camera manager used by this %SceneAnalyzer.
    const CameraManager& cameraManager() const;

    /// Get the label map used by this %SceneAnalyzer.
    LabelMap getLabelMap() const;
    /// Get the label map used by this %SceneAnalyzer.
    const LabelMap& labelMap() const;

    /// Get the maximum number of scenes that may be stored in the interal cache.
    size_t getCacheSize() const;
    /**
     * @brief Set the maximum number of scenes that may be stored in the interal cache.
     *
     * Lower this number if you want the %SceneAnalyzer to use up less memory, or raise it
     * for occasional faster image access times (and less reads from disk).
     */
    void setCacheSize(size_t cacheSize);

    /// Get the random engine used internally.
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
