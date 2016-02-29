/************************************************************//**
 * @file
 *
 * @brief Simulator class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/OgreWindow.h>
#include <TraDaG/RGBDScene.h>
#include <TraDaG/util.h>

#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreVector3.h>

#include <opencv2/core/core.hpp>

#include <limits>
#include <random>
#include <string>
#include <vector>

namespace TraDaG {
    class Simulator;
}

/**
 * @brief Class that manages simulations for a single scene.
 *
 * This class allows you to create \ref DroppableObject "DroppableObject"s,
 * adjust simulation parameters and execute simulations.
 */
class TraDaG::Simulator
{
public:
    /**
     * @brief Enumerator describing the result of a simulation.
     */
    enum class DropStatus {
        SUCCESS, /**< Success, an optimal result was found. */
        MAX_ATTEMPTS_REACHED, /**< Success, but no optimal result was found within the allowed number of attempts. */
        PLANE_TOO_STEEP, /**< Error, the angle between the gravity vector and the plane normal is too large. */
        NO_OBJECTS, /**< Error, no objects to drop into the scene were specified. */
        PLANE_UNDEFINED, /**< Error, no plane was specified. */
        USER_DISCARDED, /**< Error, the user discarded the result (can only happen when preview window is enabled). */
        USER_ABORTED, /**< Error, the user aborted the whole simulation (can only happen when preview window is enabled). */
        UNKNOWN_ERROR /**< All other possible errors. */
    };

    /**
     * @brief Struct holding the result of a simulation.
     */
    struct DropResult {
        /**
         * @brief Default constructor.
         *
         * This constructs an empty result with an error status.
         */
        DropResult()
            : status(DropStatus::UNKNOWN_ERROR), score(std::numeric_limits<float>::infinity())
        {
        }

        /**
         * @brief Constructor.
         * @param status Status indicating success or, in case of an error, which error occurred.
         * @param depthImage The rendered depth image of the scene with the dropped objects inside.
         * @param rgbImage The rendered RGB image of the scene with the dropped objects inside.
         * @param score The score that was calculated for this result.
         */
        DropResult(DropStatus status, const cv::Mat& depthImage, const cv::Mat& rgbImage, float score = std::numeric_limits<float>::infinity())
            : status(status), depthImage(depthImage), rgbImage(rgbImage), score(score)
        {
        }

        /// The status of the result indicating success or, in case of an error, which error occurred.
        DropStatus status;
        /// The rendered depth image of the scene with the dropped objects inside.
        cv::Mat depthImage;
        /// The rendered RGB image of the scene with the dropped objects inside.
        cv::Mat rgbImage;
        /**
         * @brief The score that was calculated for this result. The lower this value, the closer the result is
         * to the desired parameters, with a value of 0 indicating an optimal result.
         */
        float score;
    };

    /**
     * @brief Constructor with images already in memory.
     * @param depthImage The depth image of the scene.
     * @param rgbImage The RGB image of the scene.
     * @param cameraParams The CameraManager instance holding the camera parameters for this scene.
     */
    Simulator(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams);

    /**
     * @brief Constructor with image file paths.
     * @param depthImagePath Path of the depth image file of the scene.
     * @param rgbImagePath Path of the RGB image file of the scene.
     * @param cameraParams The CameraManager instance holding the camera parameters for this scene.
     *
     * @throws std::runtime_error Thrown if the files cannot be read correctly.
     */
    Simulator(const std::string& depthImagePath, const std::string& rgbImagePath, const CameraManager& cameraParams);

    /**
     * @brief Destructor.
     */
    ~Simulator();

    /**
     * @brief Move constructor.
     * @param other The object to move from.
     */
    Simulator(Simulator&& other);

    /**
     * @brief Move assignment operator.
     * @param other The object to move from.
     * @return This object after the move operation.
     */
    Simulator& operator=(Simulator&& other);

    /**
     * @brief Create a new object that will be dropped into the scene.
     * @param meshName File name of the <em>.mesh</em> file to use as the object mesh (without path).
     * @return Pointer to the created object.
     *
     * This creates a new DroppableObject and registers it for future simulations. Use the returned pointer
     * to set object-specific simulation parameters.
     *
     * @note In order for the OGRE resource managers to find and load the mesh, ensure that the path
     * that the <em>.mesh</em> file resides in is included in the <em>config/resources.cfg</em>
     * configuration file.
     */
    DroppableObject* createObject(const std::string& meshName);
    /**
     * @brief Destroy a DroppableObject and unregister if with this Simulator.
     * @param object Pointer to the object to destroy (as returned by \ref createObject "createObject").
     */
    void destroyObject(DroppableObject* object);
    /**
     * @brief Destroy a DroppableObject and unregister if with this Simulator.
     * @param index Index of the object to destroy.
     */
    void destroyObject(unsigned int index);
    /**
     * @brief Destroy all \ref DroppableObject "DroppableObject"s registered with this Simulator.
     */
    void destroyAllObjects();

    /// Get an iterator to the beginning of the registered objects vector.
    ObjectVec::iterator beginObjects();
    /// Get a const_iterator to the beginning of the registered objects vector.
    ObjectVec::const_iterator beginObjects() const;
    /// Get an iterator to the @a past-the-end element of the registered objects vector.
    ObjectVec::iterator endObjects();
    /// Get a const_iterator to the @a past-the-end element of the registered objects vector.
    ObjectVec::const_iterator endObjects() const;

    /**
     * @brief Kick off the execution of a simulation.
     * @return A DropResult containing a status and the rendered images of the result.
     *
     * This function will start a simulation process with the current parameters of this Simulator
     * and the parameters set on the \ref DroppableObject "DroppableObject"s registered with this
     * Simulator. The simulation will run until an optimal result is found or the
     * \ref setMaxAttempts "maximum attempts" are reached. If a
     * \ref setShowPreviewWindow "preview window" is requested, the simulation will run until the
     * user decides to keep a result or to abort the simulation.
     */
    DropResult execute();

    /// Get the underlying scene object representing the 3D mesh of the scene.
    RGBDScene* getRGBDScene() const;

    /// Get the plane used for simulations.
    GroundPlane getGroundPlane() const;
    /// Set the plane to use for simulations.
    void setGroundPlane(const GroundPlane& groundPlane);

    /// Get the maximum number of attempts to perform if no optimal result is found.
    unsigned int getMaxAttempts() const;
    /// Set the maximum number of attempts to perform if no optimal result is found.
    void setMaxAttempts(unsigned int maxAttempts);

    /**
     * @brief Get if a preview window should be displayed for results.
     *
     * When this is @c true, a preview window will be displayed showing the result of the best simulation attempt.
     * The user can then choose whether to keep the image or discard it.
     */
    bool showPreviewWindow() const;
    /**
     * @brief Set if a preview window should be displayed for results.
     *
     * When this is @c true, a preview window will be displayed showing the result of the best simulation attempt.
     * The user can then choose whether to keep the image or discard it.
     */
    void setShowPreviewWindow(bool showWindow);

    /**
     * @brief Get if the object dropping should be animated in the preview window.
     *
     * When this is @c true, the dropping simulation of the physics engine will be shown in real-time for each attempt.
     *
     * @remarks This has no effect if no preview window will be shown.
     *
     * @sa showPreviewWindow
     * @sa setShowPreviewWindow
     */
    bool showPhysicsAnimation() const;
    /**
     * @brief Set if the object dropping should be animated in the preview window.
     *
     * When this is @c true, the dropping simulation of the physics engine will be shown in real-time for each attempt.
     *
     * @remarks This has no effect if no preview window will be shown.
     *
     * @note Settings this to @c true will cause @e each attempt to be animated in real-time. When setting a large number of
     * maximum attempts, this can take very long. It is mostly useful for debugging purposes.
     *
     * @sa getShowPreviewWindow
     * @sa setShowPreviewWindow
     * @sa getMaxAttempts
     * @sa setMaxAttempts
     */
    void setShowPhysicsAnimation(bool showAnimation);

    /**
     * @brief Get if the inlier vertices of the \ref setGroundPlane "ground plane" will be marked in the preview window.
     *
     * @remarks This has no effect if no preview window will be shown.
     */
    bool debugMarkInlierSet() const;
    /**
     * @brief Set if the inlier vertices of the \ref setGroundPlane "ground plane" will be marked in the preview window.
     *
     * @remarks This has no effect if no preview window will be shown.
     */
    void setDebugMarkInlierSet(bool markInliers);

    /**
     * @brief Get if the collision shapes of the \ref createObject "dropped objects" will be displayed in the preview window.
     *
     * @remarks This has no effect if no preview window will be shown.
     */
    bool debugDrawBulletShapes() const;
    /**
     * @brief Set if the collision shapes of the \ref createObject "dropped objects" will be displayed in the preview window.
     *
     * @remarks This has no effect if no preview window will be shown.
     */
    void setDebugDrawBulletShapes(bool drawShapes);

    /**
     * @brief Get gravity vector.
     * @return The gravity vector (if specified), and if it will be calculated automatically.
     */
    Auto<cv::Vec3f> getGravity() const;
    /**
     * @brief Set gravity vector.
     * @param The gravity vector (if not automatic), and if it should be calculated automatically.
     */
    void setGravity(const Auto<cv::Vec3f>& gravity);

private:
    Simulator();
    void init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams);

    Ogre::Vector3 computePosition(const std::vector<Ogre::Vector3>& inliers, const Ogre::Vector3& gravity);
    Ogre::Matrix3 computeRotation(float azimuth, const Ogre::Vector3& gravity) const;

    float distanceToIntervalSquared(float value, float min, float max) const;

    RGBDScene* mRGBDScene;

    ObjectVec mObjects;
    GroundPlane mGroundPlane;

    unsigned int mMaxAttempts;
    bool mShowPreviewWindow;
    bool mShowPhysicsAnimation;
    bool mMarkInlierSet;
    bool mDrawBulletShapes;
    Auto<cv::Vec3f> mGravity;

    std::default_random_engine mRandomEngine;

};

#endif // SIMULATOR_H
