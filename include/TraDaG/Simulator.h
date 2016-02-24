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

    struct DropResult {
        DropResult()
            : status(DropStatus::UNKNOWN_ERROR), score(std::numeric_limits<float>::infinity())
        {
        }

        DropResult(DropStatus status, const cv::Mat& depthImage, const cv::Mat& rgbImage, float score = std::numeric_limits<float>::infinity())
            : status(status), depthImage(depthImage), rgbImage(rgbImage), score(score)
        {
        }

        DropStatus status;
        cv::Mat depthImage;
        cv::Mat rgbImage;
        float score;
    };

    // Constructors
    Simulator(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams);

    Simulator(const std::string& depthImagePath, const std::string& rgbImagePath, const CameraManager& cameraParams);

    // Destructor
    ~Simulator();

    // Move constructor
    Simulator(Simulator&& other);

    // Move assignment operator
    Simulator& operator=(Simulator&& other);

    /**
     * @brief createObject
     * @param meshName
     * @return
     */
    DroppableObject* createObject(const std::string& meshName);
    void destroyObject(DroppableObject* object);
    void destroyObject(unsigned int index);
    void destroyAllObjects();

    ObjectVec::iterator beginObjects();
    ObjectVec::const_iterator beginObjects() const;
    ObjectVec::iterator endObjects();
    ObjectVec::const_iterator endObjects() const;

    /**
     * @brief execute
     * @return
     */
    DropResult execute();

    RGBDScene* getRGBDScene() const;

    /**
     * @brief getGroundPlane
     * @return
     */
    GroundPlane getGroundPlane() const;
    /**
     * @brief setGroundPlane
     * @param groundPlane
     */
    void setGroundPlane(const GroundPlane& groundPlane);

    /**
     * @brief getMaxAttempts
     * @return
     */
    unsigned int getMaxAttempts() const;
    /**
     * @brief setMaxAttempts
     * @param maxAttempts
     */
    void setMaxAttempts(unsigned int maxAttempts);

    bool showPreviewWindow() const;
    void setShowPreviewWindow(bool showWindow);

    bool showPhysicsAnimation() const;
    void setShowPhysicsAnimation(bool showAnimation);

    bool debugMarkInlierSet() const;
    void setDebugMarkInlierSet(bool markInliers);

    bool debugDrawBulletShapes() const;
    void setDebugDrawBulletShapes(bool drawShapes);

    Auto<cv::Vec3f> getGravity() const;
    /**
     * @brief setGravity
     * @param gravity
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
