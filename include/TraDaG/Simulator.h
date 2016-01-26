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

#include <random>
#include <string>
#include <vector>

namespace TraDaG {
    class Simulator;
}

class TraDaG::Simulator
{
public:
    enum class DropStatus { SUCCESS, PLANE_TOO_STEEP, MAX_ATTEMPTS_REACHED, NO_OBJECTS, PLANE_UNDEFINED, USER_ABORTED, UNKNOWN_ERROR };

    struct DropResult {
        DropResult(DropStatus status, const cv::Mat& depthImage, const cv::Mat& rgbImage)
            : status(status), depthImage(depthImage), rgbImage(rgbImage)
        {
        }

        DropStatus status;
        cv::Mat depthImage;
        cv::Mat rgbImage;
    };

    // TODO: check depth/label image for single channel, check rgb image for color image
    // TODO: azimuth in degrees
    // Constructors
    Simulator(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams);

    Simulator(const std::string& depthImagePath, const std::string& rgbImagePath, const CameraManager& cameraParams);

    // Destructor
    ~Simulator();

    // Move constructor
    Simulator(Simulator&& other);

    // Move assignment operator
    Simulator& operator=(Simulator&& other);

    DroppableObject* createObject(const std::string& meshName);
    void destroyObject(DroppableObject* object);
    void destroyObject(unsigned int index);
    void destroyAllObjects();

    ObjectVec::iterator beginObjects();
    ObjectVec::const_iterator beginObjects() const;
    ObjectVec::iterator endObjects();
    ObjectVec::const_iterator endObjects() const;

    DropResult execute();

    RGBDScene* getRGBDScene() const;

    GroundPlane getGroundPlane() const;
    void setGroundPlane(const GroundPlane& groundPlane);

    unsigned int getMaxAttempts() const;
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
    void setGravity(const Auto<cv::Vec3f>& gravity);

private:
    Simulator();
    void init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams);

    Ogre::Vector3 computePosition(const std::vector<Ogre::Vector3>& inliers, const Ogre::Vector3& gravity);
    Ogre::Matrix3 computeRotation(const float azimuth, const Ogre::Vector3& gravity) const;

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
