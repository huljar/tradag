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
    // TODO: check depth/label image for single channel, check rgb image for color image
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

    // TODO: at end: check if inliers are within x radius of object (center of mass), sample again if not
    // TODO: when checking, cast a ray from object center of mass in direction of gravity (or direction of negative plane normal?) onto the mesh
    // TODO: when checking if inliers are still underneath the object, only check for the visible part of the object (underneath the covered parts is obviously no inlier)
    // TODO: when using objectMustBeUpright, disable angular restriction as soon as object is not moving anymore?
    // TODO: IDs for object selection in SceneAnalyzer (getMeshName/getObjectName function)
    ObjectDropResult execute();

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
