#ifndef TRADAGMAIN_H
#define TRADAGMAIN_H

#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>
#include <TraDaG/imagelabeling.h>
#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <random>
#include <vector>

namespace TraDaG {
    class TradagMain;
}

class TraDaG::TradagMain
{
public:
    // TODO: check depth/label image for single channel, check rgb image for color image
    TradagMain(const cv::Mat& depthImage, const cv::Mat& rgbImage,
               const cv::Mat& labelImage, const LabelMap& labelMap,
               const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mapMode = MM_MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LM_DEPTH_IMAGE);

    TradagMain(const std::string& depthImagePath, const std::string& rgbImagePath,
               const std::string& labelImagePath, const LabelMap& labelMap,
               const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mapMode = MM_MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LM_DEPTH_IMAGE);

    // no copy
    TradagMain(const TradagMain&) = delete;

    // no assign
    TradagMain& operator=(const TradagMain&) = delete;

    ~TradagMain();

    void updateMesh();

    // TODO: define angle of plane to camera and tolerance (in separate interface)
    // TODO: at end: check if inliers are within x radius of object (center of mass), sample again if not
    // TODO: when checking, cast a ray from object center of mass in direction of gravity (or direction of negative plane normal?) onto the mesh
    // TODO: prevent setting a value for fraction covered which is too large
    // TODO: when checking if inliers are still underneath the object, only check for the visible part of the object (underneath the covered parts is obviously no inlier)
    // TODO: when using objectMustBeUpright, disable angular restriction as soon as object is not moving anymore?
    // TODO: setVerbose parameter, detailed log messages if enabled
    ObjectDropResult dropObjectIntoScene(const std::string& meshName, const Auto<PlaneFitResult>& plane, const std::string& planeLabel,
                                         const Auto<cv::Vec3f>& initialPosition = Auto<cv::Vec3f>(true),
                                         const Auto<cv::Matx33f>& initialRotation = Auto<cv::Matx33f>(true),
                                         const float initialAzimuth = 0,
                                         const cv::Vec3f& initialVelocity = cv::Vec3f(0, 0, 0),
                                         const cv::Vec3f& initialTorque = cv::Vec3f(0, 0, 0));

    cv::Mat getDepthImage();
    const cv::Mat getDepthImage() const;

    cv::Mat getRgbImage();
    const cv::Mat getRgbImage() const;

    cv::Mat getLabelImage();
    const cv::Mat getLabelImage() const;

    LabelMap getLabelMap() const;

    void setNewScene(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage, const LabelMap& labelMap);
    bool loadNewScene(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath, const LabelMap& labelMap);

    cv::Vec2f getDepthPrincipalPoint() const;
    void setDepthPrincipalPoint(const cv::Vec2f& principalPoint);
    void setDepthPrincipalPoint(float x, float y);

    cv::Vec2f getDepthFocalLength() const;
    void setDepthFocalLength(const cv::Vec2f& focalLength);
    void setDepthFocalLength(float x, float y);

    cv::Vec2f getRgbPrincipalPoint() const;
    void setRgbPrincipalPoint(const cv::Vec2f& principalPoint);
    void setRgbPrincipalPoint(float x, float y);

    cv::Vec2f getRgbFocalLength() const;
    void setRgbFocalLength(const cv::Vec2f& focalLength);
    void setRgbFocalLength(float x, float y);

    cv::Matx33f getRotation() const;
    void setRotation(const cv::Matx33f& rotation);

    cv::Vec3f getTranslation() const;
    void setTranslation(const cv::Vec3f& translation);
    void setTranslation(float x, float y, float z);

    MapMode getMapMode() const;
    void setMapMode(MapMode mode);

    LabelMode getLabelMode() const;
    void setLabelMode(LabelMode mode);

    float getObjectScale() const;
    void setObjectScale(float scale);

    bool objectMustBeUpright() const;
    void setObjectMustBeUpright(bool upright);

    Auto<float> getObjectCoveredFraction() const;
    void setObjectCoveredFraction(const Auto<float>& covered);

    bool objectCastShadows() const;
    void setObjectCastShadows(bool castShadows);

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

    float getObjectRestitution() const;
    void setObjectRestitution(float restitution);

    float getObjectFriction() const;
    void setObjectFriction(float friction);

    float getPlaneRestitution() const;
    void setPlaneRestitution(float restitution);

    float getPlaneFriction() const;
    void setPlaneFriction(float friction);

private:
    TradagMain();
    void init(const cv::Mat& depthImage, const cv::Mat& rgbImage,
              const cv::Mat& labelImage, const LabelMap& labelMap,
              const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
              const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
              const cv::Matx33f& rotation, const cv::Vec3f& translation,
              MapMode mode, LabelMode labelMode);

    Ogre::Vector3 computePosition(const std::vector<Ogre::Vector3>& inliers, const Ogre::Vector3& gravity);
    Ogre::Matrix3 computeRotation(const float azimuth, const Ogre::Vector3& gravity) const;

    Ogre::Matrix3 convertCvMatToOgreMat(const cv::Matx33f& mat) const;
    cv::Matx33f convertOgreMatToCvMat(const Ogre::Matrix3& mat) const;

    OgreWindow* mOgreWindow;
    RgbdObject* mRgbdObject;
    ImageLabeling* mImageLabeling;

    float mObjectScale;
    bool mObjectMustBeUpright;
    Auto<float> mObjectCoveredFraction;
    bool mObjectCastShadows;
    unsigned int mMaxAttempts;
    bool mShowPreviewWindow;
    bool mShowPhysicsAnimation;
    bool mMarkInlierSet;
    bool mDrawBulletShapes;
    Auto<cv::Vec3f> mGravity;
    float mObjectRestitution;
    float mObjectFriction;
    float mPlaneRestitution;
    float mPlaneFriction;

    std::default_random_engine mRandomEngine;

};

#endif // TRADAGMAIN_H
