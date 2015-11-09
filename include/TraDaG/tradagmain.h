#ifndef TRADAGMAIN_H
#define TRADAGMAIN_H

#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>
#include <TraDaG/imagelabeling.h>
#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace TraDaG {
    class TradagMain;
}

class TraDaG::TradagMain
{
public:
    TradagMain(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage,
               const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mapMode = MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LABELS_ON_DEPTH_IMAGE);

    TradagMain(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath,
               const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mapMode = MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LABELS_ON_DEPTH_IMAGE);

    ~TradagMain();

    void updateMesh();

    bool dropObjectIntoScene(const std::string& meshName, unsigned short planeLabelIndex, bool castShadows = true,
                             bool showPreviewWindow = false, bool showPhysicsAnimation = false,
                             const Ogre::Vector3& gravity = -700 * Ogre::Vector3::UNIT_Y,
                             const Automatable<Ogre::Vector3>& initialPosition = Automatable<Ogre::Vector3>(true),
                             const Automatable<Ogre::Matrix3>& initialRotation = Automatable<Ogre::Matrix3>(true),
                             const Ogre::Vector3& initialVelocity = Ogre::Vector3::ZERO,
                             Ogre::Real objectRestitution = 0.4, Ogre::Real objectFriction = 0.6, Ogre::Real objectMass = 1.0,
                             Ogre::Real planeRestitution = 0.1, Ogre::Real planeFriction = 0.9);

    cv::Mat getDepthImage();
    const cv::Mat getDepthImage() const;

    cv::Mat getRgbImage();
    const cv::Mat getRgbImage() const;

    cv::Mat getLabelImage();
    const cv::Mat getLabelImage() const;

    void setNewScene(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage);
    bool loadNewScene(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath);

    cv::Vec2f getDepthPrincipalPoint() const;
    void setDepthPrincipalPoint(const cv::Vec2f& principalPoint);

    cv::Vec2f getDepthFocalLength() const;
    void setDepthFocalLength(const cv::Vec2f& focalLength);

    cv::Vec2f getRgbPrincipalPoint() const;
    void setRgbPrincipalPoint(const cv::Vec2f& principalPoint);

    cv::Vec2f getRgbFocalLength() const;
    void setRgbFocalLength(const cv::Vec2f& focalLength);

    MapMode getMapMode() const;
    void setMapMode(MapMode mode);

    LabelMode getLabelMode() const;
    void setLabelMode(LabelMode mode);

private:
    void init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage,
              const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
              const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
              const cv::Matx33f& rotation, const cv::Vec3f& translation,
              MapMode mode, LabelMode labelMode);

    OgreWindow* mOgreWindow;
    RgbdObject* mRgbdObject;
    ImageLabeling* mImageLabeling;

};

#endif // TRADAGMAIN_H
