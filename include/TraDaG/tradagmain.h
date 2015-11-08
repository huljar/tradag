#ifndef TRADAGMAIN_H
#define TRADAGMAIN_H

#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>
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
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(0, 0), const cv::Vec2f& rgbFocalLength = cv::Vec2f(0, 0),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mapMode = MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LABELS_ON_DEPTH_IMAGE);

    TradagMain(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath,
               const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
               const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(0, 0), const cv::Vec2f& rgbFocalLength = cv::Vec2f(0, 0),
               const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
               MapMode mode = MAPPED_RGB_TO_DEPTH, LabelMode labelMode = LABELS_ON_DEPTH_IMAGE);

    ~TradagMain();

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

//    bool autoGravity() const;
//    void setAutoGravity(bool autoGravity);

//    cv::Vec3f getGravityVector() const;
//    void setGravityVector(const cv::Vec3f gravityVector);

//    cv::Vec3f getInitialVelocity() const;
//    void setInitialVelocity(const cv::Vec3f& initialVelocity);

//    bool adaptVelocityDirectionToGravity() const;
//    void setAdaptVelocityDirectionToGravity(bool adaptVelocityDirectionToGravity);

//    bool previewInWindow() const;
//    void setPreviewInWindow(bool previewInWindow);

//    bool showObjectAnimation() const;
//    void setShowObjectAnimation(bool showObjectAnimation);

private:
    void init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage,
              const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
              const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
              const cv::Matx33f& rotation, const cv::Vec3f& translation,
              MapMode mode, LabelMode labelMode);

    OgreWindow* mOgreWindow;
    RgbdObject* mRgbdObject;

    bool mPreviewInWindow;
    bool mShowObjectAnimation;

    const std::string mObjName = "sceneRgbdEntity";

};

#endif // TRADAGMAIN_H
