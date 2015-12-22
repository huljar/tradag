#ifndef RGBDOBJECT_H
#define RGBDOBJECT_H

#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace TraDaG {
    class RgbdObject;
}

class TraDaG::RgbdObject
{
public:
    RgbdObject(const Ogre::String& name, Ogre::SceneManager* sceneManager,
               const cv::Mat& depthImage, const cv::Mat& rgbImage,
               const Ogre::Vector2& depthPrincipalPoint, const Ogre::Vector2& depthFocalLength,
               const Ogre::Vector2& rgbPrincipalPoint, const Ogre::Vector2& rgbFocalLength,
               const Ogre::Matrix3& rotation, const Ogre::Vector3& translation,
               MapMode mapMode, bool autoCreateMesh = true);
    virtual ~RgbdObject();

    virtual void meshify();

    virtual Ogre::Vector3 depthToWorld(int u, int v, unsigned short depth) const;
    virtual Ogre::Vector3 depthToWorld(Ogre::Real u, Ogre::Real v, Ogre::Real depth) const;
    virtual Ogre::Vector3 depthToWorld(const Ogre::Vector3& uvdPoint) const;

    virtual Ogre::Vector3 worldToDepth(Ogre::Real x, Ogre::Real y, Ogre::Real z) const;
    virtual Ogre::Vector3 worldToDepth(const Ogre::Vector3& point) const;

    virtual Ogre::Vector2 worldToRgb(const Ogre::Vector3& point, const Ogre::Matrix3& rotation, const Ogre::Vector3& translation) const;

    virtual Ogre::ManualObject* getManualObject();

    virtual cv::Mat getDepthImage();
    virtual const cv::Mat getDepthImage() const;

    virtual cv::Mat getRgbImage();
    virtual const cv::Mat getRgbImage() const;

    virtual Ogre::Vector2 getDepthPrincipalPoint() const;
    virtual void setDepthPrincipalPoint(const Ogre::Vector2& principalPoint);
    virtual void setDepthPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY);

    virtual Ogre::Vector2 getDepthFocalLength() const;
    virtual void setDepthFocalLength(const Ogre::Vector2& focalLength);
    virtual void setDepthFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY);

    virtual Ogre::Vector2 getRgbPrincipalPoint() const;
    virtual void setRgbPrincipalPoint(const Ogre::Vector2& principalPoint);
    virtual void setRgbPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY);

    virtual Ogre::Vector2 getRgbFocalLength() const;
    virtual void setRgbFocalLength(const Ogre::Vector2& focalLength);
    virtual void setRgbFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY);

    virtual Ogre::Matrix3 getRotation() const;
    virtual void setRotation(const Ogre::Matrix3& rotation);

    virtual Ogre::Vector3 getTranslation() const;
    virtual void setTranslation(const Ogre::Vector3& translation);
    virtual void setTranslation(Ogre::Real translationX, Ogre::Real translationY, Ogre::Real translationZ);

    virtual MapMode getMapMode() const;
    virtual void setMapMode(MapMode mapMode);

protected:
    Ogre::ManualObject* mSceneObject;
    bool mMeshUpdated;

    Ogre::SceneManager* mSceneMgr;

    cv::Mat mDepthImage;
    cv::Mat mRgbImage;

    Ogre::Vector2 mDepthPrincipalPoint;
    Ogre::Vector2 mDepthFocalLength;

    Ogre::Vector2 mRgbPrincipalPoint;
    Ogre::Vector2 mRgbFocalLength;

    Ogre::Matrix3 mRotation;
    Ogre::Vector3 mTranslation;

    MapMode mMapMode;

private:
    void createVertices();
    void createIndices();

    inline Ogre::uint32 pixelToIndex(int x, int y) const { return y * mRgbImage.cols + x; }
};

#endif // RGBDOBJECT_H
