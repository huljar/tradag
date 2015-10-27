#ifndef RGBDOBJECT_H
#define RGBDOBJECT_H

#include <Ogre.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class RgbdObject : public Ogre::ManualObject
{
public:
    RgbdObject(const Ogre::String& name);

    virtual bool loadRgbFromFile(const Ogre::String& rgbFileName);
    virtual bool loadDepthFromFile(const Ogre::String& depthFileName);
    virtual bool loadLabelsFromFile(const Ogre::String& labelFileName);

    virtual void meshify();

    Ogre::Vector2 getDepthPrincipalPoint() const;
    void setDepthPrincipalPoint(const Ogre::Vector2& principalPoint);
    void setDepthPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY);

    Ogre::Vector2 getDepthFocalLength() const;
    void setDepthFocalLength(const Ogre::Vector2& focalLength);
    void setDepthFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY);

    Ogre::Vector2 getRgbPrincipalPoint() const;
    void setRgbPrincipalPoint(const Ogre::Vector2& principalPoint);
    void setRgbPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY);

    Ogre::Vector2 getRgbFocalLength() const;
    void setRgbFocalLength(const Ogre::Vector2& focalLength);
    void setRgbFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY);

    Ogre::Matrix3 getDepthToRgbRotation() const;
    void setDepthToRgbRotation(const Ogre::Matrix3& rotation);

    Ogre::Vector3 getDepthToRgbTranslation() const;
    void setDepthToRgbTranslation(const Ogre::Vector3& translation);
    void setDepthToRgbTranslation(Ogre::Real translationX, Ogre::Real translationY, Ogre::Real translationZ);

protected:
    cv::Mat mRgbImage;
    cv::Mat mDepthImage;
    cv::Mat mLabelImage;

    Ogre::String mSceneMaterial;
    Ogre::String mSceneTexture;

    Ogre::Vector2 mDepthPrincipalPoint;
    Ogre::Vector2 mDepthFocalLength;

    Ogre::Vector2 mRgbPrincipalPoint;
    Ogre::Vector2 mRgbFocalLength;

    Ogre::Matrix3 mDepthToRgbRotation;
    Ogre::Vector3 mDepthToRgbTranslation;

private:
    void createVertices();
    void createIndices();

    Ogre::Vector3 depthToWorld(Ogre::int32 x, Ogre::int32 y, Ogre::uint16 depth) const;
    Ogre::Vector2 worldToRgb(const Ogre::Vector3& point) const;
    inline Ogre::uint32 pixelToIndex(Ogre::int32 x, Ogre::int32 y) const { return y * mRgbImage.cols + x; }

    Ogre::String getNextMaterialName(bool save);
    Ogre::String getNextTextureName(bool save);

    const Ogre::String mMaterialName;
    int mMaterialId;
    const Ogre::String mTextureName;
    int mTextureId;
};

#endif // RGBDOBJECT_H
