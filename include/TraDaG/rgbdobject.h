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

    Ogre::Vector2 getPrincipalPoint() const;
    void setPrincipalPoint(const Ogre::Vector2& principalPoint);

    Ogre::Vector2 getFocalLength() const;
    void setFocalLength(const Ogre::Vector2& focalLength);

protected:
//    Ogre::Image mRgbImage;
//    Ogre::Image mDepthImage;
//    Ogre::Image mLabelImage;
    cv::Mat mRgbImage;
    cv::Mat mDepthImage;
    cv::Mat mLabelImage;

    Ogre::String mSceneMaterial;
    Ogre::String mSceneTexture;

    Ogre::Vector2 mPrincipalPoint;
    Ogre::Vector2 mFocalLength;

private:
    void createVertices();
    void createIndices();

    Ogre::Vector3 depthTo3D(Ogre::int32 x, Ogre::int32 y, Ogre::uint16 depth, Ogre::Real scale = 1.0);
    Ogre::uint32 pixelToIndex(Ogre::int32 x, Ogre::int32 y);

    Ogre::String getNextMaterialName(bool save);
    Ogre::String getNextTextureName(bool save);

    static const Ogre::String materialName;
    static int materialId;
    static const Ogre::String textureName;
    static int textureId;
};

#endif // RGBDOBJECT_H
