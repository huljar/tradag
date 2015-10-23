#ifndef RGBDOBJECT_H
#define RGBDOBJECT_H

#include <Ogre.h>

class RgbdObject : public Ogre::ManualObject
{
public:
    RgbdObject(const Ogre::String& name);

    virtual bool loadRgbFromFile(const Ogre::String& rgbFileName, bool loadAsTexture = true);
    virtual bool loadDepthFromFile(const Ogre::String& depthFileName);
    virtual bool loadLabelsFromFile(const Ogre::String& labelFileName);

    virtual void setCameraParams(const Ogre::Vector2& principalPoint, const Ogre::Vector2& focalLength);

    virtual void meshify();

protected:
    Ogre::Image mRgbImage;
    Ogre::Image mDepthImage;
    Ogre::Image mLabelImage;

    Ogre::String mSceneMaterial;
    Ogre::String mSceneTexture;

    Ogre::Vector2 mPrincipalPoint;
    Ogre::Vector2 mFocalLength;

private:
    bool loadFromFile(const Ogre::String& fileName, Ogre::Image& target);

    void createVertices();
    void createIndices();

    Ogre::Vector3 depthTo3D(Ogre::int32 x, Ogre::int32 y, Ogre::Real depth);
    Ogre::int32 pixelToIndex(Ogre::int32 x, Ogre::int32 y);

    Ogre::String getNextMaterialName(bool save);
    Ogre::String getNextTextureName(bool save);

    static const Ogre::String materialName = "tradagSceneMaterial";
    static int materialId = 0;
    static const Ogre::String textureName = "tradagSceneTexture";
    static int textureId = 0;
};

#endif // RGBDOBJECT_H
