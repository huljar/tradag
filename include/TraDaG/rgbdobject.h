#ifndef RGBDOBJECT_H
#define RGBDOBJECT_H

#include <Ogre.h>

class RgbdObject : public Ogre::ManualObject
{
public:
    RgbdObject();

    bool loadRgbFromFile(const Ogre::String& rgbFileName);
    bool loadDepthFromFile(const Ogre::String& depthFileName);

protected:
    Ogre::Image rgbImage;
    Ogre::Image depthImage;

private:
    bool loadFromFile(const Ogre::String& fileName, Ogre::Image& target);

};

#endif // RGBDOBJECT_H
