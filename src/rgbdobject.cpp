#include <TraDaG/rgbdobject.h>

#include <fstream>

RgbdObject::RgbdObject()
{

}

bool RgbdObject::loadRgbFromFile(const Ogre::String& rgbFileName) {
    return loadFromFile(rgbFileName, rgbImage);
}

bool RgbdObject::loadDepthFromFile(const Ogre::String& depthFileName) {
    return loadFromFile(depthFileName, depthImage);
}

bool RgbdObject::loadFromFile(const Ogre::String& fileName, Ogre::Image& target) {
    bool imageLoaded = false;
    std::ifstream input(fileName.c_str(), std::ios::binary | std::ios::in);
    if(input.is_open()) {
        Ogre::String extension;
        Ogre::String::size_type extensionIndex = fileName.find_last_of('.');
        if(extensionIndex != Ogre::String::npos) {
            extension = fileName.substr(extensionIndex + 1);
            Ogre::DataStreamPtr data(new Ogre::FileStreamDataStream(fileName, &input, false));
            target.load(data, extension);
            imageLoaded = true;
        }
    }
    return imageLoaded;
}
