#include <TraDaG/rgbdobject.h>

#include <fstream>

RgbdObject::RgbdObject(const Ogre::String& name)
    : ManualObject(name)
    , mPrincipalPoint(320, 240)
    , mFocalLength(500, 500)
{

}

bool RgbdObject::loadRgbFromFile(const Ogre::String& rgbFileName, bool loadAsTexture) {
    return loadFromFile(rgbFileName, mRgbImage);
    if(loadAsTexture)
        Ogre::TextureManager::getSingleton().loadImage(
                    getNextTextureName(true),
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    mRgbImage,
                    Ogre::TEX_TYPE_2D,
                    0);
}

bool RgbdObject::loadDepthFromFile(const Ogre::String& depthFileName) {
    return loadFromFile(depthFileName, mDepthImage);
}

bool RgbdObject::loadLabelsFromFile(const Ogre::String& labelFileName) {
    return loadFromFile(labelFileName, mLabelImage);
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

void RgbdObject::setCameraParams(const Ogre::Vector2& principalPoint, const Ogre::Vector2& focalLength) {
    mPrincipalPoint = principalPoint;
    mFocalLength = focalLength;
}

void RgbdObject::meshify() {
    // Check if all images have the same size
    if(!(mRgbImage.getWidth() == mDepthImage.getWidth() && mRgbImage.getHeight() == mDepthImage.getHeight()
            && mRgbImage.getWidth() == mLabelImage.getWidth() && mRgbImage.getHeight() == mLabelImage.getHeight())) {
        std::cerr << "ERROR: RGB, depth and label image do not have the same sizes" << std::endl;
        return;
    }

    // Create material with RGB image as texture
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(getNextMaterialName(true), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->createTextureUnitState(mSceneTexture);

    begin(mSceneMaterial, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    createVertices();
    createIndices();
    end();
}

void RgbdObject::createVertices() {
    //std::cout << "Depth Image bits per pixel: " << (unsigned int)mDepthImage.getBPP() << std::endl;
    //std::cout << "RGB Image bits per pixel: " << (unsigned int)mRgbImage.getBPP() << std::endl;
    //std::cout << "Label Image bits per pixel: " << (unsigned int)mLabelImage.getBPP() << std::endl;
    for(int y = 0; y < mRgbImage.getHeight(); ++y) {
        for(int x = 0; x < mRgbImage.getWidth(); ++x) {
            //Ogre::ColourValue c = mDepthImage.getColourAt(i, j, 0);
            //std::cout << "Depth at (" << j << ", " << i << "): (" << c.r << ", " << c.g << ", " << c.b << ")" << std::endl;
            position(depthTo3D(x, y, mDepthImage.getColourAt(x, y, 0).r)); // Since it's a single channel image, r, g and b have the same values
            textureCoord(x, y);
        }
    }
}

void RgbdObject::createIndices() {
    for(int y = 0; y < mRgbImage.getHeight() - 1; ++y) {
        for(int x = 0; x < mRgbImage.getWidth(); ++x) {
            index() // TODO: continue here
        }
    }
}

Ogre::Vector3 RgbdObject::depthTo3D(Ogre::int32 x, Ogre::int32 y, Ogre::Real depth) {
    Ogre::int32 retX = (double)x - mPrincipalPoint.x / ((double)mFocalLength.x / (double)depth);
    Ogre::int32 retY = -((double)y - (mPrincipalPoint.y) / ((double)mFocalLength.y / (double)depth));
    Ogre::int32 retZ = -depth;

    return Ogre::Vector3(retX, retY, retZ);
}

Ogre::int32 RgbdObject::pixelToIndex(Ogre::int32 x, Ogre::int32 y) {
    return y * mRgbImage.getWidth() + x;
}

Ogre::String RgbdObject::getNextMaterialName(bool save) {
    if(save) mSceneMaterial = materialName + materialId;
    return materialName + materialId++;
}

Ogre::String RgbdObject::getNextTextureName(bool save) {
    if(save) mSceneTexture = textureName + textureId;
    return textureName + textureId++;
}
