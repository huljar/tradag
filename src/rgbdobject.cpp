#include <TraDaG/rgbdobject.h>

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <sstream>

// define static members
const Ogre::String RgbdObject::materialName = "tradagSceneMaterial";
int RgbdObject::materialId = 0;
const Ogre::String RgbdObject::textureName = "tradagSceneTexture";
int RgbdObject::textureId = 0;

RgbdObject::RgbdObject(const Ogre::String& name)
    : ManualObject(name)
    , mPrincipalPoint(320, 240)
    , mFocalLength(500, 500)
{

}

bool RgbdObject::loadRgbFromFile(const Ogre::String& rgbFileName) {
    mRgbImage = cv::imread(rgbFileName, CV_LOAD_IMAGE_COLOR);
    return mRgbImage.data;
}

bool RgbdObject::loadDepthFromFile(const Ogre::String& depthFileName) {
    mDepthImage = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    return mDepthImage.data;
}

bool RgbdObject::loadLabelsFromFile(const Ogre::String& labelFileName) {
    mLabelImage = cv::imread(labelFileName, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    return mLabelImage.data;
}

void RgbdObject::meshify() {
    // Check if all images have the same size
    if(!(mRgbImage.cols == mDepthImage.cols && mRgbImage.rows == mDepthImage.rows
            && mRgbImage.cols == mLabelImage.cols && mRgbImage.rows == mLabelImage.rows)) {
        std::cerr << "ERROR: RGB, depth and label image do not have the same sizes" << std::endl;
        return;
    }

    // Create material with RGB image as texture
//    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(getNextMaterialName(true), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
//    pass->setLightingEnabled(false);
//    Ogre::TextureUnitState* texUnit = pass->createTextureUnitState(mSceneTexture);
//    texUnit->setTextureScale(1.0, 1.0);

    begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    createVertices();
    createIndices();
    end();
}

void RgbdObject::createVertices() {
    for(int y = 0; y < mRgbImage.rows; ++y) {
        for(int x = 0; x < mRgbImage.cols; ++x) {
            //uint16_t c = mDepthImage.at<uint16_t>(y, x);
            //std::cout << "Depth at (" << x << ", " << y << "): " << c << std::endl;
            position(depthTo3D(x, y, mDepthImage.at<uint16_t>(y, x), 0.5));

            cv::Vec3b pxColor = mRgbImage.at<cv::Vec3b>(y, x);
            // Ogre uses RGB and OpenCV uses BGR, hence the reversed indexing
            colour(((Ogre::Real)pxColor[2]) / 255.0f, ((Ogre::Real)pxColor[1]) / 255.0f, ((Ogre::Real)pxColor[0]) / 255.0f);
        }
    }
}

void RgbdObject::createIndices() {
    for(int y = 0; y < mRgbImage.rows - 1; ++y) {
        for(int x = 0; x < mRgbImage.cols - 1; ++x) {
            index(pixelToIndex(x, y));
            index(pixelToIndex(x, y + 1));
            index(pixelToIndex(x + 1, y));

            index(pixelToIndex(x + 1, y));
            index(pixelToIndex(x, y + 1));
            index(pixelToIndex(x + 1, y + 1));
        }
    }
}

Ogre::Vector3 RgbdObject::depthTo3D(Ogre::int32 x, Ogre::int32 y, Ogre::uint16 depth, Ogre::Real scale) {
    Ogre::Real retX = (x - mPrincipalPoint.x) * (Ogre::Real)depth * scale / mFocalLength.x;
    // Ogre's y-vector points up, but OpenCV's y-vector points down (therefore negate result)
    Ogre::Real retY = -((y - mPrincipalPoint.y) * (Ogre::Real)depth * scale / mFocalLength.y);
    Ogre::Real retZ = -((Ogre::Real)depth);

    return Ogre::Vector3(retX, retY, retZ);
}

//Ogre::Vector3 RgbdObject::depthTo3D(Ogre::int32 x, Ogre::int32 y, Ogre::uint16 depth) {
//    Ogre::int32 retX = (double)x - mPrincipalPoint.x / ((double)mFocalLength.x / (double)depth);
//    Ogre::int32 retY = -((double)y - mPrincipalPoint.y / ((double)mFocalLength.y / (double)depth));
//    Ogre::uint16 retZ = -depth;

//    return Ogre::Vector3(retX, retY, retZ);
//}

Ogre::uint32 RgbdObject::pixelToIndex(Ogre::int32 x, Ogre::int32 y) {
    return y * mRgbImage.cols + x;
}

Ogre::String RgbdObject::getNextMaterialName(bool save) {
    if(save) mSceneMaterial = materialName + boost::lexical_cast<Ogre::String>(materialId);
    return materialName + boost::lexical_cast<Ogre::String>(materialId++);
}

Ogre::String RgbdObject::getNextTextureName(bool save) {
    if(save) mSceneTexture = textureName + boost::lexical_cast<Ogre::String>(textureId);
    return textureName + boost::lexical_cast<Ogre::String>(textureId++);
}

Ogre::Vector2 RgbdObject::getPrincipalPoint() const {
    return mPrincipalPoint;
}

void RgbdObject::setPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mPrincipalPoint = principalPoint;
}

Ogre::Vector2 RgbdObject::getFocalLength() const {
    return mFocalLength;
}

void RgbdObject::setFocalLength(const Ogre::Vector2& focalLength) {
    mFocalLength = focalLength;
}
