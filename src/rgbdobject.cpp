#include <TraDaG/rgbdobject.h>

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <sstream>

RgbdObject::RgbdObject(const Ogre::String& name)
    : ManualObject(name)
    , mDepthPrincipalPoint(320, 240)
    , mDepthFocalLength(500, 500)
    , mRgbPrincipalPoint(320, 240)
    , mRgbFocalLength(500, 500)
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

    begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    createVertices();
    createIndices();
    end();
}

void RgbdObject::createVertices() {
    for(int y = 0; y < mDepthImage.rows; ++y) {
        for(int x = 0; x < mDepthImage.cols; ++x) {
            // Transform depth pixel to world coordinates
            Ogre::Vector3 worldPoint = depthToWorld(x, y, mDepthImage.at<uint16_t>(y, x));
            position(worldPoint);

            // Retrieve RGB pixel for this world point
            //Ogre::Vector2 rgbPixel = worldToRgb(worldPoint);
            //cv::Vec3b rgbColor = mRgbImage.at<cv::Vec3b>(rgbPixel.y, rgbPixel.x);
            cv::Vec3b rgbColor = mRgbImage.at<cv::Vec3b>(y, x);

            // Ogre uses RGB and OpenCV uses BGR, hence the reversed indexing
            colour(((Ogre::Real)rgbColor[2]) / 255.0f, ((Ogre::Real)rgbColor[1]) / 255.0f, ((Ogre::Real)rgbColor[0]) / 255.0f);
        }
    }
}

void RgbdObject::createIndices() {
    for(int y = 0; y < mDepthImage.rows - 1; ++y) {
        for(int x = 0; x < mDepthImage.cols - 1; ++x) {
            // Create 2 triangles (= 1 "square") per iteration
            index(pixelToIndex(x, y));
            index(pixelToIndex(x, y + 1));
            index(pixelToIndex(x + 1, y));

            index(pixelToIndex(x + 1, y));
            index(pixelToIndex(x, y + 1));
            index(pixelToIndex(x + 1, y + 1));
        }
    }
}

Ogre::Vector3 RgbdObject::depthToWorld(Ogre::int32 x, Ogre::int32 y, Ogre::uint16 depth) const {
    Ogre::Real retX = ((Ogre::Real)x - mDepthPrincipalPoint.x) * (Ogre::Real)depth / mDepthFocalLength.x;
    // Ogre's y-vector points up, but OpenCV's y-vector points down (therefore negate result)
    Ogre::Real retY = -(((Ogre::Real)y - mDepthPrincipalPoint.y) * (Ogre::Real)depth / mDepthFocalLength.y);
    Ogre::Real retZ = -((Ogre::Real)depth);

    return Ogre::Vector3(retX, retY, retZ);
}

Ogre::Vector2 RgbdObject::worldToRgb(const Ogre::Vector3& point) const {
    Ogre::Vector3 transformed = mDepthToRgbRotation * point + mDepthToRgbTranslation;
    //Ogre::Vector3 transformed = point;

    Ogre::Real retX = std::round(transformed.x * mRgbFocalLength.x / (-transformed.z) + mRgbPrincipalPoint.x);
    Ogre::Real retY = std::round((-transformed.y) * mRgbFocalLength.y / (-transformed.z) + mRgbPrincipalPoint.y);
    //Ogre::Real retX = std::round(transformed.x * mDepthFocalLength.x / (-transformed.z) + mDepthPrincipalPoint.x);
    //Ogre::Real retY = std::round((-transformed.y) * mDepthFocalLength.y / (-transformed.z) + mDepthPrincipalPoint.y);

    return Ogre::Vector2(
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.cols, retX)),
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.rows, retY)));
}

Ogre::Vector2 RgbdObject::getDepthPrincipalPoint() const {
    return mDepthPrincipalPoint;
}

void RgbdObject::setDepthPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mDepthPrincipalPoint = principalPoint;
}

void RgbdObject::setDepthPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mDepthPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
}

Ogre::Vector2 RgbdObject::getDepthFocalLength() const {
    return mDepthFocalLength;
}

void RgbdObject::setDepthFocalLength(const Ogre::Vector2& focalLength) {
    mDepthFocalLength = focalLength;
}

void RgbdObject::setDepthFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mDepthFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
}

Ogre::Vector2 RgbdObject::getRgbPrincipalPoint() const {
    return mRgbPrincipalPoint;
}

void RgbdObject::setRgbPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mRgbPrincipalPoint = principalPoint;
}

void RgbdObject::setRgbPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mRgbPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
}

Ogre::Vector2 RgbdObject::getRgbFocalLength() const {
    return mRgbFocalLength;
}

void RgbdObject::setRgbFocalLength(const Ogre::Vector2& focalLength) {
    mRgbFocalLength = focalLength;
}

void RgbdObject::setRgbFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mRgbFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
}

Ogre::Matrix3 RgbdObject::getDepthToRgbRotation() const {
    return mDepthToRgbRotation;
}

void RgbdObject::setDepthToRgbRotation(const Ogre::Matrix3& rotation) {
    mDepthToRgbRotation = rotation;
}

Ogre::Vector3 RgbdObject::getDepthToRgbTranslation() const {
    return mDepthToRgbTranslation;
}

void RgbdObject::setDepthToRgbTranslation(const Ogre::Vector3& translation) {
    mDepthToRgbTranslation = translation;
}

void RgbdObject::setDepthToRgbTranslation(Ogre::Real translationX, Ogre::Real translationY, Ogre::Real translationZ) {
    mDepthToRgbTranslation = Ogre::Vector3(translationX, translationY, translationZ);
}
