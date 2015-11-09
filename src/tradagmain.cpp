#include <TraDaG/tradagmain.h>

#include <stdexcept>

using namespace TraDaG;

TradagMain::TradagMain(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage,
                       const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                       const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                       const cv::Matx33f& rotation, const cv::Vec3f translation,
                       MapMode mapMode, LabelMode labelMode) {

    // Perform default initialization
    init(depthImage, rgbImage, labelImage, depthPrincipalPoint, depthFocalLength, rgbPrincipalPoint, rgbFocalLength,
         rotation, translation, mapMode, labelMode);
}

TradagMain::TradagMain(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath,
                       const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                       const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                       const cv::Matx33f& rotation, const cv::Vec3f translation,
                       MapMode mapMode, LabelMode labelMode) {

    // Load images from specified paths
    cv::Mat rgbImage = cv::imread(rgbImagePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat depthImage = cv::imread(depthImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat labelImage = cv::imread(labelImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    // Throw an exception if any of the images was not loaded correctly
    if(!depthImage.data || !rgbImage.data || !labelImage.data)
        throw std::runtime_error("Unable to load at least one of the following images: \""
                                 + depthImagePath + "\", \"" + rgbImagePath + "\", \"" + labelImagePath + "\"");

    // Continue with default initialization
    init(depthImage, rgbImage, labelImage, depthPrincipalPoint, depthFocalLength, rgbPrincipalPoint, rgbFocalLength,
         rotation, translation, mapMode, labelMode);
}

void TradagMain::init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage,
                      const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                      const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                      const cv::Matx33f& rotation, const cv::Vec3f& translation,
                      MapMode mapMode, LabelMode labelMode) {

    mOgreWindow = new OgreWindow();

    Ogre::Real rotationConverted[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            rotationConverted[i][j] = rotation(i, j);
        }
    }
    mRgbdObject = new RgbdObject(Ogre::String(Strings::RgbdObjName), mOgreWindow->getSceneManager(), depthImage, rgbImage,
                                 Ogre::Vector2(depthPrincipalPoint[0], depthPrincipalPoint[1]), Ogre::Vector2(depthFocalLength[0], depthFocalLength[1]),
                                 Ogre::Vector2(rgbPrincipalPoint[0], rgbPrincipalPoint[1]), Ogre::Vector2(rgbFocalLength[0], rgbFocalLength[1]),
                                 Ogre::Matrix3(rotationConverted), Ogre::Vector3(translation[0], translation[1], translation[2]), mapMode);

    mImageLabeling = new ImageLabeling(labelImage, labelMode);

    mOgreWindow->setScene(mRgbdObject);
}

TradagMain::~TradagMain() {
    delete mImageLabeling;
    delete mRgbdObject;
    delete mOgreWindow;
}

void TradagMain::updateMesh() {
    mRgbdObject->meshify();
}

bool TradagMain::dropObjectIntoScene(bool showPreviewWindow, bool showPhysicsAnimation) {
    if(mOgreWindow->hidden())
        mOgreWindow->show();

    mOgreWindow->startAnimation();

    return true;
}

cv::Mat TradagMain::getDepthImage() {
    return mRgbdObject->getDepthImage();
}

const cv::Mat TradagMain::getDepthImage() const {
    return mRgbdObject->getDepthImage();
}

cv::Mat TradagMain::getRgbImage() {
    return mRgbdObject->getRgbImage();
}

const cv::Mat TradagMain::getRgbImage() const {
    return mRgbdObject->getRgbImage();
}

cv::Mat TradagMain::getLabelImage() {
    return cv::Mat(); // TODO: correct later
}

const cv::Mat TradagMain::getLabelImage() const {
    return cv::Mat(); // TODO: correct later
}

void TradagMain::setNewScene(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage) {
    // Get old camera parameters
    Ogre::Vector2 tmpDPP = mRgbdObject->getDepthPrincipalPoint();
    Ogre::Vector2 tmpDFL = mRgbdObject->getDepthFocalLength();
    Ogre::Vector2 tmpRPP = mRgbdObject->getRgbPrincipalPoint();
    Ogre::Vector2 tmpRFL = mRgbdObject->getRgbFocalLength();
    Ogre::Matrix3 tmpRot = mRgbdObject->getRotation();
    Ogre::Vector3 tmpTrans = mRgbdObject->getTranslation();
    MapMode tmpMode = mRgbdObject->getMapMode();

    // Delete old scene and create a new one
    delete mRgbdObject;
    mRgbdObject = new RgbdObject(Strings::RgbdObjName, mOgreWindow->getSceneManager(), depthImage, rgbImage, tmpDPP, tmpDFL, tmpRPP, tmpRFL, tmpRot, tmpTrans, tmpMode);

    // Notify OgreWindow of the new scene
    mOgreWindow->setScene(mRgbdObject);
}

bool TradagMain::loadNewScene(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath) {
    cv::Mat depthImage = cv::imread(depthImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat rgbImage = cv::imread(rgbImagePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat labelImage = cv::imread(labelImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    if(depthImage.data && rgbImage.data && labelImage.data) {
        setNewScene(depthImage, rgbImage, labelImage);
        return true;
    }

    return false;
}

cv::Vec2f TradagMain::getDepthPrincipalPoint() const {
    Ogre::Vector2 tmpDPP = mRgbdObject->getDepthPrincipalPoint();
    return cv::Vec2f(tmpDPP.x, tmpDPP.y);
}

void TradagMain::setDepthPrincipalPoint(const cv::Vec2f& principalPoint) {
    mRgbdObject->setDepthPrincipalPoint(principalPoint[0], principalPoint[1]);
}

cv::Vec2f TradagMain::getDepthFocalLength() const {
    Ogre::Vector2 tmpDFL = mRgbdObject->getDepthFocalLength();
    return cv::Vec2f(tmpDFL.x, tmpDFL.y);
}

void TradagMain::setDepthFocalLength(const cv::Vec2f& focalLength) {
    mRgbdObject->setDepthFocalLength(focalLength[0], focalLength[1]);
}

cv::Vec2f TradagMain::getRgbPrincipalPoint() const {
    Ogre::Vector2 tmpRPP = mRgbdObject->getRgbPrincipalPoint();
    return cv::Vec2f(tmpRPP.x, tmpRPP.y);
}

void TradagMain::setRgbPrincipalPoint(const cv::Vec2f& principalPoint) {
    mRgbdObject->setRgbPrincipalPoint(principalPoint[0], principalPoint[1]);
}

cv::Vec2f TradagMain::getRgbFocalLength() const {
    Ogre::Vector2 tmpRFL = mRgbdObject->getRgbFocalLength();
    return cv::Vec2f(tmpRFL.x, tmpRFL.y);
}

void TradagMain::setRgbFocalLength(const cv::Vec2f& focalLength) {
    mRgbdObject->setRgbFocalLength(focalLength[0], focalLength[1]);
}

MapMode TradagMain::getMapMode() const {
    return mRgbdObject->getMapMode();
}

void TradagMain::setMapMode(MapMode mode) {
    mRgbdObject->setMapMode(mode);
}
