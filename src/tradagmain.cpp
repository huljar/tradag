#include <TraDaG/tradagmain.h>

#include <stdexcept>

using namespace TraDaG;

TradagMain::TradagMain(const cv::Mat& depthImage, const cv::Mat& rgbImage,
                       const cv::Mat& labelImage, const LabelMap& labelMap,
                       const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                       const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                       const cv::Matx33f& rotation, const cv::Vec3f translation,
                       MapMode mapMode, LabelMode labelMode)
    : TradagMain()
{
    // Perform default initialization
    init(depthImage, rgbImage, labelImage, labelMap, depthPrincipalPoint, depthFocalLength, rgbPrincipalPoint, rgbFocalLength,
         rotation, translation, mapMode, labelMode);
}

TradagMain::TradagMain(const std::string& depthImagePath, const std::string& rgbImagePath,
                       const std::string& labelImagePath, const LabelMap& labelMap,
                       const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                       const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                       const cv::Matx33f& rotation, const cv::Vec3f translation,
                       MapMode mapMode, LabelMode labelMode)
    : TradagMain()
{
    // Load images from specified paths
    cv::Mat rgbImage = cv::imread(rgbImagePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat depthImage = cv::imread(depthImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat labelImage = cv::imread(labelImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    // Throw an exception if any of the images was not loaded correctly
    if(!depthImage.data || !rgbImage.data || !labelImage.data)
        throw std::runtime_error("Unable to load at least one of the following images: \""
                                 + depthImagePath + "\", \"" + rgbImagePath + "\", \"" + labelImagePath + "\"");

    // Continue with default initialization
    init(depthImage, rgbImage, labelImage, labelMap, depthPrincipalPoint, depthFocalLength, rgbPrincipalPoint, rgbFocalLength,
         rotation, translation, mapMode, labelMode);
}

TradagMain::TradagMain()
    : mOgreWindow(NULL)
    , mRgbdObject(NULL)
    , mImageLabeling(NULL)
    , mObjectScale(Defaults::ObjectScale)
    , mObjectMustBeUpright(Defaults::ObjectMustBeUpright)
    , mObjectCoveredFraction(Defaults::ObjectCoveredFraction)
    , mObjectCastShadows(Defaults::ObjectCastShadows)
    , mMaxAttempts(Defaults::MaxAttempts)
    , mShowPreviewWindow(Defaults::ShowPreviewWindow)
    , mShowPhysicsAnimation(Defaults::ShowPhysicsAnimation)
    , mGravity(Defaults::Gravity)
    , mObjectRestitution(Defaults::ObjectRestitution)
    , mObjectFriction(Defaults::ObjectFriction)
    , mPlaneRestitution(Defaults::PlaneRestitution)
    , mPlaneFriction(Defaults::PlaneFriction)
{
}

TradagMain::~TradagMain() {
    delete mImageLabeling;
    delete mRgbdObject;
    delete mOgreWindow;
}

void TradagMain::init(const cv::Mat& depthImage, const cv::Mat& rgbImage,
                      const cv::Mat& labelImage, const LabelMap& labelMap,
                      const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                      const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                      const cv::Matx33f& rotation, const cv::Vec3f& translation,
                      MapMode mapMode, LabelMode labelMode) {

    mOgreWindow = new OgreWindow();
    // TODO: light position?

    Ogre::Real rotationConverted[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            rotationConverted[i][j] = rotation(i, j);
        }
    }
    mRgbdObject = new RgbdObject(Ogre::String(Strings::RgbdSceneName), mOgreWindow->getSceneManager(), depthImage, rgbImage,
                                 Ogre::Vector2(depthPrincipalPoint[0], depthPrincipalPoint[1]),
                                 Ogre::Vector2(depthFocalLength[0], depthFocalLength[1]),
                                 Ogre::Vector2(rgbPrincipalPoint[0], rgbPrincipalPoint[1]),
                                 Ogre::Vector2(rgbFocalLength[0], rgbFocalLength[1]),
                                 Ogre::Matrix3(rotationConverted), Ogre::Vector3(translation[0], translation[1], translation[2]),
                                 mapMode);

    mImageLabeling = new ImageLabeling(labelImage, labelMap, labelMode);

    mOgreWindow->setScene(mRgbdObject);
}

void TradagMain::updateMesh() {
    mRgbdObject->meshify();
}

ObjectDropResult TradagMain::dropObjectIntoScene(const std::string& meshName, const std::string& planeLabel,
                                                 const Auto<cv::Vec3f>& initialPosition, const Auto<cv::Matx33f>& initialRotation,
                                                 const cv::Vec3f& initialVelocity, const cv::Vec3f& initialTorque) {

    // Calculate plane from labels using RANSAC
    // TODO: handle non-success
    PlaneFittingResult planeFit = mImageLabeling->getPlaneForLabel(planeLabel, mRgbdObject);
    Ogre::Plane groundPlane(planeFit.plane.normal, planeFit.plane.d); // If I don't do this, Bullet freaks out

    // TEST: draw the plane
    if(planeFit.result == SUCCESS_FIT) {
        std::cout << "Plane data: (" << groundPlane.normal.x << ", " << groundPlane.normal.y << ", " << groundPlane.normal.z
                  << "), " << groundPlane.d << std::endl;
        Ogre::MeshManager::getSingleton().createPlane("testPlane2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, groundPlane, 5000, 5000,
                                                      1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
        Ogre::Entity* planeEntity = mOgreWindow->getSceneManager()->createEntity("testPlane2");
        planeEntity->setMaterialName("BaseWhiteNoLighting");
        mOgreWindow->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(planeEntity);
    }
    else {
        std::cerr << "ERROR: plane fitting error " << planeFit.result << std::endl;
    }

    // Calculate initial position
    Ogre::Vector3 actualPosition(-1300, 400, -3600); // TODO

    // Calculate initial rotation
    Ogre::Matrix3 actualRotation(Ogre::Matrix3::IDENTITY); // TODO

    // Calculate/convert additional parameters
    Ogre::Vector3 linearVelocity(initialVelocity[0], initialVelocity[1], initialVelocity[2]);

    // If the object shall be upright, ignore any torque passed to the function (anything else wouldn't make sense... right?)
    Ogre::Vector3 angularVelocity = mObjectMustBeUpright ? Ogre::Vector3::ZERO : Ogre::Vector3(initialTorque[0], initialTorque[1], initialTorque[2]);

    // Pass all-zero vector as angular factor to prevent random infinite object spinning
    // (yes, this can happen if you allow yaw rotation, i.e. if you pass (0, 1, 0))
    // To emulate yaw rotation, one can pass a random initial rotation around the y-axis to this function
    Ogre::Vector3 angularFactor = mObjectMustBeUpright ? Ogre::Vector3::ZERO : Ogre::Vector3::UNIT_SCALE;
    Ogre::Vector3 gravity(mGravity[0], mGravity[1], mGravity[2]);

    // Display the window if requested
    if(mShowPreviewWindow && mOgreWindow->hidden())
        mOgreWindow->show();

    // Simulate with animation only if needed
    // TODO: objectCoveredFraction, maxAttempts
    if(mShowPreviewWindow && mShowPhysicsAnimation) {
        mOgreWindow->startAnimation(meshName, actualPosition, actualRotation, mObjectScale, linearVelocity, angularVelocity, angularFactor, mObjectRestitution,
                                    mObjectFriction, 1.0, groundPlane, mPlaneRestitution, mPlaneFriction, gravity, mObjectCastShadows);
    }
    else {
        // TODO: simulate without animating
    }

    // TODO: render and return the final image
    return ObjectDropResult(SUCCESS_DROP, cv::Mat(), 0.0, cv::Matx33f::eye(), cv::Vec3f(0, 0, 0));
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

LabelMap TradagMain::getLabelMap() const {
    return mImageLabeling->getLabelMap();
}

void TradagMain::setNewScene(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage, const LabelMap& labelMap) {
    // Get old camera parameters
    Ogre::Vector2 tmpDPP = mRgbdObject->getDepthPrincipalPoint();
    Ogre::Vector2 tmpDFL = mRgbdObject->getDepthFocalLength();
    Ogre::Vector2 tmpRPP = mRgbdObject->getRgbPrincipalPoint();
    Ogre::Vector2 tmpRFL = mRgbdObject->getRgbFocalLength();
    Ogre::Matrix3 tmpRot = mRgbdObject->getRotation();
    Ogre::Vector3 tmpTrans = mRgbdObject->getTranslation();
    MapMode tmpMM = mRgbdObject->getMapMode();
    LabelMode tmpLM = mImageLabeling->getLabelMode();

    // Delete old scene and create a new one
    delete mRgbdObject;
    mRgbdObject = new RgbdObject(Strings::RgbdSceneName, mOgreWindow->getSceneManager(), depthImage, rgbImage, tmpDPP, tmpDFL, tmpRPP, tmpRFL, tmpRot, tmpTrans, tmpMM);

    // Delete old labeling and create a new one
    delete mImageLabeling;
    mImageLabeling = new ImageLabeling(labelImage, labelMap, tmpLM);

    // Notify OgreWindow of the new scene
    mOgreWindow->setScene(mRgbdObject);
}

bool TradagMain::loadNewScene(const std::string& depthImagePath, const std::string& rgbImagePath, const std::string& labelImagePath, const LabelMap& labelMap) {
    cv::Mat depthImage = cv::imread(depthImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat rgbImage = cv::imread(rgbImagePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat labelImage = cv::imread(labelImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    if(depthImage.data && rgbImage.data && labelImage.data) {
        setNewScene(depthImage, rgbImage, labelImage, labelMap);
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

cv::Matx33f TradagMain::getRotation() const {
    Ogre::Matrix3 tmpRot = mRgbdObject->getRotation();
    return cv::Matx33f(tmpRot[0][0], tmpRot[0][1], tmpRot[0][2],
                       tmpRot[1][0], tmpRot[1][1], tmpRot[1][2],
                       tmpRot[2][0], tmpRot[2][1], tmpRot[2][2]);
}

void TradagMain::setRotation(const cv::Matx33f& rotation) {
    Ogre::Real rotationConverted[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            rotationConverted[i][j] = rotation(i, j);
        }
    }
    mRgbdObject->setRotation(Ogre::Matrix3(rotationConverted));
}

cv::Vec3f TradagMain::getTranslation() const {
    Ogre::Vector3 tmpTrans = mRgbdObject->getTranslation();
    return cv::Vec3f(tmpTrans.x, tmpTrans.y, tmpTrans.z);
}

void TradagMain::setTranslation(const cv::Vec3f& translation) {
    mRgbdObject->setTranslation(translation[0], translation[1], translation[2]);
}

MapMode TradagMain::getMapMode() const {
    return mRgbdObject->getMapMode();
}

void TradagMain::setMapMode(MapMode mode) {
    mRgbdObject->setMapMode(mode);
}

LabelMode TradagMain::getLabelMode() const {
    return mImageLabeling->getLabelMode();
}

void TradagMain::setLabelMode(LabelMode mode) {
    mImageLabeling->setLabelMode(mode);
}

float TradagMain::getObjectScale() const {
    return mObjectScale;
}

void TradagMain::setObjectScale(float scale) {
    mObjectScale = scale;
}

bool TradagMain::objectMustBeUpright() const {
    return mObjectMustBeUpright;
}

void TradagMain::setObjectMustBeUpright(bool upright) {
    mObjectMustBeUpright = upright;
}

Auto<float> TradagMain::getObjectCoveredFraction() const {
    return mObjectCoveredFraction;
}

void TradagMain::setObjectCoveredFraction(const Auto<float>& covered) {
    mObjectCoveredFraction = covered;
}

bool TradagMain::objectCastShadows() const {
    return mObjectCastShadows;
}

void TradagMain::setObjectCastShadows(bool castShadows) {
    mObjectCastShadows = castShadows;
}

unsigned int TradagMain::getMaxAttempts() const {
    return mMaxAttempts;
}

void TradagMain::setMaxAttempts(unsigned int maxAttempts) {
    mMaxAttempts = maxAttempts;
}

bool TradagMain::showPreviewWindow() const {
    return mShowPreviewWindow;
}

void TradagMain::setShowPreviewWindow(bool showWindow) {
    mShowPreviewWindow = showWindow;
}

bool TradagMain::showPhysicsAnimation() const {
    return mShowPhysicsAnimation;
}

void TradagMain::setShowPhysicsAnimation(bool showAnimation) {
    mShowPhysicsAnimation = showAnimation;
}

cv::Vec3f TradagMain::getGravity() const {
    return mGravity;
}

void TradagMain::setGravity(const cv::Vec3f& gravity) {
    mGravity = gravity;
}

float TradagMain::getObjectRestitution() const {
    return mObjectRestitution;
}

void TradagMain::setObjectRestitution(float restitution) {
    mObjectRestitution = restitution;
}

float TradagMain::getObjectFriction() const {
    return mObjectFriction;
}

void TradagMain::setObjectFriction(float friction) {
    mObjectFriction = friction;
}

float TradagMain::getPlaneRestitution() const {
    return mPlaneRestitution;
}

void TradagMain::setPlaneRestitution(float restitution) {
    mPlaneRestitution = restitution;
}

float TradagMain::getPlaneFriction() const {
    return mPlaneFriction;
}

void TradagMain::setPlaneFriction(float friction) {
    mPlaneFriction = friction;
}
