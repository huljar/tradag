#include <TraDaG/tradagmain.h>

#include <cmath>
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
    , mMarkInlierSet(Defaults::MarkInlierSet)
    , mDrawBulletShapes(Defaults::DrawBulletShapes)
    , mGravity(Defaults::Gravity)
    , mObjectRestitution(Defaults::ObjectRestitution)
    , mObjectFriction(Defaults::ObjectFriction)
    , mPlaneRestitution(Defaults::PlaneRestitution)
    , mPlaneFriction(Defaults::PlaneFriction)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
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

    mRgbdObject = new RgbdObject(Ogre::String(Strings::RgbdSceneName), mOgreWindow->getSceneManager(), depthImage, rgbImage,
                                 Ogre::Vector2(depthPrincipalPoint[0], depthPrincipalPoint[1]),
                                 Ogre::Vector2(depthFocalLength[0], depthFocalLength[1]),
                                 Ogre::Vector2(rgbPrincipalPoint[0], rgbPrincipalPoint[1]),
                                 Ogre::Vector2(rgbFocalLength[0], rgbFocalLength[1]),
                                 convertCvMatToOgreMat(rotation), Ogre::Vector3(translation[0], translation[1], translation[2]),
                                 mapMode);

    mImageLabeling = new ImageLabeling(labelImage, labelMap, labelMode);

    mOgreWindow->setScene(mRgbdObject, true);
}

void TradagMain::updateMesh() {
    mRgbdObject->meshify();
}

ObjectDropResult TradagMain::dropObjectIntoScene(const std::string& meshName, const std::string& planeLabel,
                                                 const Auto<cv::Vec3f>& initialPosition, const Auto<cv::Matx33f>& initialRotation,
                                                 const float initialAzimuth,
                                                 const cv::Vec3f& initialVelocity, const cv::Vec3f& initialTorque) {

    // Calculate plane from labels using RANSAC
    // TODO: handle non-success, re-use previous planes
    PlaneFitResult planeFit = mImageLabeling->getPlaneForLabel(planeLabel, mRgbdObject);

    if(planeFit.status != PF_SUCCESS)
        return ObjectDropResult(OD_UNKNOWN_ERROR, cv::Mat(), 0.0, cv::Matx33f::eye(), cv::Vec3f(0, 0, 0)); // TODO: use different error

    // If the plane does not face the camera (i.e. the camera lies on the negative side of the plane), invert its normal (to make it face the camera)
    Ogre::Plane groundPlane(planeFit.plane);
    if(planeFit.plane.getDistance(mOgreWindow->getInitialCameraPosition()) < 0) {
        groundPlane.normal = -groundPlane.normal;
        groundPlane.d = -groundPlane.d;
    }

    // Calculate gravity vector
    Ogre::Vector3 gravity = mGravity.automate
                            ? groundPlane.normal.normalisedCopy() * Defaults::Gravity.manualValue[1]
                            : Ogre::Vector3(mGravity.manualValue[0], mGravity.manualValue[1], mGravity.manualValue[2]);

    // Abort if the angle between plane normal and gravity vector is too large
    if(!mGravity.automate && groundPlane.normal.angleBetween(-gravity) > Constants::MaxPlaneNormalToGravityAngle) {
        return ObjectDropResult(OD_PLANE_TOO_STEEP, cv::Mat(), 0.0, cv::Matx33f::eye(), cv::Vec3f(0, 0, 0));
    }

    // Mark the plane inlier set if requested
    if(mMarkInlierSet) {
        mOgreWindow->markVertices(planeFit.vertices);
    }

    // Calculate initial position
    Ogre::Vector3 actualPosition = initialPosition.automate
                                   ? computePosition(planeFit.vertices, gravity)
                                   : Ogre::Vector3(initialPosition.manualValue[0], initialPosition.manualValue[1], initialPosition.manualValue[2]);

    // Calculate initial rotation
    Ogre::Matrix3 actualRotation = initialRotation.automate
                                   ? computeRotation(initialAzimuth, gravity)
                                   : convertCvMatToOgreMat(initialRotation.manualValue);

    // Set linear velocity
    Ogre::Vector3 linearVelocity(initialVelocity[0], initialVelocity[1], initialVelocity[2]);

    // If the object shall be upright, ignore any torque passed to the function (anything else wouldn't make sense... right?)
    Ogre::Vector3 angularVelocity = mObjectMustBeUpright ? Ogre::Vector3::ZERO : Ogre::Vector3(initialTorque[0], initialTorque[1], initialTorque[2]);

    // Pass all-zero vector as angular factor to prevent random infinite object spinning
    // (yes, this can happen if you allow yaw rotation, i.e. if you pass (0, 1, 0))
    // To emulate yaw rotation, one can pass a random initial rotation around the y-axis to this function
    Ogre::Vector3 angularFactor = mObjectMustBeUpright ? Ogre::Vector3::ZERO : Ogre::Vector3::UNIT_SCALE;

    // Display the window to view animation
    if(mShowPreviewWindow && mShowPhysicsAnimation && mOgreWindow->hidden())
        mOgreWindow->show();

    // Simulate (with animation only if needed)
    // This function does not return until the simulation is completed
    // TODO: objectCoveredFraction, maxAttempts
    SimulationResult result = mOgreWindow->startSimulation(
                                  meshName, actualPosition, actualRotation, mObjectScale, linearVelocity, angularVelocity, angularFactor,
                                  mObjectRestitution, mObjectFriction, 1.0, groundPlane, mPlaneRestitution, mPlaneFriction, gravity,
                                  mObjectCastShadows, mDrawBulletShapes, mShowPreviewWindow && mShowPhysicsAnimation
                              );

    // Get covered fraction of the object
    Ogre::Real covered = mOgreWindow->queryCoveredFraction();

    // If a preview without animation was requested, display the window now
    if(mShowPreviewWindow && !mShowPhysicsAnimation && mOgreWindow->hidden())
        mOgreWindow->show();

    // Select an action describing what to do with the result
    UserAction action = UA_KEEP;
    if(!mOgreWindow->hidden()) {
        action = mOgreWindow->promptUserAction();
        // TODO
    }
    else {
        // Auto-select action
        action = UA_KEEP;
    }

    // Remove vertex markings
    mOgreWindow->unmarkVertices();

    // Get and show rendered image
    cv::Mat renderedImage;
    if(mOgreWindow->render(renderedImage)) {
        return ObjectDropResult(OD_SUCCESS, renderedImage, covered, cv::Matx33f::eye(), cv::Vec3f(0, 0, 0)); // TODO: set rot, trans
    }

    return ObjectDropResult(OD_UNKNOWN_ERROR, cv::Mat(), 0.0, cv::Matx33f::eye(), cv::Vec3f(0, 0, 0));
}

Ogre::Vector3 TradagMain::computePosition(const std::vector<Ogre::Vector3>& inliers, const Ogre::Vector3& gravity) {
    std::uniform_int_distribution<size_t> distribution(0, inliers.size() - 1);
    Ogre::Vector3 point = inliers[distribution(mRandomEngine)];

    return point - Constants::ObjectDropDistance * gravity.normalisedCopy();
}

Ogre::Matrix3 TradagMain::computeRotation(const float azimuth, const Ogre::Vector3& gravity) const {
    // Find rotation matrix so the object starts upright
    // (see https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d)
    Ogre::Vector3 a = Ogre::Vector3::UNIT_Y;
    Ogre::Vector3 b = -gravity.normalisedCopy();

    Ogre::Vector3 v = a.crossProduct(b); // axis to rotate around
    Ogre::Real s = v.length(); // sine of angle
    Ogre::Real c = a.dotProduct(b); // cosine of angle

    Ogre::Matrix3 rotation = Ogre::Matrix3::IDENTITY;

    // Only proceed if rotation is not identity (otherwise division by zero can happen if s==0)
    if(v != Ogre::Vector3::ZERO && s != 0) {
        Ogre::Matrix3 v_x(   0, -v.z,  v.y,
                           v.z,    0, -v.x,
                          -v.y,  v.x,    0); // cross product matrix of v

        rotation = Ogre::Matrix3::IDENTITY
                   + v_x
                   + v_x * v_x * ((1 - c) / (s * s));
    }

    // Incorporate specified azimuth
    float a_cos = std::cos(azimuth);
    float a_sin = std::sin(azimuth);
    rotation = rotation * Ogre::Matrix3(a_cos,  0,  a_sin,
                                        0,      1,      0,
                                        -a_sin, 0,  a_cos);

    return rotation;
}

Ogre::Matrix3 TradagMain::convertCvMatToOgreMat(const cv::Matx33f& mat) const {
    Ogre::Real conv[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            conv[i][j] = mat(i, j);
        }
    }
    return Ogre::Matrix3(conv);
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
    mOgreWindow->setScene(mRgbdObject, true);
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

void TradagMain::setDepthPrincipalPoint(float x, float y) {
    mRgbdObject->setDepthPrincipalPoint(x, y);
}

cv::Vec2f TradagMain::getDepthFocalLength() const {
    Ogre::Vector2 tmpDFL = mRgbdObject->getDepthFocalLength();
    return cv::Vec2f(tmpDFL.x, tmpDFL.y);
}

void TradagMain::setDepthFocalLength(const cv::Vec2f& focalLength) {
    mRgbdObject->setDepthFocalLength(focalLength[0], focalLength[1]);
}

void TradagMain::setDepthFocalLength(float x, float y) {
    mRgbdObject->setDepthFocalLength(x, y);
}

cv::Vec2f TradagMain::getRgbPrincipalPoint() const {
    Ogre::Vector2 tmpRPP = mRgbdObject->getRgbPrincipalPoint();
    return cv::Vec2f(tmpRPP.x, tmpRPP.y);
}

void TradagMain::setRgbPrincipalPoint(const cv::Vec2f& principalPoint) {
    mRgbdObject->setRgbPrincipalPoint(principalPoint[0], principalPoint[1]);
}

void TradagMain::setRgbPrincipalPoint(float x, float y) {
    mRgbdObject->setRgbPrincipalPoint(x, y);
}

cv::Vec2f TradagMain::getRgbFocalLength() const {
    Ogre::Vector2 tmpRFL = mRgbdObject->getRgbFocalLength();
    return cv::Vec2f(tmpRFL.x, tmpRFL.y);
}

void TradagMain::setRgbFocalLength(const cv::Vec2f& focalLength) {
    mRgbdObject->setRgbFocalLength(focalLength[0], focalLength[1]);
}

void TradagMain::setRgbFocalLength(float x, float y) {
    mRgbdObject->setRgbFocalLength(x, y);
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

void TradagMain::setTranslation(float x, float y, float z) {
    mRgbdObject->setTranslation(x, y, z);
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

bool TradagMain::debugMarkInlierSet() const {
    return mMarkInlierSet;
}

void TradagMain::setDebugMarkInlierSet(bool markInliers) {
    mMarkInlierSet = markInliers;
}

bool TradagMain::debugDrawBulletShapes() const {
    return mDrawBulletShapes;
}

void TradagMain::setDebugDrawBulletShapes(bool drawShapes) {
    mDrawBulletShapes = drawShapes;
}

Auto<cv::Vec3f> TradagMain::getGravity() const {
    return mGravity;
}

void TradagMain::setGravity(const Auto<cv::Vec3f>& gravity) {
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
