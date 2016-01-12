#include <TraDaG/tradagmain.h>

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <limits>

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
    , mRGBDScene(NULL)
    , mImageLabeling(NULL)
    , mMaxAttempts(Defaults::MaxAttempts)
    , mShowPreviewWindow(Defaults::ShowPreviewWindow)
    , mShowPhysicsAnimation(Defaults::ShowPhysicsAnimation)
    , mMarkInlierSet(Defaults::MarkInlierSet)
    , mDrawBulletShapes(Defaults::DrawBulletShapes)
    , mGravity(Defaults::Gravity)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
}

TradagMain::~TradagMain() {
    destroyAllObjects();

    delete mImageLabeling;
    delete mRGBDScene;
    delete mOgreWindow;
}

void TradagMain::init(const cv::Mat& depthImage, const cv::Mat& rgbImage,
                      const cv::Mat& labelImage, const LabelMap& labelMap,
                      const cv::Vec2f& depthPrincipalPoint, const cv::Vec2f& depthFocalLength,
                      const cv::Vec2f& rgbPrincipalPoint, const cv::Vec2f& rgbFocalLength,
                      const cv::Matx33f& rotation, const cv::Vec3f& translation,
                      MapMode mapMode, LabelMode labelMode) {

    // Create OGRE window (hidden by default)
    mOgreWindow = new OgreWindow();
    // TODO: light position?

    // Create scene from RGB and depth image
    mRGBDScene = new RGBDScene(Ogre::String(Strings::RgbdSceneName), mOgreWindow->getSceneManager(), depthImage, rgbImage,
                               Ogre::Vector2(depthPrincipalPoint[0], depthPrincipalPoint[1]),
                               Ogre::Vector2(depthFocalLength[0], depthFocalLength[1]),
                               Ogre::Vector2(rgbPrincipalPoint[0], rgbPrincipalPoint[1]),
                               Ogre::Vector2(rgbFocalLength[0], rgbFocalLength[1]),
                               convertCvMatToOgreMat(rotation), Ogre::Vector3(translation[0], translation[1], translation[2]),
                               mapMode);

    // Create image labeling from label image
    mImageLabeling = new ImageLabeling(labelImage, labelMap, labelMode);

    // Register the RGBD scene with OGRE
    mOgreWindow->setScene(mRGBDScene, true);
}

DroppableObject* TradagMain::createObject(const std::string& meshName) {
    DroppableObject* obj = new DroppableObject(meshName, mOgreWindow->getSceneManager());
    mObjects.push_back(obj);
    return obj;
}

void TradagMain::destroyObject(DroppableObject* object) {
    ObjectVec::iterator objPos = std::find(mObjects.begin(), mObjects.end(), object);
    if(objPos != mObjects.end())
        mObjects.erase(objPos);

    delete object;
}

void TradagMain::destroyObject(unsigned int index) {
    assert(index < mObjects.size());
    delete mObjects[index];
    mObjects.erase(mObjects.begin() + index);

}

void TradagMain::destroyAllObjects() {
    for(ObjectVec::iterator it = mObjects.begin(); it != mObjects.end(); ++it)
        delete *it;

    mObjects.clear();
}

ObjectVec::iterator TradagMain::beginObjects() {
    return mObjects.begin();
}

ObjectVec::const_iterator TradagMain::beginObjects() const {
    return mObjects.cbegin();
}

ObjectVec::iterator TradagMain::endObjects() {
    return mObjects.end();
}

ObjectVec::const_iterator TradagMain::endObjects() const {
    return mObjects.cend();
}

ObjectDropResult TradagMain::execute() {
    const Ogre::Plane& groundPlane = mGroundPlane.ogrePlane();

    // Calculate gravity vector
    Ogre::Vector3 gravity = mGravity.automate
                            ? groundPlane.normal.normalisedCopy() * Defaults::Gravity.manualValue[1]
                            : Ogre::Vector3(mGravity.manualValue[0], mGravity.manualValue[1], mGravity.manualValue[2]);

    // Abort if the angle between plane normal and gravity vector is too large
    if(!mGravity.automate && groundPlane.normal.angleBetween(-gravity) > Constants::MaxPlaneNormalToGravityAngle)
        return ObjectDropResult(OD_PLANE_TOO_STEEP, cv::Mat(), cv::Mat());

    // Calculate initial rotations
    for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
        Auto<cv::Matx33f> rotation = (*it)->getInitialRotation();
        if(rotation.automate) {
            rotation.manualValue = convertOgreMatToCvMat(computeRotation((*it)->getInitialAzimuth(), gravity));
            (*it)->setInitialRotation(rotation);
        }
    }

    // Mark the plane inlier set if requested
    if(mMarkInlierSet && mShowPreviewWindow)
        mOgreWindow->markVertices(mGroundPlane.vertices());

    // Matrices to store the best result
    cv::Mat bestAttemptDepthImage;
    cv::Mat bestAttemptRGBImage;

    // Loop to restart if the user chooses so
    UserAction action = UA_KEEP;
    do {
        // If animation is requested, display the window now
        if(mShowPreviewWindow && mShowPhysicsAnimation && mOgreWindow->hidden())
            mOgreWindow->show();

        float bestAttemptScore = std::numeric_limits<float>::max(); // TODO: weighting object for score influence?

        bool solutionFound = false;
        unsigned int attempt = 0;

        // Execution loop - perform simulations until the criteria are met or the maximum number of attempts was reached
        do {
            ++attempt;

            // Calculate initial positions (this is done for each attempt because it contains randomization)
            for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
                Auto<cv::Vec3f> position = (*it)->getInitialPosition();
                if(position.automate) {
                    Ogre::Vector3 posValue = computePosition(mGroundPlane.vertices(), gravity);
                    position.manualValue = cv::Vec3f(posValue.x, posValue.y, posValue.z);
                    (*it)->setInitialPosition(position);
                }
            }

            // Simulate (with animation only if needed)
            // This function does not return until the simulation is completed
            SimulationResult result = mOgreWindow->startSimulation(mObjects, mGroundPlane, gravity, mDrawBulletShapes, mShowPreviewWindow && mShowPhysicsAnimation);

            if(result == SR_TIMEOUT) {
                continue;
            }
            else if(result == SR_ABORTED) {
                action = UA_ABORT;
                break;
            }

            // Calculate score of this result
            float score = 0.0;
            std::vector<float> occlusions;
            for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
                // TODO: check if object still on plane

                Ogre::Real occlusion;
                std::vector<std::pair<Ogre::Vector3, bool>> pixelInfo;
                bool onPlane;
                if(!mOgreWindow->queryObjectInfo(*it, occlusion, pixelInfo, onPlane))
                    continue;

                std::pair<float, float> desiredOcclusion = (*it)->getDesiredOcclusion();

                occlusions.push_back(occlusion);
                score += distanceToIntervalSquared(occlusion, desiredOcclusion.first, desiredOcclusion.second);
            }

            if(score < bestAttemptScore) {
                // Hide vertex markings
                mOgreWindow->unmarkVertices();

                // Render depth and RGB images
                cv::Mat depthRender, rgbRender;
                if(!mOgreWindow->render(depthRender, rgbRender))
                    continue;

                // Re-mark vertices (if any were previously marked)
                mOgreWindow->markVertices();

                // Store this attempt
                bestAttemptDepthImage = depthRender;
                bestAttemptRGBImage = rgbRender;
                bestAttemptScore = score;

                for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
                    Ogre::SceneNode* node = (*it)->getOgreEntity()->getParentSceneNode();

                    Ogre::Matrix3 rot;
                    node->getOrientation().ToRotationMatrix(rot);
                    (*it)->setFinalRotation(rot);
                    (*it)->setFinalPosition(node->getPosition());
                    (*it)->setFinalOcclusion(occlusions[std::distance(beginObjects(), it)]);
                }

                if(score <= 0.0)
                    solutionFound = true;
            }
        } while(!solutionFound && attempt < mMaxAttempts);

        // Restore the object poses of the best attempt found
        const std::vector<Ogre::Entity*>& entities = mOgreWindow->objectEntities();
        if(entities.size() != mObjects.size())
            return ObjectDropResult(OD_UNKNOWN_ERROR, cv::Mat(), cv::Mat());

        for(size_t i = 0; i < entities.size(); ++i) {
            Ogre::SceneNode* node = entities[i]->getParentSceneNode();
            node->setOrientation(Ogre::Quaternion(convertCvMatToOgreMat(mObjects[i]->getFinalRotation())));
            cv::Vec3f pos = mObjects[i]->getFinalPosition();
            node->setPosition(pos[0], pos[1], pos[2]);
        }

        // If a preview without animation was requested, display the window now
        if(mShowPreviewWindow && !mShowPhysicsAnimation && mOgreWindow->hidden())
            mOgreWindow->show();

        // Select an action describing what to do with the result
        if(!mOgreWindow->hidden()) {
            action = mOgreWindow->promptUserAction();
            mOgreWindow->hide();
        }
    } while(action == UA_RESTART);

    // Check user choice
    if(action == UA_KEEP) {
        // Check rendered images
        if(bestAttemptDepthImage.data && bestAttemptRGBImage.data) {
            return ObjectDropResult(OD_SUCCESS, bestAttemptDepthImage, bestAttemptRGBImage);
        }
    }
    else if(action == UA_ABORT) {
        return ObjectDropResult(OD_USER_ABORTED, cv::Mat(), cv::Mat());
    }

    return ObjectDropResult(OD_UNKNOWN_ERROR, cv::Mat(), cv::Mat());
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

float TradagMain::distanceToIntervalSquared(float value, float min, float max) const {
    if(value < min)
        return std::pow(min - value, 2);

    if(value > max)
        return std::pow(value - max, 2);

    return 0.0;
}

cv::Mat TradagMain::getDepthImage() const {
    return mRGBDScene->getDepthImage();
}

cv::Mat TradagMain::getRgbImage() const {
    return mRGBDScene->getRgbImage();
}

cv::Mat TradagMain::getLabelImage() const {
    return mImageLabeling->getLabelImage();
}

LabelMap TradagMain::getLabelMap() const {
    return mImageLabeling->getLabelMap();
}

void TradagMain::setNewScene(const cv::Mat& depthImage, const cv::Mat& rgbImage, const cv::Mat& labelImage, const LabelMap& labelMap) {
    // Get old camera parameters
    Ogre::Vector2 tmpDPP = mRGBDScene->getDepthPrincipalPoint();
    Ogre::Vector2 tmpDFL = mRGBDScene->getDepthFocalLength();
    Ogre::Vector2 tmpRPP = mRGBDScene->getRgbPrincipalPoint();
    Ogre::Vector2 tmpRFL = mRGBDScene->getRgbFocalLength();
    Ogre::Matrix3 tmpRot = mRGBDScene->getRotation();
    Ogre::Vector3 tmpTrans = mRGBDScene->getTranslation();
    MapMode tmpMM = mRGBDScene->getMapMode();
    LabelMode tmpLM = mImageLabeling->getLabelMode();

    // Delete old scene and create a new one
    delete mRGBDScene;
    mRGBDScene = new RGBDScene(Strings::RgbdSceneName, mOgreWindow->getSceneManager(), depthImage, rgbImage, tmpDPP, tmpDFL, tmpRPP, tmpRFL, tmpRot, tmpTrans, tmpMM);

    // Delete old labeling and create a new one
    delete mImageLabeling;
    mImageLabeling = new ImageLabeling(labelImage, labelMap, tmpLM);

    // Notify OgreWindow of the new scene
    mOgreWindow->setScene(mRGBDScene, true);
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

GroundPlane TradagMain::getGroundPlane() const {
    return mGroundPlane;
}

void TradagMain::setGroundPlane(const GroundPlane& groundPlane) {
    mGroundPlane = groundPlane;

    // Ensure that the camera lies on the positive side of the plane
    Ogre::Plane& plane = mGroundPlane.ogrePlane();
    if(plane.getDistance(mOgreWindow->getInitialCameraPosition()) < 0) {
        plane.normal = -plane.normal;
        plane.d = -plane.d;
    }
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

OgreWindow*TradagMain::getOgreWindow() const {
    return mOgreWindow;
}

RGBDScene*TradagMain::getRGBDScene() const {
    return mRGBDScene;
}

ImageLabeling*TradagMain::getImageLabeling() const {
    return mImageLabeling;
}
