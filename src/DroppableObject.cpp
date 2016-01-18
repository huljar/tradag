#include <TraDaG/DroppableObject.h>

#include <stdexcept>
#include <utility>

using namespace TraDaG;

DroppableObject::DroppableObject(const std::string& meshName, Ogre::SceneManager* sceneManager)
    : mSceneMgr(sceneManager)
    , mDesiredOcclusion(Defaults::ObjectDesiredOcclusion)
    , mFinalOcclusion(0)
    , mDesiredDistance(Defaults::ObjectDesiredDistance)
    , mFinalDistance(0)
    , mMustBeUpright(Defaults::ObjectMustBeUpright)
    , mCastShadows(Defaults::ObjectCastShadows)
    , mScale(Defaults::ObjectScale)
    , mInitialPosition(Defaults::ObjectInitialPosition)
    , mInitialRotation(Defaults::ObjectInitialRotation)
    , mInitialAzimuth(Defaults::ObjectInitialAzimuth)
    , mInitialVelocity(Defaults::ObjectInitialVelocity)
    , mInitialTorque(Defaults::ObjectInitialTorque)
    , mRestitution(Defaults::ObjectRestitution)
    , mFriction(Defaults::ObjectFriction)
    , mMass(Defaults::ObjectMass)
{
    mEntity = sceneManager->createEntity(meshName);
    mEntity->setCastShadows(mCastShadows);
}

DroppableObject::~DroppableObject() {
    mEntity->detachFromParent();
    mSceneMgr->destroyEntity(mEntity);
}

Ogre::Entity* DroppableObject::getOgreEntity() const {
    return mEntity;
}

std::pair<float, float> DroppableObject::getDesiredOcclusion() const {
    return mDesiredOcclusion;
}

void DroppableObject::setDesiredOcclusion(const std::pair<float, float>& occlusion) {
    if(checkOcclusionValid(occlusion.first, occlusion.second))
        mDesiredOcclusion = occlusion;
    else
        throw std::invalid_argument("The provided occlusion interval does not lie in [0, 1] or max is smaller than min");
}

void DroppableObject::setDesiredOcclusion(float minOcclusion, float maxOcclusion) {
    setDesiredOcclusion(std::make_pair(minOcclusion, maxOcclusion));
}

float DroppableObject::getFinalOcclusion() const {
    return mFinalOcclusion;
}

void DroppableObject::setFinalOcclusion(float finalOcclusion) {
    if(finalOcclusion < 0.0 || finalOcclusion > 1.0)
        throw std::invalid_argument("Occlusion must lie in [0, 1] range");

    mFinalOcclusion = finalOcclusion;
}

std::pair<unsigned short, unsigned short> DroppableObject::getDesiredDistance() const {
    return mDesiredDistance;
}

void DroppableObject::setDesiredDistance(const std::pair<unsigned short, unsigned short>& distance) {
    if(checkDistanceValid(distance.first, distance.second))
        mDesiredDistance = distance;
    else
        throw std::invalid_argument("The provided max distance is smaller than min distance");
}

void DroppableObject::setDesiredDistance(unsigned short minDistance, unsigned short maxDistance) {
    setDesiredDistance(std::make_pair(minDistance, maxDistance));
}

unsigned short DroppableObject::getFinalDistance() const {
    return mFinalDistance;
}

void DroppableObject::setFinalDistance(unsigned short finalDistance) {
    mFinalDistance = finalDistance;
}

bool DroppableObject::getMustBeUpright() const {
    return mMustBeUpright;
}

void DroppableObject::setMustBeUpright(bool mustBeUpright) {
    mMustBeUpright = mustBeUpright;
}

bool DroppableObject::getCastShadows() const {
    return mCastShadows;
}

void DroppableObject::setCastShadows(bool castShadows) {
    mCastShadows = castShadows;
    mEntity->setCastShadows(castShadows);
}

cv::Vec3f DroppableObject::getScale() const {
    return mScale;
}

void DroppableObject::setScale(const cv::Vec3f& scale) {
    mScale = scale;
}

void DroppableObject::setScale(float scaleX, float scaleY, float scaleZ) {
    mScale = cv::Vec3f(scaleX, scaleY, scaleZ);
}

void DroppableObject::setScale(float scale) {
    mScale = cv::Vec3f(scale, scale, scale);
}

Auto<cv::Vec3f> DroppableObject::getInitialPosition() const {
    return mInitialPosition;
}

void DroppableObject::setInitialPosition(const Auto<cv::Vec3f>& initialPosition) {
    mInitialPosition = initialPosition;
}

cv::Vec3f DroppableObject::getFinalPosition() const {
    return mFinalPosition;
}

void DroppableObject::setFinalPosition(const cv::Vec3f& finalPosition) {
    mFinalPosition = finalPosition;
}

void DroppableObject::setFinalPosition(float finalPositionX, float finalPositionY, float finalPositionZ) {
    mFinalPosition = cv::Vec3f(finalPositionX, finalPositionY, finalPositionZ);
}

Auto<cv::Matx33f> DroppableObject::getInitialRotation() const {
    return mInitialRotation;
}

void DroppableObject::setInitialRotation(const Auto<cv::Matx33f>& initialRotation) {
    mInitialRotation = initialRotation;
}

cv::Matx33f DroppableObject::getFinalRotation() const {
    return mFinalRotation;
}

void DroppableObject::setFinalRotation(const cv::Matx33f& finalRotation) {
    mFinalRotation = finalRotation;
}

float DroppableObject::getInitialAzimuth() const {
    return mInitialAzimuth;
}

void DroppableObject::setInitialAzimuth(float initialAzimuth) {
    mInitialAzimuth = initialAzimuth;
}

cv::Vec3f DroppableObject::getInitialVelocity() const {
    return mInitialVelocity;
}

void DroppableObject::setInitialVelocity(const cv::Vec3f& initialVelocity) {
    mInitialVelocity = initialVelocity;
}

void DroppableObject::setInitialVelocity(float initialVelocityX, float initialVelocityY, float initialVelocityZ) {
    mInitialVelocity = cv::Vec3f(initialVelocityX, initialVelocityY, initialVelocityZ);
}

cv::Vec3f DroppableObject::getInitialTorque() const {
    return mInitialTorque;
}

void DroppableObject::setInitialTorque(const cv::Vec3f& initialTorque) {
    mInitialTorque = initialTorque;
}

void DroppableObject::setInitialTorque(float initialTorqueX, float initialTorqueY, float initialTorqueZ) {
    mInitialTorque = cv::Vec3f(initialTorqueX, initialTorqueY, initialTorqueZ);
}

float DroppableObject::getRestitution() const {
    return mRestitution;
}

void DroppableObject::setRestitution(float restitution) {
    mRestitution = restitution;
}

float DroppableObject::getFriction() const {
    return mFriction;
}

void DroppableObject::setFriction(float friction) {
    mFriction = friction;
}

float DroppableObject::getMass() const {
    return mMass;
}

void DroppableObject::setMass(float mass) {
    mMass = mass;
}

bool DroppableObject::checkOcclusionValid(float min, float max) const {
    if(max < min)
        return false;

    if(min < 0.0 || max > 1.0)
        return false;

    return true;
}

bool DroppableObject::checkDistanceValid(unsigned short min, unsigned short max) const {
    if(max < min)
        return false;

    return true;
}
