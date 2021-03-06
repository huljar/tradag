#include <TraDaG/Simulator.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <OGRE/OgreMath.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/lexical_cast.hpp>

#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace TraDaG;

Simulator::Simulator(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams)
    : Simulator()
{
    DEBUG_OUT("Constructing Simulator with OpenCV matrices");

    // Perform default initialization
    init(depthImage, rgbImage, cameraParams);
}

Simulator::Simulator(const std::string& depthImagePath, const std::string& rgbImagePath, const CameraManager& cameraParams)
    : Simulator()
{
    DEBUG_OUT("Constructing Simulator with image paths:");
    DEBUG_OUT("    Depth image: " << depthImagePath);
    DEBUG_OUT("    RGB image:   " << rgbImagePath);

    // Load images from specified paths
    cv::Mat depthImage = cv::imread(depthImagePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    cv::Mat rgbImage = cv::imread(rgbImagePath, CV_LOAD_IMAGE_COLOR);

    // Throw an exception if any of the images was not loaded correctly
    if(!depthImage.data || !rgbImage.data)
        throw std::runtime_error("Unable to load at least one of the following images: \""
                                 + depthImagePath + "\", \"" + rgbImagePath + "\"");

    // Continue with default initialization
    init(depthImage, rgbImage, cameraParams);
}

Simulator::Simulator()
    : mRGBDScene(nullptr)
    , mMaxAttempts(Defaults::MaxAttempts)
    , mShowPreviewWindow(Defaults::ShowPreviewWindow)
    , mShowPhysicsAnimation(Defaults::ShowPhysicsAnimation)
    , mMarkInlierSet(Defaults::MarkInlierSet)
    , mDrawBulletShapes(Defaults::DrawBulletShapes)
    , mGravity(Defaults::Gravity)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
}

Simulator::~Simulator() {
    destroyAllObjects();
    OgreWindow::getSingletonPtr()->invalidate(mRGBDScene);
    delete mRGBDScene;
}

Simulator::Simulator(Simulator&& other)
    : mRGBDScene(other.mRGBDScene)
    , mObjects(std::move(other.mObjects))
    , mGroundPlane(std::move(other.mGroundPlane))
    , mMaxAttempts(other.mMaxAttempts)
    , mShowPreviewWindow(other.mShowPreviewWindow)
    , mShowPhysicsAnimation(other.mShowPhysicsAnimation)
    , mMarkInlierSet(other.mMarkInlierSet)
    , mDrawBulletShapes(other.mDrawBulletShapes)
    , mGravity(std::move(other.mGravity))
    , mRandomEngine(std::move(other.mRandomEngine))
{
    other.mRGBDScene = nullptr;
    other.mObjects.clear();
}

Simulator& Simulator::operator=(Simulator&& other) {
    // Destroy self
    destroyAllObjects();
    OgreWindow::getSingletonPtr()->invalidate(mRGBDScene);
    delete mRGBDScene;

    // Move-assign members
    mRGBDScene = other.mRGBDScene;
    mObjects = std::move(other.mObjects);
    mGroundPlane = std::move(other.mGroundPlane);

    mMaxAttempts = other.mMaxAttempts;
    mShowPreviewWindow = other.mShowPreviewWindow;
    mShowPhysicsAnimation = other.mShowPhysicsAnimation;
    mMarkInlierSet = other.mMarkInlierSet;
    mDrawBulletShapes = other.mDrawBulletShapes;
    mGravity = std::move(other.mGravity);

    mRandomEngine = std::move(other.mRandomEngine);

    // Make other resource-less
    other.mRGBDScene = nullptr;
    other.mObjects.clear();

    return *this;
}

void Simulator::init(const cv::Mat& depthImage, const cv::Mat& rgbImage, const CameraManager& cameraParams) {
    if(!(depthImage.rows == rgbImage.rows && depthImage.cols == rgbImage.cols))
        throw std::invalid_argument("Supplied images do not have the same dimensions");

    DEBUG_OUT("Image resolution is " << depthImage.cols << "x" << depthImage.rows);

    // Create scene from RGB and depth image
    mRGBDScene = new RGBDScene(OgreWindow::getSingletonPtr()->getSceneManager(),
                               depthImage, rgbImage, cameraParams);
}

DroppableObject* Simulator::createObject(const std::string& meshName) {
    DroppableObject* obj = new DroppableObject(meshName, OgreWindow::getSingletonPtr()->getSceneManager());
    mObjects.push_back(obj);
    return obj;
}

void Simulator::destroyObject(DroppableObject* object) {
    ObjectVec::iterator objPos = std::find(mObjects.begin(), mObjects.end(), object);
    if(objPos != mObjects.end())
        mObjects.erase(objPos);

    OgreWindow::getSingletonPtr()->invalidate(object);
    delete object;
}

void Simulator::destroyObject(unsigned int index) {
    if(index < mObjects.size())
        throw std::out_of_range("Index is out of bounds");

    OgreWindow::getSingletonPtr()->invalidate(mObjects[index]);
    delete mObjects[index];
    mObjects.erase(mObjects.begin() + index);
}

void Simulator::destroyAllObjects() {
    OgreWindow::getSingletonPtr()->invalidate(mObjects, nullptr);

    for(ObjectVec::iterator it = mObjects.begin(); it != mObjects.end(); ++it)
        delete *it;

    mObjects.clear();
}

ObjectVec::iterator Simulator::beginObjects() {
    return mObjects.begin();
}

ObjectVec::const_iterator Simulator::beginObjects() const {
    return mObjects.cbegin();
}

ObjectVec::iterator Simulator::endObjects() {
    return mObjects.end();
}

ObjectVec::const_iterator Simulator::endObjects() const {
    return mObjects.cend();
}

Simulator::DropResult Simulator::execute() {
    DEBUG_OUT("Executing simulation with the following parameters:");
    DEBUG_OUT("    Number of objects: " << mObjects.size());
    DEBUG_OUT("    Plane normal: " << mGroundPlane.ogrePlane().normal);
    DEBUG_OUT("    Number of plane vertices: " << mGroundPlane.vertices().size());
    DEBUG_OUT("    Max attempts: " << mMaxAttempts);
    DEBUG_OUT("    Gravity: " << (mGravity.automate
                                  ? std::string("auto")
                                  : "[" + boost::lexical_cast<std::string>(mGravity.manualValue[0]) + ", "
                                        + boost::lexical_cast<std::string>(mGravity.manualValue[1]) + ", "
                                        + boost::lexical_cast<std::string>(mGravity.manualValue[2]) + "]"));

    // Check if objects were added
    if(mObjects.size() == 0) {
        DEBUG_OUT("No objects were added to the simulation");
        return DropResult(DropStatus::NO_OBJECTS, cv::Mat(), cv::Mat());
    }

    // Check if the plane is defined
    if(!mGroundPlane.isPlaneDefined()) {
        DEBUG_OUT("No valid plane was set");
        return DropResult(DropStatus::PLANE_UNDEFINED, cv::Mat(), cv::Mat());
    }

    OgreWindow& ogreWindow = OgreWindow::getSingleton();
    const Ogre::Plane& groundPlane = mGroundPlane.ogrePlane();

    // Calculate gravity vector
    Ogre::Vector3 gravity = mGravity.automate
                            ? groundPlane.normal.normalisedCopy() * Defaults::Gravity.manualValue[1]
                            : cvToOgre(mGravity.manualValue);

    // Abort if the angle between plane normal and gravity vector is too large
    if(!mGravity.automate && groundPlane.normal.angleBetween(-gravity) > Constants::MaxPlaneNormalToGravityAngle)
        return DropResult(DropStatus::PLANE_TOO_STEEP, cv::Mat(), cv::Mat());

    // Calculate initial rotations
    for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
        Auto<cv::Matx33f> rotation = (*it)->getInitialRotation();
        if(rotation.automate) {
            rotation.manualValue = ogreToCv(computeRotation((*it)->getInitialAzimuth(), gravity));
            (*it)->setInitialRotation(rotation);
        }
    }

    // Mark the plane inlier set if requested
    if(mMarkInlierSet && mShowPreviewWindow)
        ogreWindow.markVertices(mGroundPlane.vertices());
    else
        ogreWindow.unmarkVertices(true);

    // Matrices to store the best result
    cv::Mat bestAttemptDepthImage;
    cv::Mat bestAttemptRGBImage;
    float bestAttemptScore;

    bool optimalSolution;

    // Loop to restart if the user chooses so
    OgreWindow::UserAction action = OgreWindow::UserAction::NO_CHOICE;
    do {
        // If animation is requested, display the window now
        if(mShowPreviewWindow && mShowPhysicsAnimation && ogreWindow.hidden())
            ogreWindow.show();

        bestAttemptScore = std::numeric_limits<float>::infinity();

        optimalSolution = false;
        unsigned int attempt = 0;

        // Execution loop - perform simulations until the criteria are met or the maximum number of attempts was reached
        do {
            ++attempt;
            DEBUG_OUT("Starting attempt " << attempt);

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
            OgreWindow::SimulationResult result = ogreWindow.startSimulation(mObjects, mRGBDScene, mGroundPlane, gravity, mDrawBulletShapes, mShowPreviewWindow && mShowPhysicsAnimation);

            if(result == OgreWindow::SimulationResult::TIMEOUT) {
                DEBUG_OUT("Simulation timed out");
                continue;
            }
            else if(result == OgreWindow::SimulationResult::ABORTED) {
                action = OgreWindow::UserAction::ABORT;
                break;
            }

            // Calculate score of this result
            float score = 0.0;
            std::vector<std::pair<float, DroppableObject::PixelInfoMap>> objectInfos(mObjects.size());
            for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {

                float occlusion;
                unsigned short distance;
                DroppableObject::PixelInfoMap pixelInfo;
                bool onPlane;
                if(!ogreWindow.queryObjectInfo(*it, mGroundPlane, occlusion, distance, pixelInfo, onPlane)) {
                    DEBUG_OUT("Failed to retrieve object parameters for at least one object");

                    score = std::numeric_limits<float>::max();
                    break;
                }

                if(!onPlane) {
                    DEBUG_OUT("At least one object does not rest on the plane after dropping");

                    score = std::numeric_limits<float>::max();
                    break;
                }

                // Add score value for occlusion
                std::pair<float, float> desiredOcclusion = (*it)->getDesiredOcclusion();
                score += Constants::ScoreOcclusionWeight *
                         (*it)->getScoreWeight() *
                         distanceToIntervalSquared(occlusion, desiredOcclusion.first, desiredOcclusion.second);

                // Add score value for distance
                std::pair<unsigned short, unsigned short> desiredDistance = (*it)->getDesiredDistance();
                score += Constants::ScoreDistanceWeight *
                         (*it)->getScoreWeight() *
                         distanceToIntervalSquared(distance, desiredDistance.first, desiredDistance.second);

                // Store parameters of this object
                objectInfos[std::distance(beginObjects(), it)] = std::make_pair(occlusion, std::move(pixelInfo));
            }

            DEBUG_OUT("Score of this simulation: " << score);

            if(score < bestAttemptScore) {
                DEBUG_OUT("Score is better than the previous best, registering current attempt as new best");

                // Render depth and RGB images
                cv::Mat depthRender, rgbRender;
                if(!ogreWindow.render(depthRender, rgbRender)) {
                    DEBUG_OUT("Unable to render current attempt, skipping to next");
                    continue;
                }

                // Store this attempt
                bestAttemptDepthImage = depthRender;
                bestAttemptRGBImage = rgbRender;
                bestAttemptScore = score;

                for(ObjectVec::iterator it = beginObjects(); it != endObjects(); ++it) {
                    Ogre::SceneNode* node = (*it)->getOgreEntity()->getParentSceneNode();
                    size_t idx = std::distance(beginObjects(), it);

                    Ogre::Matrix3 rot;
                    node->getOrientation().ToRotationMatrix(rot);
                    (*it)->setFinalRotation(ogreToCv(rot));
                    (*it)->setFinalPosition(ogreToCv(node->getPosition()));
                    (*it)->setFinalOcclusion(objectInfos[idx].first);
                    (*it)->setFinalObjectCoords(objectInfos[idx].second);
                }

                if(score <= 0.0) {
                    DEBUG_OUT("Found an optimal solution");
                    optimalSolution = true;
                }
            }
        } while(!optimalSolution && attempt < mMaxAttempts);

        DEBUG_OUT("Finished simulation with the following result:");
        DEBUG_OUT("    Result\'s score: " << bestAttemptScore);
        DEBUG_OUT("    Result is optimal: " << std::boolalpha << optimalSolution << std::noboolalpha);
        DEBUG_OUT("    Simulations performed: " << attempt);

        // Restore the object poses of the best attempt found
        for(ObjectVec::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
            Ogre::SceneNode* node = (*it)->getOgreEntity()->getParentSceneNode();
            node->setOrientation(Ogre::Quaternion(cvToOgre((*it)->getFinalRotation())));
            node->setPosition(cvToOgre((*it)->getFinalPosition()));
        }

        // If a preview without animation was requested, display the window now
        if(mShowPreviewWindow && !mShowPhysicsAnimation && ogreWindow.hidden())
            ogreWindow.show();

        // Select an action describing what to do with the result
        if(!ogreWindow.hidden()) {
            action = ogreWindow.promptUserAction();
            ogreWindow.hide();
        }
    } while(action == OgreWindow::UserAction::RESTART);

    // Check user choice
    if(action == OgreWindow::UserAction::NO_CHOICE) {
        // No choice was made (because no preview window was requested)
        if(optimalSolution) {
            DEBUG_OUT("Simulation was successful");
            return DropResult(DropStatus::SUCCESS, bestAttemptDepthImage, bestAttemptRGBImage, bestAttemptScore);
        }

        DEBUG_OUT("Simulation was unsuccessful, performed " << mMaxAttempts << " attempts without optimal solution");
        return DropResult(DropStatus::MAX_ATTEMPTS_REACHED, bestAttemptDepthImage, bestAttemptRGBImage, bestAttemptScore);
    }
    else if(action == OgreWindow::UserAction::KEEP) {
        // Check rendered images
        if(bestAttemptDepthImage.data && bestAttemptRGBImage.data) {
            DEBUG_OUT("Simulation was successful");
            return DropResult(DropStatus::SUCCESS, bestAttemptDepthImage, bestAttemptRGBImage, bestAttemptScore);
        }
    }
    else if(action == OgreWindow::UserAction::DISCARD) {
        DEBUG_OUT("Simulation result was discarded by the user");
        return DropResult(DropStatus::USER_DISCARDED, cv::Mat(), cv::Mat());
    }
    else if(action == OgreWindow::UserAction::ABORT) {
        DEBUG_OUT("Simulation was canceled by the user");
        return DropResult(DropStatus::USER_ABORTED, cv::Mat(), cv::Mat());
    }

    DEBUG_OUT("An unknown error occurred");
    return DropResult(DropStatus::UNKNOWN_ERROR, cv::Mat(), cv::Mat());
}

Ogre::Vector3 Simulator::computePosition(const std::vector<Ogre::Vector3>& inliers, const Ogre::Vector3& gravity) {
    std::uniform_int_distribution<size_t> distribution(0, inliers.size() - 1);
    Ogre::Vector3 point = inliers[distribution(mRandomEngine)];

    return point - Constants::ObjectDropDistance * gravity.normalisedCopy();
}

Ogre::Matrix3 Simulator::computeRotation(float azimuth, const Ogre::Vector3& gravity) const {
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
    float azimuthRadians = Ogre::Degree(azimuth).valueRadians();
    float a_cos = std::cos(azimuthRadians);
    float a_sin = std::sin(azimuthRadians);
    rotation = rotation * Ogre::Matrix3(a_cos,  0,  a_sin,
                                        0,      1,      0,
                                        -a_sin, 0,  a_cos);

    return rotation;
}

float Simulator::distanceToIntervalSquared(float value, float min, float max) const {
    if(value < min)
        return std::pow(min - value, 2);

    if(value > max)
        return std::pow(value - max, 2);

    return 0.0;
}

RGBDScene* Simulator::getRGBDScene() const {
    return mRGBDScene;
}

GroundPlane Simulator::getGroundPlane() const {
    return mGroundPlane;
}

void Simulator::setGroundPlane(const GroundPlane& groundPlane) {
    mGroundPlane = groundPlane;

    // Ensure that the camera lies on the positive side of the plane
    Ogre::Plane plane = mGroundPlane.getOgrePlane();
    if(plane.getDistance(OgreWindow::getSingletonPtr()->getInitialCameraPosition()) < 0) {
        plane.normal = -plane.normal;
        plane.d = -plane.d;
        mGroundPlane.setOgrePlane(plane);
    }
}

unsigned int Simulator::getMaxAttempts() const {
    return mMaxAttempts;
}

void Simulator::setMaxAttempts(unsigned int maxAttempts) {
    mMaxAttempts = maxAttempts;
}

bool Simulator::showPreviewWindow() const {
    return mShowPreviewWindow;
}

void Simulator::setShowPreviewWindow(bool showWindow) {
    mShowPreviewWindow = showWindow;
}

bool Simulator::showPhysicsAnimation() const {
    return mShowPhysicsAnimation;
}

void Simulator::setShowPhysicsAnimation(bool showAnimation) {
    mShowPhysicsAnimation = showAnimation;
}

bool Simulator::debugMarkInlierSet() const {
    return mMarkInlierSet;
}

void Simulator::setDebugMarkInlierSet(bool markInliers) {
    mMarkInlierSet = markInliers;
}

bool Simulator::debugDrawBulletShapes() const {
    return mDrawBulletShapes;
}

void Simulator::setDebugDrawBulletShapes(bool drawShapes) {
    mDrawBulletShapes = drawShapes;
}

Auto<cv::Vec3f> Simulator::getGravity() const {
    return mGravity;
}

void Simulator::setGravity(const Auto<cv::Vec3f>& gravity) {
    mGravity = gravity;
}
