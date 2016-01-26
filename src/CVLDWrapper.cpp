#include <TraDaG/CVLDWrapper/CVLDWrapper.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/Simulator.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;

const std::vector<std::string> CVLDWrapper::msObjects({
    "001.mesh",
    "002.mesh",
    "003.mesh",
    "004.mesh",
    "005.mesh",
    "006.mesh",
    "007.mesh",
    "008.mesh",
    "009.mesh",
    "010.mesh",
    "011.mesh",
    "012.mesh",
    "013.mesh"
});

CVLDWrapper::CVLDWrapper(const std::string& datasetPath, const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxLoadImages)
    : mSceneAnalyzer((fs::path(datasetPath) / fs::path("depth")).string(), (fs::path(datasetPath) / fs::path("rgb")).string(),
                     (fs::path(datasetPath) / fs::path("label")).string(), cameraParams, labelMap, maxLoadImages)
    , mActiveObject(0)
    , mObjectScale(Defaults::ObjectScale[0])
    , mObjectMass(Defaults::ObjectMass)
    , mObjectRestitution(Defaults::ObjectRestitution)
    , mObjectFriction(Defaults::ObjectFriction)
    , mPlaneRestitution(Defaults::PlaneRestitution)
    , mPlaneFriction(Defaults::PlaneFriction)
    , mMaxAttempts(Defaults::MaxAttempts)
    , mShowPreviewWindow(Defaults::ShowPreviewWindow)
    , mShowPhysicsAnimation(Defaults::ShowPhysicsAnimation)
    , mGravity(Defaults::Gravity)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
    fs::path planePath = fs::path(datasetPath) / fs::path("plane");
    if(fs::exists(planePath) && fs::is_directory(planePath))
        mSceneAnalyzer.setPlanePath(planePath.string());
}

std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> CVLDWrapper::getTrainingImage(double occlusionMin, double occlusionMax) {
    DEBUG_OUT("Getting training image for occlusion = [" << occlusionMin << ", " << occlusionMax << "]");

    // Get all scenes with labels that should be used
    std::map<unsigned int, GroundPlane> scenes = mSceneAnalyzer.findScenesByPlane(mLabelsToUse, cv::Vec3f(0, 0, 0), 180.0,
                                                                                  0, std::numeric_limits<unsigned short>::max(), true);

    // Get scene IDs
    std::vector<unsigned int> allIDs;
    allIDs.reserve(scenes.size());
    for(std::map<unsigned int, GroundPlane>::iterator it = scenes.begin(); it != scenes.end(); ++it) {
        allIDs.push_back(it->first);
    }

    // Shuffle scene IDs
    std::shuffle(allIDs.begin(), allIDs.end(), mRandomEngine);

    // Simulate until good scene found
    // TODO: if max attempts reached, try again for another scene?
    for(std::vector<unsigned int>::iterator it = allIDs.begin(); it != allIDs.end(); ++it) {
        DEBUG_OUT("Trying simulation for ID " << *it);

        // Get Simulator and create object
        Simulator simulator = mSceneAnalyzer.createSimulator(*it, scenes[*it]);
        simulator.setMaxAttempts(mMaxAttempts);
        simulator.setShowPreviewWindow(mShowPreviewWindow);
        simulator.setShowPhysicsAnimation(mShowPhysicsAnimation);
        simulator.setGravity(mGravity);

        DroppableObject* obj = simulator.createObject(msObjects[mActiveObject]);
        obj->setDesiredOcclusion(occlusionMin, occlusionMax);
        obj->setScale(mObjectScale);
        obj->setMass(mObjectMass);
        obj->setRestitution(mObjectRestitution);
        obj->setFriction(mObjectFriction);

        Simulator::DropResult result = simulator.execute();
        if(result.status == Simulator::DropStatus::SUCCESS) {
            DEBUG_OUT("Simulation successful, returning");

            return std::make_pair(
                constructTrainingImage(result.rgbImage, result.depthImage, obj->getFinalObjectCoords(), obj->getFinalOcclusion(),
                                       obj->getFinalPosition(), obj->getFinalRotation(), *it),
                result.status
            );
        }
        else if(result.status == Simulator::DropStatus::USER_ABORTED) {
            DEBUG_OUT("Simulation aborted by user, returning");

            return std::make_pair(TrainingImage(), result.status);
        }
    }

    return std::make_pair(TrainingImage(), Simulator::DropStatus::UNKNOWN_ERROR);
}

std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> CVLDWrapper::getTrainingImage(const cv::Mat_<double>& rotation, const cv::Point3d& throwingDirection,
                                                                                           double occlusionMin, double occlusionMax) {

    DEBUG_OUT("Getting training image for occlusion = [" << occlusionMin << ", " << occlusionMax << "] with specific initial rotation and throwing direction");

    // Get all scenes with labels that should be used
    std::map<unsigned int, GroundPlane> scenes = mSceneAnalyzer.findScenesByPlane(mLabelsToUse, cv::Vec3f(0, 0, 0), 180.0,
                                                                                  0, std::numeric_limits<unsigned short>::max(), true);

    // Get scene IDs
    std::vector<unsigned int> allIDs;
    allIDs.reserve(scenes.size());
    for(std::map<unsigned int, GroundPlane>::iterator it = scenes.begin(); it != scenes.end(); ++it) {
        allIDs.push_back(it->first);
    }

    // Shuffle scene IDs
    std::shuffle(allIDs.begin(), allIDs.end(), mRandomEngine);

    // Simulate until good scene found
    // TODO: if max attempts reached, try again for another scene?
    for(std::vector<unsigned int>::iterator it = allIDs.begin(); it != allIDs.end(); ++it) {
        DEBUG_OUT("Trying simulation for ID " << *it);

        // Get Simulator and create object
        Simulator simulator = mSceneAnalyzer.createSimulator(*it, scenes[*it]);
        simulator.setMaxAttempts(mMaxAttempts);
        simulator.setShowPreviewWindow(mShowPreviewWindow);
        simulator.setShowPhysicsAnimation(mShowPhysicsAnimation);
        simulator.setGravity(mGravity);

        DroppableObject* obj = simulator.createObject(msObjects[mActiveObject]);
        obj->setDesiredOcclusion(occlusionMin, occlusionMax);
        obj->setScale(mObjectScale);
        obj->setMass(mObjectMass);
        obj->setRestitution(mObjectRestitution);
        obj->setFriction(mObjectFriction);
        cv::Matx33f rot(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                        rotation(1, 0), rotation(1, 1), rotation(1, 2),
                        rotation(2, 0), rotation(2, 1), rotation(2, 2));
        obj->setInitialRotation(Auto<cv::Matx33f>(false, rot));
        obj->setInitialVelocity(throwingDirection.x, throwingDirection.y, throwingDirection.z);

        Simulator::DropResult result = simulator.execute();
        if(result.status == Simulator::DropStatus::SUCCESS) {
            DEBUG_OUT("Simulation successful, returning");

            return std::make_pair(
                constructTrainingImage(result.rgbImage, result.depthImage, obj->getFinalObjectCoords(), obj->getFinalOcclusion(),
                                       obj->getFinalPosition(), obj->getFinalRotation(), *it),
                result.status
            );
        }
        else if(result.status == Simulator::DropStatus::USER_ABORTED) {
            DEBUG_OUT("Simulation aborted by user, returning");

            return std::make_pair(TrainingImage(), result.status);
        }
    }

    return std::make_pair(TrainingImage(), Simulator::DropStatus::UNKNOWN_ERROR);
}

std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> CVLDWrapper::getTrainingImage(const cv::Mat_<double>& rotation, double rotationTolerance,
                                                                                           unsigned short distanceMin, unsigned short distanceMax,
                                                                                           double occlusionMin, double occlusionMax) {

    DEBUG_OUT("Getting training image for occlusion = [" << occlusionMin << ", " << occlusionMax << "] with specific final rotation and tolerance");

    // Get all scenes with labels that should be used and suitable normal
    // Plane normal is the 2nd column of the rotation matrix (y-axis)
    cv::Vec3f normal(rotation(0, 1), rotation(1, 1), rotation(2, 1));
    std::map<unsigned int, GroundPlane> scenes = mSceneAnalyzer.findScenesByPlane(mLabelsToUse, normal, rotationTolerance,
                                                                                  distanceMin, distanceMax, true);

    // Get scene IDs
    std::vector<unsigned int> allIDs;
    allIDs.reserve(scenes.size());
    for(std::map<unsigned int, GroundPlane>::iterator it = scenes.begin(); it != scenes.end(); ++it) {
        allIDs.push_back(it->first);
    }

    // Shuffle scene IDs
    std::shuffle(allIDs.begin(), allIDs.end(), mRandomEngine);

    // Simulate until good scene found
    // TODO: if max attempts reached, try again for another scene?
    for(std::vector<unsigned int>::iterator it = allIDs.begin(); it != allIDs.end(); ++it) {
        DEBUG_OUT("Trying simulation for ID " << *it);

        // Get Simulator and create object
        Simulator simulator = mSceneAnalyzer.createSimulator(*it, scenes[*it]);
        simulator.setMaxAttempts(mMaxAttempts);
        simulator.setShowPreviewWindow(mShowPreviewWindow);
        simulator.setShowPhysicsAnimation(mShowPhysicsAnimation);
        simulator.setGravity(mGravity);

        DroppableObject* obj = simulator.createObject(msObjects[mActiveObject]);
        obj->setDesiredOcclusion(occlusionMin, occlusionMax);
        obj->setScale(mObjectScale);
        obj->setMass(mObjectMass);
        obj->setRestitution(mObjectRestitution);
        obj->setFriction(mObjectFriction);

        // Compute rotation
        // Use simulator.getGroundPlane() to ensure that the plane normal faces the correct direction, which is checked when registering the plane with the simulator
        cv::Mat_<double> rotTmp = computeAlignRotation(normal, ogreToCv(simulator.getGroundPlane().ogrePlane().normal)) * rotation;
        cv::Matx33f rot(rotTmp(0, 0), rotTmp(0, 1), rotTmp(0, 2),
                        rotTmp(1, 0), rotTmp(1, 1), rotTmp(1, 2),
                        rotTmp(2, 0), rotTmp(2, 1), rotTmp(2, 2));
        obj->setInitialRotation(Auto<cv::Matx33f>(false, rot));
        obj->setMustBeUpright(true);

        Simulator::DropResult result = simulator.execute();
        if(result.status == Simulator::DropStatus::SUCCESS) {
            DEBUG_OUT("Simulation successful, returning");

            return std::make_pair(
                constructTrainingImage(result.rgbImage, result.depthImage, obj->getFinalObjectCoords(), obj->getFinalOcclusion(),
                                       obj->getFinalPosition(), obj->getFinalRotation(), *it),
                result.status
            );
        }
        else if(result.status == Simulator::DropStatus::USER_ABORTED) {
            DEBUG_OUT("Simulation aborted by user, returning");

            return std::make_pair(TrainingImage(), result.status);
        }
    }

    return std::make_pair(TrainingImage(), Simulator::DropStatus::UNKNOWN_ERROR);
}

CVLDWrapper::TrainingImage CVLDWrapper::constructTrainingImage(const cv::Mat& bgr, const cv::Mat& depth, const DroppableObject::PixelInfoMap& pixelInfo, float occlusion,
                                                               const cv::Vec3f& translation, const cv::Matx33f& rotation, unsigned int imageID) const {
    // Check data types
    if(bgr.type() != CV_8UC3)
        throw std::invalid_argument("Color image does not have CV_8UC3 type");
    if(depth.type() != CV_16U)
        throw std::invalid_argument("Depth image does not have CV_16U type");

    // Build image matrices
    cv::Mat_<cv::Vec3b> retBGR(bgr.clone());
    cv::Mat_<ushort> retDepth(depth.clone());

    // Build object coordinate matrix
    // TODO: (0, 0, 0) can be a valid object coordinate?
    cv::Mat_<cv::Vec3s> retObj(depth.rows, depth.cols, cv::Vec3s(0, 0, 0));
    for(DroppableObject::PixelInfoMap::const_iterator it = pixelInfo.cbegin(); it != pixelInfo.cend(); ++it) {
        if(it->second.second) retObj(it->first) = it->second.first;
    }

    // Build occlusion
    double retOccl = static_cast<double>(occlusion);

    // Build translation vector
    cv::Point3d retTrans(static_cast<double>(translation[0]), static_cast<double>(translation[1]), static_cast<double>(translation[2]));

    // Build rotation matrix
    cv::Mat_<double> retRot(3, 3);
    for(int y = 0; y < 3; ++y) {
        for(int x = 0; x < 3; ++x) {
            retRot(y, x) = static_cast<double>(rotation(y, x));
        }
    }

    // Construct and return TrainingImage
    return TrainingImage(retBGR, retDepth, retObj, retOccl, retTrans, retRot, imageID);
}

cv::Mat_<double> CVLDWrapper::computeAlignRotation(const cv::Vec3f& a, const cv::Vec3f& b) {
    // Find rotation matrix so the object starts upright
    // (see https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d)
    cv::Vec3f v = a.cross(b); // axis to rotate around
    float s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); // sine of angle
    float c = a.dot(b); // cosine of angle

    cv::Mat_<double> rotation = cv::Mat_<double>::eye(3, 3);

    // Only proceed if rotation is not identity (otherwise division by zero can happen if s==0)
    if(v != cv::Vec3f(0, 0, 0) && s != 0) {
        cv::Mat_<double> v_x = (cv::Mat_<double>(3, 3) <<     0, -v[2],  v[1],
                                                           v[2],     0, -v[0],
                                                          -v[1],  v[0],     0); // cross product matrix of v

        rotation = cv::Mat_<double>::eye(3, 3)
                   + v_x
                   + v_x * v_x * ((1 - c) / (s * s));
    }

    return rotation;
}

std::vector<std::string> CVLDWrapper::getLabelsToUse() const {
    return mLabelsToUse;
}

std::vector<std::string>& CVLDWrapper::labelsToUse() {
    return mLabelsToUse;
}

const std::vector<std::string>& CVLDWrapper::labelsToUse() const {
    return mLabelsToUse;
}

void CVLDWrapper::setLabelsToUse(const std::vector<std::string>& labelsToUse) {
    mLabelsToUse = labelsToUse;
}

unsigned int CVLDWrapper::getActiveObject() const {
    return mActiveObject + 1;
}

void CVLDWrapper::setActiveObject(unsigned int objectID) {
    if(objectID > 0 && objectID <= msObjects.size())
        mActiveObject = objectID - 1;
    else
        throw std::invalid_argument("Non-existant object ID given");
}

unsigned int CVLDWrapper::getNumObjects() {
    return msObjects.size();
}

float CVLDWrapper::getObjectScale() const {
    return mObjectScale;
}

void CVLDWrapper::setObjectScale(float objectScale) {
    mObjectScale = objectScale;
}

float CVLDWrapper::getObjectMass() const {
    return mObjectMass;
}

void CVLDWrapper::setObjectMass(float objectMass) {
    mObjectMass = objectMass;
}

float CVLDWrapper::getObjectRestitution() const {
    return mObjectRestitution;
}

void CVLDWrapper::setObjectRestitution(float objectRestitution) {
    mObjectRestitution = objectRestitution;
}

float CVLDWrapper::getObjectFriction() const {
    return mObjectFriction;
}

void CVLDWrapper::setObjectFriction(float objectFriction) {
    mObjectFriction = objectFriction;
}

float CVLDWrapper::getPlaneRestitution() const {
    return mPlaneRestitution;
}

void CVLDWrapper::setPlaneRestitution(float planeRestitution) {
    mPlaneRestitution = planeRestitution;
}

float CVLDWrapper::getPlaneFriction() const {
    return mPlaneFriction;
}

void CVLDWrapper::setPlaneFriction(float planeFriction) {
    mPlaneFriction = planeFriction;
}

unsigned int CVLDWrapper::getMaxAttempts() const {
    return mMaxAttempts;
}

void CVLDWrapper::setMaxAttempts(unsigned int maxAttempts) {
    mMaxAttempts = maxAttempts;
}

bool CVLDWrapper::getShowPreviewWindow() const {
    return mShowPreviewWindow;
}

void CVLDWrapper::setShowPreviewWindow(bool showPreviewWindow) {
    mShowPreviewWindow = showPreviewWindow;
}

bool CVLDWrapper::getShowPhysicsAnimation() const {
    return mShowPhysicsAnimation;
}

void CVLDWrapper::setShowPhysicsAnimation(bool showPhysicsAnimation) {
    mShowPhysicsAnimation = showPhysicsAnimation;
}

Auto<cv::Vec3f> CVLDWrapper::getGravity() const {
    return mGravity;
}

void CVLDWrapper::setGravity(const Auto<cv::Vec3f>& gravity) {
    mGravity = gravity;
}
