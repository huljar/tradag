#ifndef CVLDWRAPPER_H
#define CVLDWRAPPER_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/SceneAnalyzer.h>
#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

#include <random>
#include <string>
#include <utility>
#include <vector>

namespace TraDaG {
    class CVLDWrapper;
}

class TraDaG::CVLDWrapper
{
public:
    struct TrainingImage {
        TrainingImage()
        {
        }

        // TODO: better return value for precompute
        TrainingImage(const cv::Mat_<cv::Vec3b>& bgr, const cv::Mat_<ushort>& depth, const cv::Mat_<cv::Vec3s>& obj, double occlusion,
                      const cv::Point3d& translation, const cv::Mat_<double>& rotation, unsigned int imageID)
            : bgr(bgr), depth(depth), obj(obj), occlusion(occlusion), translation(translation), rotation(rotation), imageID(imageID)
        {
        }

        cv::Mat_<cv::Vec3b> bgr;
        cv::Mat_<ushort> depth;
        cv::Mat_<cv::Vec3s> obj;

        double occlusion;
        cv::Point3d translation;
        cv::Mat_<double> rotation;
        unsigned int imageID;
    };

    CVLDWrapper(const std::string& datasetPath, const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxLoadImages = 0);

    bool precomputePlaneInfo(const std::vector<std::string>& labels, const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);
    bool precomputePlaneInfo(const std::string& label, const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(double occlusionMin = 0.0, double occlusionMax = 1.0);

    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(const cv::Mat_<double>& rotation, const cv::Point3d& throwingDirection,
                                                                     double occlusionMin = 0.0, double occlusionMax = 1.0);

    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(const cv::Mat_<double>& rotation, double rotationTolerance,
                                                                     unsigned short distanceMin, unsigned short distanceMax,
                                                                     double occlusionMin = 0.0, double occlusionMax = 1.0);

    std::vector<std::string> getLabelsToUse() const;
    std::vector<std::string>& labelsToUse();
    const std::vector<std::string>& labelsToUse() const;
    void setLabelsToUse(const std::vector<std::string>& labelsToUse);

    bool getComputePlaneIfNoFile() const;
    void setComputePlaneIfNoFile(bool computePlaneIfNoFile);

    unsigned int getActiveObject() const;
    void setActiveObject(unsigned int objectID);

    unsigned int getNumObjects();

    float getObjectScale() const;
    void setObjectScale(float objectScale);

    float getObjectMass() const;
    void setObjectMass(float objectMass);

    float getObjectRestitution() const;
    void setObjectRestitution(float objectRestitution);

    float getObjectFriction() const;
    void setObjectFriction(float objectFriction);

    float getPlaneRestitution() const;
    void setPlaneRestitution(float planeRestitution);

    float getPlaneFriction() const;
    void setPlaneFriction(float planeFriction);

    unsigned int getMaxAttempts() const;
    void setMaxAttempts(unsigned int maxAttempts);

    bool getShowPreviewWindow() const;
    void setShowPreviewWindow(bool showPreviewWindow);

    bool getShowPhysicsAnimation() const;
    void setShowPhysicsAnimation(bool showPhysicsAnimation);

    Auto<cv::Vec3f> getGravity() const;
    void setGravity(const Auto<cv::Vec3f>& gravity);

    static std::vector<std::string>& availableObjects();

private:
    TrainingImage constructTrainingImage(const cv::Mat& bgr, const cv::Mat& depth, const DroppableObject::PixelInfoMap& pixelInfo, float occlusion,
                                         const cv::Vec3f& translation, const cv::Matx33f& rotation, unsigned int imageID) const;

    cv::Mat_<double> computeAlignRotation(const cv::Vec3f& a, const cv::Vec3f& b);

    bool checkObjectID() const;
    bool checkObjectID(unsigned int objectID) const;

    SceneAnalyzer mSceneAnalyzer;

    std::vector<std::string> mLabelsToUse;

    bool mComputePlaneIfNoFile;

    unsigned int mActiveObject;

    float mObjectScale;

    float mObjectMass;
    float mObjectRestitution;
    float mObjectFriction;

    float mPlaneRestitution;
    float mPlaneFriction;

    unsigned int mMaxAttempts;

    bool mShowPreviewWindow;
    bool mShowPhysicsAnimation;

    Auto<cv::Vec3f> mGravity;

    std::default_random_engine mRandomEngine;

    static std::vector<std::string> msObjects;

};

#endif // CVLDWRAPPER_H
