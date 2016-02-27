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

/**
 * @brief Wrapper class for CVLD use cases of the framework.
 *
 * This wrapper supports some comfort functions for easily generating training data that conforms to the
 * specification of the CVLD members.
 *
 * @author Julian Harttung
 *
 * @remarks The interface provided here is tailored to specific use cases. If you need a more generic interface,
 * look into SceneAnalyzer and Simulator (and the \ref getting_started_generic_sec "introduction" to the
 * important classes).
 */
class TraDaG::CVLDWrapper
{
public:
    /**
     * @brief Data of a single training image from the CVLDWrapper class.
     *
     * Struct holding the data of a single training image with CVLD-specific types.
     */
    struct TrainingImage {
        /**
         * @brief Default constructor (constructs an empty training image)
         */
        TrainingImage() : occlusion(0.0), imageID(0)
        {
        }

        /**
         * @brief Constructor for a valid training image with all necessary data.
         * @param bgr The color image with BGR channel order.
         * @param depth The depth image with 16 bit per pixel.
         * @param obj Image containing the object coordinates of the dropped object with origin at the object center.
         * Pixels that are not part of the object have value (0, 0, 0).
         * @param occlusion Fraction of the object which is occluded (0.0 = completely visible, 1.0 = completely occluded).
         * @param translation Translation vector of the dropped object.
         * @param rotation 3x3 rotation matrix of the dropped object.
         * @param imageID ID of the selected scene. IDs are assigned in alphabetical order of the depth image file name, starting at 1.
         */
        TrainingImage(const cv::Mat_<cv::Vec3b>& bgr, const cv::Mat_<ushort>& depth, const cv::Mat_<cv::Vec3s>& obj, double occlusion,
                      const cv::Point3d& translation, const cv::Mat_<double>& rotation, unsigned int imageID)
            : bgr(bgr), depth(depth), obj(obj), occlusion(occlusion), translation(translation), rotation(rotation), imageID(imageID)
        {
        }

        /// The color image with BGR channel order.
        cv::Mat_<cv::Vec3b> bgr;
        /// The depth image with 16 bit per pixel.
        cv::Mat_<ushort> depth;
        /// Matrix containing the object coordinates of the dropped object with origin at the object center. Pixels that are not part of the object have value (0, 0, 0).
        cv::Mat_<cv::Vec3s> obj;

        /// Fraction of the object which is occluded (0.0 = completely visible, 1.0 = completely occluded).
        double occlusion;
        /// Translation vector of the dropped object.
        cv::Point3d translation;
        /// 3x3 rotation matrix of the dropped object.
        cv::Mat_<double> rotation;
        /// ID of the selected scene. IDs are assigned in alphabetical order of the depth image file name, starting at 1.
        unsigned int imageID;
    };

    /**
     * @brief Constructor.
     * @param datasetPath Absolute or relative path to the data set base directory.
     * @param cameraParams A CameraManager instance containing all the camera parameters which were used to record the data set.
     * @param labelMap A mapping of string labels to 16 bit unsigned label values. For the <em>NYU Depth V1</em> and <em>NYU Depth
     * V2</em> data sets, sensible label maps are predefined in @c util.h which can be used here.
     * @param maxLoadImages The maximum number of scenes to load from the data set. If this number is smaller than the number of
     * scenes in the data set, some scenes will not be loaded, and the loaded scenes are selected randomly from the data set.
     * Set this to 0 to specify no limit.
     *
     * The class will look for the subdirectories @e depth, @e rgb and @e label in datasetPath, which are expected to contain the depth,
     * color and label images of the scenes. Depth, color and label images are matched to the same scene if they have the same file name.
     *
     * If a subdirectory @e plane exists, it will be used for loading and storing plane information.
     */
    CVLDWrapper(const std::string& datasetPath, const CameraManager& cameraParams, const LabelMap& labelMap, unsigned int maxLoadImages = 0);

    // TODO: better return values
    /**
     * @brief Precompute plane information and store it to disk.
     * @param labels Vector of labels for which planes shall be searched.
     * @param normal Plane normal to search for. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance in degrees in which a plane normal will be considered matching.
     * @return Status indicating success/failure/...
     *
     * @remarks This requires that a @e plane subdirectory exists in the data set path.
     */
    bool precomputePlaneInfo(const std::vector<std::string>& labels, const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);
    /**
     * @brief Precompute plane information and store it to disk.
     * @param label Label for which planes shall be searched.
     * @param normal Plane normal to search for. Pass all-zero vector to accept any normal.
     * @param tolerance Angular tolerance in degrees in which a plane normal will be considered matching.
     * @return Status indicating success/failure/...
     *
     * @remarks This requires that a @e plane subdirectory exists in the data set path.
     */
    bool precomputePlaneInfo(const std::string& label, const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    /**
     * @brief Generate a training image with a specific object occlusion.
     * @param occlusionMin Minimum occlusion fraction for which a training image will be considered optimal.
     * @param occlusionMax Maximum occlusion fraction for which a training image will be considered optimal.
     * @return Pair containing the generated training image and a status describing the result.
     *
     * This function overload has very weak constraints on the generated training image. There are no restrictions
     * on the selected scene (i.e. the plane normal/distance) or the final object pose. The only restriction is the occlusion
     * that the object should have.
     *
     * @remarks If no training image with the desired occlusion can be generated, the best result found until then will be returned
     * and the status will indicate this.
     *
     * @warning In case of an error, the training image will be empty, so one should always check the returned status.
     */
    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(double occlusionMin = 0.0, double occlusionMax = 1.0);

    /**
     * @brief Generate a training image with a specific object occlusion and an initial rotation and velocity.
     * @param rotation 3x3 rotation matrix describing the initial rotation the object should have when starting to drop.
     * @param throwingDirection The initial velocity the object should have when starting to drop.
     * @param occlusionMin Minimum occlusion fraction for which a training image will be considered optimal.
     * @param occlusionMax Maximum occlusion fraction for which a training image will be considered optimal.
     * @return Pair containing the generated training image and a status describing the result.
     *
     * This function overload has very weak constraints on the generated training image. There are no restrictions
     * on the selected scene (i.e. the plane normal/distance) or the final object pose. The only restriction is the occlusion
     * that the object should have.
     *
     * @remarks If no training image with the desired occlusion can be generated, the best result found until then will be returned
     * and the status will indicate this.
     *
     * @warning In case of an error, the training image will be empty, so one should always check the returned status.
     */
    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(const cv::Mat_<double>& rotation, const cv::Point3d& throwingDirection,
                                                                     double occlusionMin = 0.0, double occlusionMax = 1.0);

    /**
     * @brief Generate a training image with a specific object occlusion and constraints on the final object pose.
     * @param rotation 3x3 rotation matrix describing the final rotation the object should have.
     * @param rotationTolerance Angular tolerance in degrees for the planes that are allowed to be selected, in relation to
     * the up-vector of the final object rotation.
     * @param distanceMin Minimum distance in millimeters that a plane may have to be valid.
     * @param distanceMax Maximum distance in millimeters that a plane may have to be valid.
     * @param occlusionMin Minimum occlusion fraction for which a training image will be considered optimal.
     * @param occlusionMax Maximum occlusion fraction for which a training image will be considered optimal.
     * @return Pair containing the generated training image and a status describing the result.
     *
     * This function overload has very specific constraints on the generated training image and the final object pose.
     * The final object rotation as well as the distance to the camera can be constrained in addition to the occlusion.
     *
     * @remarks If no training image with the desired occlusion can be generated, the best result found until then will be returned
     * and the status will indicate this.
     *
     * @note The final object pose is implemented by setting the initial object pose to this value, and restricting
     * the object from turning in any direction. This has the side effect that it will bounce and slide a lot less than when
     * turning is not prevented. If you want to compensate for this, consider adjusting object/plane restitution/friction
     * using setObjectRestitution(float) etc.
     *
     * @warning In case of an error, the training image will be empty, so one should always check the returned status.
     */
    std::pair<TrainingImage, Simulator::DropStatus> getTrainingImage(const cv::Mat_<double>& rotation, double rotationTolerance,
                                                                     unsigned short distanceMin, unsigned short distanceMax,
                                                                     double occlusionMin = 0.0, double occlusionMax = 1.0);

    /// Get the labels allowed for finding a plane.
    std::vector<std::string> getLabelsToUse() const;
    /// Reference to the vector containing the labels allowed for finding a plane.
    std::vector<std::string>& labelsToUse();
    /// Reference to the vector containing the labels allowed for finding a plane.
    const std::vector<std::string>& labelsToUse() const;
    /// Set the labels allowed for finding a plane.
    void setLabelsToUse(const std::vector<std::string>& labelsToUse);

    /// Get if the simulator should attempt to find a plane if none of the precomputed planes match the simulation parameters.
    bool getComputePlaneIfNoFile() const;
    /// Set if the simulator should attempt to find a plane if none of the precomputed planes match the simulation parameters.
    void setComputePlaneIfNoFile(bool computePlaneIfNoFile);

    /// Get the currently active object ID. The referenced object will be used for the simulations.
    unsigned int getActiveObject() const;
    /// Set the currently active object ID. The referenced object will be used for the simulations.
    void setActiveObject(unsigned int objectID);

    /// Get the number of available objects. This is also the largest valid object ID.
    unsigned int getNumObjects();

    /// Get object scaling factor.
    float getObjectScale() const;
    /// Set object scaling factor.
    void setObjectScale(float objectScale);

    /**
     * @brief Get object mass.
     * @remarks With single-object simulations, this should have no effect.
     */
    float getObjectMass() const;
    /**
     * @brief Set object mass.
     * @remarks With single-object simulations, this should have no effect.
     */
    void setObjectMass(float objectMass);

    /// Get object restitution.
    float getObjectRestitution() const;
    /// Set object restitution.
    void setObjectRestitution(float objectRestitution);

    /// Get object friction.
    float getObjectFriction() const;
    /// Set object friction.
    void setObjectFriction(float objectFriction);

    /// Get plane restitution.
    float getPlaneRestitution() const;
    /// Set plane restitution.
    void setPlaneRestitution(float planeRestitution);

    /// Get plane friction.
    float getPlaneFriction() const;
    /// Set plane friction.
    void setPlaneFriction(float planeFriction);

    /// Get maximum number of allowed attempts per scene.
    unsigned int getMaxAttempts() const;
    /**
     * @brief Set maximum number of allowed attempts per scene.
     * @remarks This sets the max attempts <em>per scene</em>, so the total max attempts for a getTrainingImage() call will be
     * @e maxAttempts * @e numValidScenes (where @e numValidScenes is the number of scenes satisfying the plane constraints)
     */
    void setMaxAttempts(unsigned int maxAttempts);

    /**
     * @brief Get if a preview window should be displayed for each scene.
     *
     * When this is @c true, a preview window will be displayed showing the result of the best attempt for each scene.
     * The user can then choose whether to keep the image or discard it.
     */
    bool getShowPreviewWindow() const;
    /**
     * @brief Set if a preview window should be displayed for each scene.
     *
     * When this is @c true, a preview window will be displayed showing the result of the best attempt for each scene.
     * The user can then choose whether to keep the image or discard it.
     */
    void setShowPreviewWindow(bool showPreviewWindow);

    /**
     * @brief Get if the object dropping should be animated in the preview window.
     *
     * When this is @c true, the dropping simulation of the physics engine will be shown in real-time for each attempt.
     *
     * @remarks This has no effect if no preview window will be shown.
     *
     * @sa getShowPreviewWindow()
     * @sa setShowPreviewWindow()
     */
    bool getShowPhysicsAnimation() const;
    /**
     * @brief Set if the object dropping should be animated in the preview window.
     *
     * When this is @c true, the dropping simulation of the physics engine will be shown in real-time for each attempt.
     *
     * @remarks This has no effect if no preview window will be shown.
     *
     * @note Settings this to @c true will cause @e each attempt to be animated in real-time. When setting a large number of
     * maximum attempts, this can take very long. It is mostly useful for debugging purposes.
     *
     * @sa getShowPreviewWindow()
     * @sa setShowPreviewWindow()
     * @sa getMaxAttempts()
     * @sa setMaxAttempts()
     */
    void setShowPhysicsAnimation(bool showPhysicsAnimation);

    /**
     * @brief Get gravity vector.
     * @return The gravity vector (if specified), and if it will be calculated automatically.
     */
    Auto<cv::Vec3f> getGravity() const;
    /**
     * @brief Set gravity vector.
     * @param The gravity vector (if not automatic), and if it should be calculated automatically.
     */
    void setGravity(const Auto<cv::Vec3f>& gravity);

    /**
     * @brief Reference to the vector containing the available objects that can be dropped into scenes.
     * @warning When modifying this vector, make sure that the active object ID is still valid (or update it). Otherwise, getTrainingImage() will throw an exception.
     * @sa getActiveObject()
     * @sa setActiveObject()
     * @sa getNumObjects()
     */
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
