#ifndef IMAGELABELING_H
#define IMAGELABELING_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/PlaneInfo.h>
#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

#include <limits>
#include <random>
#include <string>

namespace TraDaG {
    class ImageLabeling;
}

/**
 * @brief Class that manages the labeling of a single scene.
 *
 * This class contains the label image of a single scene and has the ability to find and fit planes
 * into the scene using RANSAC and region growing.
 *
 * @author Julian Harttung
 */
class TraDaG::ImageLabeling
{
public:
    /**
     * @brief Enumerator describing the result of a plane fit.
     */
    enum class PlaneFitStatus {
        SUCCESS, /**< A plane was successfully fitted. */
        INVALID_LABEL, /**< The specified label is invalid, i.e. the scene does not contain enough pixels with this label. */
        NO_GOOD_PLANE /**< The algorithm was unable to fit a plane that matches the given constraints. */
    };

    /// Map that contains the world points for image pixels and the ID assigned during region growing.
    typedef std::map<cv::Point, std::pair<Ogre::Vector3, int>, bool(*)(const cv::Point&, const cv::Point&)> PixelWorldMap;

    /**
     * @brief Constructor.
     * @param depthImage The depth image of the scene.
     * @param labelImage The label image of the scene.
     * @param labelMap The mapping of string labels to label image pixel values that will be used for fitting
     * (e.g. \ref TraDaG::Labels::NYUDepthV1 "NYUDepthV1" or \ref TraDaG::Labels::NYUDepthV2 "NYUDepthV2").
     * @param cameraParams The CameraManager instance to use.
     */
    ImageLabeling(const cv::Mat& depthImage, const cv::Mat& labelImage, const LabelMap& labelMap, const CameraManager& cameraParams);
    /**
     * @brief Destructor.
     */
    virtual ~ImageLabeling();

    /// Check if a specific label is contained in the scene.
    bool containsLabel(const std::string& label) const;

    /**
     * @brief Find all pixel values for the given label with enough pixels in the scene.
     * @param label The label to search for.
     * @return Vector with all label image pixel values (for the given label) that occur often enough in the scene.
     */
    LabelVec findValidLabelValues(const std::string& label) const;

    /**
     * @brief Try to fit a plane into the scene with the given constraints.
     * @param label Label for which the plane will be fitted.
     * @param result Output parameter to store the resulting plane in.
     * @param normal Constraint on the plane normal that the result must have. Specify (0, 0, 0) for no constraint.
     * @param tolerance Angular tolerance (in degrees) for the plane normal to be considered \"valid\" with the constraint.
     * This is irrelevant if no plane normal constraint is given.
     * @param minDistance Constraint on the minimum distance to the camera that all plane vertices must have.
     * @param maxDistance Constraint on the maximum distance to the camera that all plane vertices must have.
     * @param regionMode This specifies how the region should be selected that will be used for the final plane after
     * performing region growing.
     * @return Status indicating success or, in case of an error, which error occurred.
     *
     * This function will perform region growing on all the inlier vertices found and then select a region
     * according to @c regionMode. To fit a plane that preserves all plane vertices (which is probably better
     * if you want to store it in a file), use the other
     * \ref findPlaneForLabel(const std::string&, PlaneInfo&, const cv::Vec3f&, float) "overload".
     *
     * @remarks Region growing and picking is done to ensure that the plane has a continuous region of vertices
     * and no fragmented vertices or fragmented patches of vertices that are not connected to the rest.
     */
    virtual PlaneFitStatus findPlaneForLabel(const std::string& label, GroundPlane& result,
                                             const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0,
                                             unsigned short minDistance = 0,
                                             unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                             PlaneInfo::PickMode regionMode = PlaneInfo::PickMode::WEIGHTED_RANDOM);

    /**
     * @brief Try to fit a plane into the scene with the given constraints.
     * @param label Label for which the plane will be fitted.
     * @param result Output parameter to store the resulting plane information in.
     * @param normal Constraint on the plane normal that the result must have. Specify (0, 0, 0) for no constraint.
     * @param tolerance Angular tolerance (in degrees) for the plane normal to be considered \"valid\" with the constraint.
     * This is irrelevant if no plane normal constraint is given.
     * @return Status indicating success or, in case of an error, which error occurred.
     *
     * In contrast to the other
     * \ref findPlaneForLabel(const std::string&, GroundPlane&, const cv::Vec3f&, float, unsigned short, unsigned short, PlaneInfo::PickMode) "overload",
     * this function will keep all the vertices that were found as inliers by RANSAC. This is useful if you want to pick a region later using different
     * \ref PlaneInfo::PickMode "pick modes" and/or distance limitations. It is also more useful for storing plane information to files.
     *
     * @sa PlaneInfo::createGroundPlane
     * @sa PlaneInfo::saveToFile
     */
    virtual PlaneFitStatus findPlaneForLabel(const std::string& label, PlaneInfo& result,
                                             const cv::Vec3f& normal = cv::Vec3f(0, 0, 0), float tolerance = 15.0);

    /// Get the depth image of the scene.
    cv::Mat getDepthImage() const;
    /// Get the label image of the scene.
    cv::Mat getLabelImage() const;
    /// Get the label map.
    LabelMap getLabelMap() const;
    /// Get the camera manager.
    CameraManager getCameraManager() const;

protected:
    std::vector<unsigned int> doRegionGrowing(PixelWorldMap& inliers) const;

    cv::Mat mDepthImage;
    cv::Mat mLabelImage;
    LabelMap mLabelMap;
    CameraManager mCameraManager;

    std::default_random_engine mRandomEngine;
};

#endif // IMAGELABELING_H
