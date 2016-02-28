/************************************************************//**
 * @file
 *
 * @brief CameraManager class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

namespace TraDaG {
    class CameraManager;
}

/**
 * @brief Utility class to hold all the camera parameters of depth and RGB camera in one place.
 *
 * @a %CameraManager holds all the camera parameters for depth and RGB camera, like principal points and
 * focal lengths, in one place. In addition, it provides conversions between pixels in the depth
 * image and corresponding world points, between world points and the corresponding RGB pixels,
 * and some more.
 */
class TraDaG::CameraManager
{
public:
    /**
     * @brief Enumerator describing the mapping between depth and RGB images.
     */
    enum class MapMode {
        MAPPED_RGB_TO_DEPTH, /**< The RGB images have been mapped onto the depth image. */
        MAPPED_DEPTH_TO_RGB, /**< The depth images have been mapped onto the RGB image. */
        UNMAPPED_RGB_TO_DEPTH, /**< Images are not mapped, and the provided rotation and translation are from RGB to depth camera. */
        UNMAPPED_DEPTH_TO_RGB /**< Images are not mapped, and the provided rotation and translation are from depth to RGB camera. */
    };

    /**
     * @brief Constructor taking all required camera parameters.
     * @param depthPrincipalPoint Principal point of the depth camera. Unused if @c mapMode is @c MAPPED_DEPTH_TO_RGB.
     * @param depthFocalLength Focal length of the depth camera. Unused if @c mapMode is @c MAPPED_DEPTH_TO_RGB.
     * @param rgbPrincipalPoint Principal point of the RGB camera. Unused if @c mapMode is @c MAPPED_RGB_TO_DEPTH.
     * @param rgbFocalLength Focal length of the RGB camera. Unused if @c mapMode is @c MAPPED_RGB_TO_DEPTH.
     * @param rotation Rotation matrix from one camera to the other, according to the specified @c mapMode. Unused if @c mapMode is <tt>MAPPED_...</tt> .
     * @param translation Translation vector from one camera to the other, according to the specified @c mapMode. Unused if @c mapMode is <tt>MAPPED_...</tt> .
     * @param mapMode Specifies the relation between depth and RGB images/cameras. See \ref CameraManager::MapMode "MapMode" for details.
     *
     * This constructor simply \"collects\" all information about the cameras/images. Depending on the @c mapMode, some information is not
     * required (e.g. if the depth images have been mapped onto the RGB images, rotation/translation between the cameras don't have to
     * be known, and for all \ref getWorldForDepth "depth pixel to world point" conversions, the parameters of the RGB camera will be
     * used).
     *
     * @remarks If you have all the information available and want to be sure, just specify everything even if it is not strictly
     * necessary. This ensures that the correct conversions will be performed.
     */
    CameraManager(const cv::Vec2f& depthPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& depthFocalLength = cv::Vec2f(500, 500),
                  const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
                  const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
                  MapMode mapMode = MapMode::MAPPED_DEPTH_TO_RGB);

    /**
     * @brief Get corresponding world point for a depth pixel.
     * @param uv Pixel position in the depth image.
     * @param d Depth at this pixel.
     * @return Corresponding world point.
     */
    cv::Vec3f getWorldForDepth(const cv::Point& uv, unsigned short d) const;
    /**
     * @brief Get corresponding world point for a depth pixel.
     * @param u Pixel position in the depth image (u-coordinate).
     * @param v Pixel position in the depth image (v-coordinate).
     * @param d Depth at this pixel.
     * @return Corresponding world point.
     */
    cv::Vec3f getWorldForDepth(int u, int v, unsigned short d) const;

    /**
     * @brief Get corresponding depth image pixel for a world point.
     * @param xyz The world point.
     * @return Corresponding depth pixel.
     */
    cv::Point getDepthForWorld(const cv::Vec3f& xyz) const;
    /**
     * @brief Get corresponding depth image pixel for a world point.
     * @param x The world point (x-coordinate).
     * @param y The world point (y-coordinate).
     * @param z The world point (z-coordinate).
     * @return Corresponding depth pixel.
     */
    cv::Point getDepthForWorld(float x, float y, float z) const;

    /**
     * @brief Get corresponding RGB image pixel for a world point.
     * @param xyz The world point.
     * @return Corresponding RGB pixel.
     */
    cv::Point getRGBForWorld(const cv::Vec3f& xyz) const;
    /**
     * @brief Get corresponding RGB image pixel for a world point.
     * @param x The world point (x-coordinate).
     * @param y The world point (y-coordinate).
     * @param z The world point (z-coordinate).
     * @return Corresponding RGB pixel.
     */
    cv::Point getRGBForWorld(float x, float y, float z) const;

    /**
     * @brief Get corresponding label image pixel for a world point.
     * @param xyz The world point.
     * @return Corresponding label pixel.
     */
    cv::Point getLabelForWorld(const cv::Vec3f& xyz) const;
    /**
     * @brief Get corresponding label image pixel for a world point.
     * @param x The world point (x-coordinate).
     * @param y The world point (y-coordinate).
     * @param z The world point (z-coordinate).
     * @return Corresponding label pixel.
     */
    cv::Point getLabelForWorld(float x, float y, float z) const;

    /**
     * @brief Get corresponding RGB image pixel for a depth image pixel.
     * @param uv Pixel position in the depth image.
     * @param d Depth at this pixel.
     * @return Corresponding RGB pixel.
     */
    cv::Point getRGBForDepth(const cv::Point& uv, unsigned short d) const;
    /**
     * @brief Get corresponding RGB image pixel for a depth image pixel.
     * @param u Pixel position in the depth image (u-coordinate).
     * @param v Pixel position in the depth image (v-coordinate).
     * @param d Depth at this pixel.
     * @return Corresponding RGB pixel.
     */
    cv::Point getRGBForDepth(int u, int v, unsigned short d) const;

    /**
     * @brief Get corresponding label image pixel for a depth image pixel.
     * @param uv Pixel position in the depth image.
     * @param d Depth at this pixel.
     * @return Corresponding label pixel.
     */
    cv::Point getLabelForDepth(const cv::Point& uv, unsigned short d) const;
    /**
     * @brief Get corresponding label image pixel for a depth image pixel.
     * @param u Pixel position in the depth image (u-coordinate).
     * @param v Pixel position in the depth image (v-coordinate).
     * @param d Depth at this pixel.
     * @return Corresponding label pixel.
     */
    cv::Point getLabelForDepth(int u, int v, unsigned short d) const;

    /// Get principal point of the depth camera.
    cv::Vec2f getDepthPrincipalPoint() const;
    /// Get focal length of the depth camera.
    cv::Vec2f getDepthFocalLength() const;

    /// Get principal point of the RGB camera.
    cv::Vec2f getRGBPrincipalPoint() const;
    /// Get focal length of the RGB camera.
    cv::Vec2f getRGBFocalLength() const;

    /// Get rotation between the depth/RGB camera.
    cv::Matx33f getRotation() const;
    /// Get translation between the depth/RGB camera.
    cv::Vec3f getTranslation() const;

    /// Get relation between depth and RGB images/cameras. See \ref CameraManager::MapMode "MapMode" for details.
    MapMode getMapMode() const;

protected:
    cv::Vec2f mDepthPrincipalPoint;
    cv::Vec2f mDepthFocalLength;

    cv::Vec2f mRGBPrincipalPoint;
    cv::Vec2f mRGBFocalLength;

    cv::Matx33f mRotation;
    cv::Vec3f mTranslation;

    MapMode mMapMode;
};

#endif // CAMERAMANAGER_H
