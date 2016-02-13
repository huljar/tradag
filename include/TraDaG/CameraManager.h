#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <TraDaG/util.h>

#include <opencv2/core/core.hpp>

namespace TraDaG {
    class CameraManager;
}

class TraDaG::CameraManager
{
public:
    enum class MapMode { MAPPED_RGB_TO_DEPTH, MAPPED_DEPTH_TO_RGB, UNMAPPED_RGB_TO_DEPTH, UNMAPPED_DEPTH_TO_RGB };

    CameraManager(const cv::Vec2f& depthPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& depthFocalLength = cv::Vec2f(500, 500),
                  const cv::Vec2f& rgbPrincipalPoint = cv::Vec2f(320, 240), const cv::Vec2f& rgbFocalLength = cv::Vec2f(500, 500),
                  const cv::Matx33f& rotation = cv::Matx33f::eye(), const cv::Vec3f translation = cv::Vec3f(0, 0, 0),
                  MapMode mapMode = MapMode::MAPPED_DEPTH_TO_RGB);

    cv::Vec3f getWorldForDepth(const cv::Point& uv, unsigned short d) const;
    cv::Vec3f getWorldForDepth(int u, int v, unsigned short d) const;

    cv::Vec2i getDepthForWorld(const cv::Vec3f& xyz) const;
    cv::Vec2i getDepthForWorld(float x, float y, float z) const;

    cv::Vec2i getRGBForWorld(const cv::Vec3f& xyz) const;
    cv::Vec2i getRGBForWorld(float x, float y, float z) const;

    cv::Vec2i getLabelForWorld(const cv::Vec3f& xyz) const;
    cv::Vec2i getLabelForWorld(float x, float y, float z) const;

    cv::Vec2i getRGBForDepth(const cv::Point& uv, unsigned short d) const;
    cv::Vec2i getRGBForDepth(int u, int v, unsigned short d) const;

    cv::Vec2i getLabelForDepth(const cv::Point& uv, unsigned short d) const;
    cv::Vec2i getLabelForDepth(int u, int v, unsigned short d) const;

    cv::Vec2f getDepthPrincipalPoint() const;
    cv::Vec2f getDepthFocalLength() const;

    cv::Vec2f getRGBPrincipalPoint() const;
    cv::Vec2f getRGBFocalLength() const;

    cv::Matx33f getRotation() const;
    cv::Vec3f getTranslation() const;

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
