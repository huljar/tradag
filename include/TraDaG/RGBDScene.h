#ifndef RGBDSCENE_H
#define RGBDSCENE_H

#include <TraDaG/CameraManager.h>
#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace TraDaG {
    class RGBDScene;
}

class TraDaG::RGBDScene
{
public:
    RGBDScene(const Ogre::String& name, Ogre::SceneManager* sceneManager,
              const cv::Mat& depthImage, const cv::Mat& rgbImage,
              const CameraManager& cameraParams, bool autoCreateMesh = true);
    virtual ~RGBDScene();

    void meshify();

    bool screenspaceCoords(const Ogre::Camera* camera, Ogre::Vector2& resultTopLeft, Ogre::Vector2& resultBottomRight) const;

    Ogre::ManualObject* getManualObject() const;

    cv::Mat getDepthImage() const;
    cv::Mat getRgbImage() const;

    CameraManager getCameraManager() const;
    CameraManager& cameraManager();

protected:
    Ogre::ManualObject* mSceneObject;
    bool mMeshUpdated;

    Ogre::SceneManager* mSceneMgr;

    cv::Mat mDepthImage;
    cv::Mat mRgbImage;

    CameraManager mCameraManager;

private:
    void createVertices();
    void createIndices();

    inline Ogre::uint32 pixelToIndex(int x, int y) const { return y * mRgbImage.cols + x; }
};

#endif // RGBDSCENE_H
