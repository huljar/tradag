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
    RGBDScene(Ogre::SceneManager* sceneManager, const cv::Mat& depthImage, const cv::Mat& rgbImage,
              const CameraManager& cameraParams, bool autoCreateMesh = true);
    virtual ~RGBDScene();

    // Move semantics (defining those automatically prevents generation of default copy constructor and copy assignment operator)
    RGBDScene(RGBDScene&& other);
    RGBDScene& operator=(RGBDScene&& other);

    void meshify();

    bool screenspaceCoords(const Ogre::Camera* camera, Ogre::Vector2& resultTopLeft, Ogre::Vector2& resultBottomRight) const;

    Ogre::ManualObject* getManualObject() const;

    cv::Mat getDepthImage() const;
    cv::Mat getRGBImage() const;

    CameraManager getCameraManager() const;
    CameraManager& cameraManager();

protected:
    Ogre::ManualObject* mSceneObject;
    Ogre::SceneManager* mSceneMgr;

    cv::Mat mDepthImage;
    cv::Mat mRGBImage;

    CameraManager mCameraManager;

    bool mMeshUpdated;

private:
    void createVertices();
    void createIndices();

    inline Ogre::uint32 pixelToIndex(int x, int y) const { return y * mRGBImage.cols + x; }
};

#endif // RGBDSCENE_H
