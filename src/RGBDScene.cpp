#include <TraDaG/RGBDScene.h>
#include <TraDaG/interop.h>

#include <stdexcept>

using namespace TraDaG;

RGBDScene::RGBDScene(const Ogre::String& name, Ogre::SceneManager* sceneManager,
                     const cv::Mat& depthImage, const cv::Mat& rgbImage,
                     const CameraManager& cameraParams, bool autoCreateMesh)
    : mMeshUpdated(false)
    , mDepthImage(depthImage)
    , mRgbImage(rgbImage)
    , mCameraManager(cameraParams)
{
    // Check if Depth and RGB image have the same dimensions
    if(!(mDepthImage.cols == mRgbImage.cols && mDepthImage.rows == mRgbImage.rows))
        throw std::runtime_error("Depth and RGB image do not have the same dimensions");

    // Check scene manager for null pointer
    if(!sceneManager)
        throw std::invalid_argument("Received null pointer as scene manager");

    mSceneMgr = sceneManager;

    // Create the underlying ManualObject that will contain the mesh
    mSceneObject = sceneManager->createManualObject(name);

    // Create the mesh from the images if requested
    if(autoCreateMesh)
        meshify();
}

RGBDScene::~RGBDScene() {
    // If our object was attached to a scene node at some point, it has to be detached before being destroyed
    mSceneObject->detachFromParent();
    // Use the scene manager to do the cleanup
    mSceneMgr->destroyManualObject(mSceneObject);
}

void RGBDScene::meshify() {
    if(!mMeshUpdated) {
        mSceneObject->clear();

        mSceneObject->begin(Strings::StandardMaterialName, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        createVertices();
        createIndices();
        mSceneObject->end();

        mMeshUpdated = true;
    }
}

bool RGBDScene::screenspaceCoords(const Ogre::Camera* camera, Ogre::Vector2& resultTopLeft, Ogre::Vector2& resultBottomRight) const {
   if(!mSceneObject->isInScene())
      return false;

   Ogre::Vector3 topLeft = cvToOgre(mCameraManager.getWorldForDepth(0, 0, Constants::WorkPlaneDepth));
   Ogre::Vector3 bottomRight = cvToOgre(mCameraManager.getWorldForDepth(mDepthImage.cols - 1, mDepthImage.rows - 1, Constants::WorkPlaneDepth));

   Ogre::Plane cameraPlane = Ogre::Plane(camera->getDerivedOrientation().zAxis(), camera->getDerivedPosition());
   if(cameraPlane.getSide(topLeft) != Ogre::Plane::NEGATIVE_SIDE || cameraPlane.getSide(bottomRight) != Ogre::Plane::NEGATIVE_SIDE)
      return false;

   topLeft = camera->getProjectionMatrix() * camera->getViewMatrix() * topLeft;
   bottomRight = camera->getProjectionMatrix() * camera->getViewMatrix() * bottomRight;

   // Transform from coordinate space [-1, 1] to screen space [0, 1]
   resultTopLeft.x = topLeft.x / 2.0 + 0.5;
   resultTopLeft.y = 1 - (topLeft.y / 2.0 + 0.5);
   resultBottomRight.x = bottomRight.x / 2.0 + 0.5;
   resultBottomRight.y = 1 - (bottomRight.y / 2.0 + 0.5);

   return true;
}

void RGBDScene::createVertices() {
    for(int v = 0; v < mDepthImage.rows; ++v) {
        for(int u = 0; u < mDepthImage.cols; ++u) {
            // Transform depth pixel to world coordinates
            cv::Vec3f worldPoint = mCameraManager.getWorldForDepth(u, v, mDepthImage.at<unsigned short>(v, u));
            mSceneObject->position(cvToOgre(worldPoint));

            // Retrieve corresponding RGB pixel
            cv::Vec2i rgbPx = mCameraManager.getRGBForDepth(u, v, mDepthImage.at<unsigned short>(v, u));
            cv::Vec3b rgbColor = mRgbImage.at<cv::Vec3b>(rgbPx[1], rgbPx[0]);

            // Ogre uses RGB and OpenCV uses BGR, hence the reversed indexing
            mSceneObject->colour(static_cast<float>(rgbColor[2]) / 255.0f,
                                 static_cast<float>(rgbColor[1]) / 255.0f,
                                 static_cast<float>(rgbColor[0]) / 255.0f);
        }
    }
}

void RGBDScene::createIndices() {
    for(int v = 0; v < mDepthImage.rows - 1; ++v) {
        for(int u = 0; u < mDepthImage.cols - 1; ++u) {
            // Create 2 triangles (= 1 "square") per iteration
            mSceneObject->index(pixelToIndex(u, v));
            mSceneObject->index(pixelToIndex(u, v + 1));
            mSceneObject->index(pixelToIndex(u + 1, v));

            mSceneObject->index(pixelToIndex(u + 1, v));
            mSceneObject->index(pixelToIndex(u, v + 1));
            mSceneObject->index(pixelToIndex(u + 1, v + 1));
        }
    }
}

Ogre::ManualObject* RGBDScene::getManualObject() const {
    return mSceneObject;
}

cv::Mat RGBDScene::getDepthImage() const {
    return mDepthImage;
}

cv::Mat RGBDScene::getRgbImage() const {
    return mRgbImage;
}

CameraManager RGBDScene::getCameraManager() const {
    return mCameraManager;
}

CameraManager& RGBDScene::cameraManager() {
    return mCameraManager;
}
