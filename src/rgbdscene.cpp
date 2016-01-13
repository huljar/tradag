#include <TraDaG/rgbdscene.h>

#include <stdexcept>

using namespace TraDaG;

RGBDScene::RGBDScene(const Ogre::String& name, Ogre::SceneManager* sceneManager,
                     const cv::Mat& depthImage, const cv::Mat& rgbImage,
                     const Ogre::Vector2& depthPrincipalPoint, const Ogre::Vector2& depthFocalLength,
                     const Ogre::Vector2& rgbPrincipalPoint, const Ogre::Vector2& rgbFocalLength,
                     const Ogre::Matrix3& rotation, const Ogre::Vector3& translation,
                     MapMode mapMode, bool autoCreateMesh)
    : mDepthImage(depthImage)
    , mRgbImage(rgbImage)
    , mDepthPrincipalPoint(depthPrincipalPoint)
    , mDepthFocalLength(depthFocalLength)
    , mRgbPrincipalPoint(rgbPrincipalPoint)
    , mRgbFocalLength(rgbFocalLength)
    , mRotation(rotation)
    , mTranslation(translation)
    , mMapMode(mapMode)
    , mMeshUpdated(false)
{
    // Check if Depth and RGB image have the same dimensions
    if(!(mDepthImage.cols == mRgbImage.cols && mDepthImage.rows == mRgbImage.rows))
        throw std::runtime_error("Depth and RGB image do not have the same dimensions");

    // Check scene manager for null pointer
    if(!sceneManager)
        throw std::runtime_error("Received null pointer as scene manager");

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

Ogre::Vector3 RGBDScene::depthToWorld(Ogre::Real u, Ogre::Real v, Ogre::Real depth) const {
    Ogre::Vector2 principalPoint = mMapMode == MM_MAPPED_DEPTH_TO_RGB ? mRgbPrincipalPoint : mDepthPrincipalPoint;
    Ogre::Vector2 focalLength = mMapMode == MM_MAPPED_DEPTH_TO_RGB ? mRgbFocalLength : mDepthFocalLength;

    Ogre::Real retX = (u - principalPoint.x) * depth / focalLength.x;
    // Ogre's y-vector points up, but OpenCV's y-vector points down (therefore negate result)
    Ogre::Real retY = -((v - principalPoint.y) * depth / focalLength.y);
    Ogre::Real retZ = -depth;

    return Ogre::Vector3(retX, retY, retZ);
}

Ogre::Vector3 RGBDScene::depthToWorld(const Ogre::Vector3& uvdPoint) const {
    return depthToWorld(uvdPoint.x, uvdPoint.y, uvdPoint.z);
}

Ogre::Vector3 RGBDScene::worldToDepth(Ogre::Real x, Ogre::Real y, Ogre::Real z) const {
    Ogre::Vector2 principalPoint = mMapMode == MM_MAPPED_DEPTH_TO_RGB ? mRgbPrincipalPoint : mDepthPrincipalPoint;
    Ogre::Vector2 focalLength = mMapMode == MM_MAPPED_DEPTH_TO_RGB ? mRgbFocalLength : mDepthFocalLength;

    Ogre::Real retU = x * focalLength.x / (-z) + principalPoint.x;
    Ogre::Real retV = (-y) * focalLength.y / (-z) + principalPoint.y;
    Ogre::Real retD = -z;

    return Ogre::Vector3(retU, retV, retD);
}

Ogre::Vector3 RGBDScene::worldToDepth(const Ogre::Vector3& point) const {
    return worldToDepth(point.x, point.y, point.z);
}

Ogre::Vector2 RGBDScene::worldToRgb(const Ogre::Vector3& point, const Ogre::Matrix3& rotation, const Ogre::Vector3& translation) const {
    Ogre::Vector3 transformed = rotation * point + translation;

    Ogre::Real retU = std::round(transformed.x * mRgbFocalLength.x / (-transformed.z) + mRgbPrincipalPoint.x);
    Ogre::Real retV = std::round((-transformed.y) * mRgbFocalLength.y / (-transformed.z) + mRgbPrincipalPoint.y);

    return Ogre::Vector2(
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.cols, retU)),
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.rows, retV)));
}

bool RGBDScene::screenspaceCoords(const Ogre::Camera* camera, Ogre::Vector2& resultTopLeft, Ogre::Vector2& resultBottomRight) const {
   if(!mSceneObject->isInScene())
      return false;

   Ogre::Vector3 topLeft = depthToWorld(0, 0, Constants::WorkPlaneDepth);
   Ogre::Vector3 bottomRight = depthToWorld(mDepthImage.cols - 1, mDepthImage.rows - 1, Constants::WorkPlaneDepth);

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
            Ogre::Vector3 worldPoint = depthToWorld(u, v, mDepthImage.at<unsigned short>(v, u));
            mSceneObject->position(worldPoint);

            // Retrieve RGB pixel for this world point according to map mode
            cv::Vec3b rgbColor(255, 255, 255);
            if(mMapMode == MM_MAPPED_RGB_TO_DEPTH || mMapMode == MM_MAPPED_DEPTH_TO_RGB) {
                rgbColor = mRgbImage.at<cv::Vec3b>(v, u);
            }
            else if(mMapMode == MM_UNMAPPED_RGB_TO_DEPTH) {
                Ogre::Vector2 rgbPixel = worldToRgb(worldPoint, mRotation, mTranslation);
                rgbColor = mRgbImage.at<cv::Vec3b>(rgbPixel.y, rgbPixel.x);
            }
            else if(mMapMode == MM_UNMAPPED_DEPTH_TO_RGB) {
                Ogre::Vector2 rgbPixel = worldToRgb(worldPoint, mRotation.Transpose(), -mTranslation); // TODO: check if correct
                rgbColor = mRgbImage.at<cv::Vec3b>(rgbPixel.y, rgbPixel.x);
            }

            // Ogre uses RGB and OpenCV uses BGR, hence the reversed indexing
            mSceneObject->colour(((Ogre::Real)rgbColor[2]) / 255.0f, ((Ogre::Real)rgbColor[1]) / 255.0f, ((Ogre::Real)rgbColor[0]) / 255.0f);
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

Ogre::Vector2 RGBDScene::getDepthPrincipalPoint() const {
    return mDepthPrincipalPoint;
}

void RGBDScene::setDepthPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mDepthPrincipalPoint = principalPoint;
    mMeshUpdated = false;
}

void RGBDScene::setDepthPrincipalPoint(const cv::Vec2f& principalPoint) {
    mDepthPrincipalPoint = Ogre::Vector2(principalPoint[0], principalPoint[1]);
    mMeshUpdated = false;
}

void RGBDScene::setDepthPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mDepthPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
    mMeshUpdated = false;
}

Ogre::Vector2 RGBDScene::getDepthFocalLength() const {
    return mDepthFocalLength;
}

void RGBDScene::setDepthFocalLength(const Ogre::Vector2& focalLength) {
    mDepthFocalLength = focalLength;
    mMeshUpdated = false;
}

void RGBDScene::setDepthFocalLength(const cv::Vec2f& focalLength) {
    mDepthFocalLength = Ogre::Vector2(focalLength[0], focalLength[1]);
    mMeshUpdated = false;
}

void RGBDScene::setDepthFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mDepthFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
    mMeshUpdated = false;
}

Ogre::Vector2 RGBDScene::getRgbPrincipalPoint() const {
    return mRgbPrincipalPoint;
}

void RGBDScene::setRgbPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mRgbPrincipalPoint = principalPoint;
    mMeshUpdated = false;
}

void RGBDScene::setRgbPrincipalPoint(const cv::Vec2f& principalPoint) {
    mRgbPrincipalPoint = Ogre::Vector2(principalPoint[0], principalPoint[1]);
    mMeshUpdated = false;
}

void RGBDScene::setRgbPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mRgbPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
    mMeshUpdated = false;
}

Ogre::Vector2 RGBDScene::getRgbFocalLength() const {
    return mRgbFocalLength;
}

void RGBDScene::setRgbFocalLength(const Ogre::Vector2& focalLength) {
    mRgbFocalLength = focalLength;
    mMeshUpdated = false;
}

void RGBDScene::setRgbFocalLength(const cv::Vec2f& focalLength) {
    mRgbFocalLength = Ogre::Vector2(focalLength[0], focalLength[1]);
    mMeshUpdated = false;
}

void RGBDScene::setRgbFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mRgbFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
    mMeshUpdated = false;
}

Ogre::Matrix3 RGBDScene::getRotation() const {
    return mRotation;
}

void RGBDScene::setRotation(const Ogre::Matrix3& rotation) {
    mRotation = rotation;
    mMeshUpdated = false;
}

void RGBDScene::setRotation(const cv::Matx33f& rotation) {
    mRotation = Ogre::Matrix3(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                              rotation(1, 0), rotation(1, 1), rotation(1, 2),
                              rotation(2, 0), rotation(2, 1), rotation(2, 2));
    mMeshUpdated = false;
}

Ogre::Vector3 RGBDScene::getTranslation() const {
    return mTranslation;
}

void RGBDScene::setTranslation(const Ogre::Vector3& translation) {
    mTranslation = translation;
    mMeshUpdated = false;
}

void RGBDScene::setTranslation(const cv::Vec3f& translation) {
    mTranslation = Ogre::Vector3(translation[0], translation[1], translation[2]);
    mMeshUpdated = false;
}

void RGBDScene::setTranslation(Ogre::Real translationX, Ogre::Real translationY, Ogre::Real translationZ) {
    mTranslation = Ogre::Vector3(translationX, translationY, translationZ);
    mMeshUpdated = false;
}

MapMode RGBDScene::getMapMode() const {
    return mMapMode;
}

void RGBDScene::setMapMode(MapMode mapMode) {
    mMapMode = mapMode;
    mMeshUpdated = false;
}
