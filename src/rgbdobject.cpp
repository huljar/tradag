#include <TraDaG/rgbdobject.h>

#include <stdexcept>

using namespace TraDaG;

RgbdObject::RgbdObject(const Ogre::String& name, Ogre::SceneManager* sceneManager,
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

RgbdObject::~RgbdObject() {
    // If our object was attached to a scene node at some point, it has to be detached before being destroyed
    mSceneObject->detachFromParent();
    // Use the scene manager to do the cleanup
    mSceneMgr->destroyManualObject(mSceneObject);
}

void RgbdObject::meshify() {
    if(!mMeshUpdated) {
        mSceneObject->clear();

        mSceneObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        createVertices();
        createIndices();
        mSceneObject->end();

        mMeshUpdated = true;
    }
}

Ogre::Vector3 RgbdObject::depthToWorld(int x, int y, unsigned short depth) const {
    Ogre::Vector2 principalPoint = mMapMode == MAPPED_DEPTH_TO_RGB ? mRgbPrincipalPoint : mDepthPrincipalPoint;
    Ogre::Vector2 focalLength = mMapMode == MAPPED_DEPTH_TO_RGB ? mRgbFocalLength : mDepthFocalLength;

    Ogre::Real retX = ((Ogre::Real)x - principalPoint.x) * (Ogre::Real)depth / focalLength.x;
    // Ogre's y-vector points up, but OpenCV's y-vector points down (therefore negate result)
    Ogre::Real retY = -(((Ogre::Real)y - principalPoint.y) * (Ogre::Real)depth / focalLength.y);
    Ogre::Real retZ = -((Ogre::Real)depth);

    return Ogre::Vector3(retX, retY, retZ);
}

Ogre::Vector2 RgbdObject::worldToRgb(const Ogre::Vector3& point, const Ogre::Matrix3& rotation, const Ogre::Vector3& translation) const {
    Ogre::Vector3 transformed = rotation * point + translation;

    Ogre::Real retX = std::round(transformed.x * mRgbFocalLength.x / (-transformed.z) + mRgbPrincipalPoint.x);
    Ogre::Real retY = std::round((-transformed.y) * mRgbFocalLength.y / (-transformed.z) + mRgbPrincipalPoint.y);

    return Ogre::Vector2(
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.cols, retX)),
            std::max(0.0f, std::min((Ogre::Real)mRgbImage.rows, retY)));
}

void RgbdObject::createVertices() {
    for(int y = 0; y < mDepthImage.rows; ++y) {
        for(int x = 0; x < mDepthImage.cols; ++x) {
            // Transform depth pixel to world coordinates
            Ogre::Vector3 worldPoint = depthToWorld(x, y, mDepthImage.at<unsigned short>(y, x));
            mSceneObject->position(worldPoint);

            // Retrieve RGB pixel for this world point according to map mode
            cv::Vec3b rgbColor(255, 255, 255);
            if(mMapMode == MAPPED_RGB_TO_DEPTH || mMapMode == MAPPED_DEPTH_TO_RGB) {
                rgbColor = mRgbImage.at<cv::Vec3b>(y, x);
            }
            else if(mMapMode == UNMAPPED_RGB_TO_DEPTH) {
                Ogre::Vector2 rgbPixel = worldToRgb(worldPoint, mRotation, mTranslation);
                rgbColor = mRgbImage.at<cv::Vec3b>(rgbPixel.y, rgbPixel.x);
            }
            else if(mMapMode == UNMAPPED_DEPTH_TO_RGB) {
                Ogre::Vector2 rgbPixel = worldToRgb(worldPoint, mRotation.Transpose(), -mTranslation); // TODO: check if correct
                rgbColor = mRgbImage.at<cv::Vec3b>(rgbPixel.y, rgbPixel.x);
            }

            // Ogre uses RGB and OpenCV uses BGR, hence the reversed indexing
            mSceneObject->colour(((Ogre::Real)rgbColor[2]) / 255.0f, ((Ogre::Real)rgbColor[1]) / 255.0f, ((Ogre::Real)rgbColor[0]) / 255.0f);
        }
    }
}

void RgbdObject::createIndices() {
    for(int y = 0; y < mDepthImage.rows - 1; ++y) {
        for(int x = 0; x < mDepthImage.cols - 1; ++x) {
            // Create 2 triangles (= 1 "square") per iteration
            mSceneObject->index(pixelToIndex(x, y));
            mSceneObject->index(pixelToIndex(x, y + 1));
            mSceneObject->index(pixelToIndex(x + 1, y));

            mSceneObject->index(pixelToIndex(x + 1, y));
            mSceneObject->index(pixelToIndex(x, y + 1));
            mSceneObject->index(pixelToIndex(x + 1, y + 1));
        }
    }
}

Ogre::ManualObject* RgbdObject::getManualObject() {
    return mSceneObject;
}

cv::Mat RgbdObject::getDepthImage() {
    return mDepthImage;
}

const cv::Mat RgbdObject::getDepthImage() const {
    return mDepthImage;
}

cv::Mat RgbdObject::getRgbImage() {
    return mRgbImage;
}

const cv::Mat RgbdObject::getRgbImage() const {
    return mRgbImage;
}

Ogre::Vector2 RgbdObject::getDepthPrincipalPoint() const {
    return mDepthPrincipalPoint;
}

void RgbdObject::setDepthPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mDepthPrincipalPoint = principalPoint;
    mMeshUpdated = false;
}

void RgbdObject::setDepthPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mDepthPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
    mMeshUpdated = false;
}

Ogre::Vector2 RgbdObject::getDepthFocalLength() const {
    return mDepthFocalLength;
}

void RgbdObject::setDepthFocalLength(const Ogre::Vector2& focalLength) {
    mDepthFocalLength = focalLength;
    mMeshUpdated = false;
}

void RgbdObject::setDepthFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mDepthFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
    mMeshUpdated = false;
}

Ogre::Vector2 RgbdObject::getRgbPrincipalPoint() const {
    return mRgbPrincipalPoint;
}

void RgbdObject::setRgbPrincipalPoint(const Ogre::Vector2& principalPoint) {
    mRgbPrincipalPoint = principalPoint;
    mMeshUpdated = false;
}

void RgbdObject::setRgbPrincipalPoint(Ogre::Real principalPointX, Ogre::Real principalPointY) {
    mRgbPrincipalPoint = Ogre::Vector2(principalPointX, principalPointY);
    mMeshUpdated = false;
}

Ogre::Vector2 RgbdObject::getRgbFocalLength() const {
    return mRgbFocalLength;
}

void RgbdObject::setRgbFocalLength(const Ogre::Vector2& focalLength) {
    mRgbFocalLength = focalLength;
    mMeshUpdated = false;
}

void RgbdObject::setRgbFocalLength(Ogre::Real focalLengthX, Ogre::Real focalLengthY) {
    mRgbFocalLength = Ogre::Vector2(focalLengthX, focalLengthY);
    mMeshUpdated = false;
}

Ogre::Matrix3 RgbdObject::getRotation() const {
    return mRotation;
}

void RgbdObject::setRotation(const Ogre::Matrix3& rotation) {
    mRotation = rotation;
    mMeshUpdated = false;
}

Ogre::Vector3 RgbdObject::getTranslation() const {
    return mTranslation;
}

void RgbdObject::setTranslation(const Ogre::Vector3& translation) {
    mTranslation = translation;
    mMeshUpdated = false;
}

void RgbdObject::setTranslation(Ogre::Real translationX, Ogre::Real translationY, Ogre::Real translationZ) {
    mTranslation = Ogre::Vector3(translationX, translationY, translationZ);
    mMeshUpdated = false;
}

MapMode RgbdObject::getMapMode() const {
    return mMapMode;
}

void RgbdObject::setMapMode(MapMode mapMode) {
    mMapMode = mapMode;
    mMeshUpdated = false;
}
