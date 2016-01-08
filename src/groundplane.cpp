#include <TraDaG/groundplane.h>
#include <TraDaG/util.h>

using namespace TraDaG;

GroundPlane::GroundPlane()
    : mRestitution(Defaults::PlaneRestitution)
    , mFriction(Defaults::PlaneFriction)
{

}

GroundPlane::GroundPlane(const Ogre::Plane& plane, const std::vector<Ogre::Vector3>& vertices, const std::string& label)
    : mPlane(plane)
    , mVertices(vertices)
    , mLabel(label)
    , mRestitution(Defaults::PlaneRestitution)
    , mFriction(Defaults::PlaneFriction)
{
    if(!mPlane.normal.isZeroLength()) mPlane.normalise();
}

GroundPlane::~GroundPlane() {

}

void GroundPlane::leastSquaresFit() {
    if(mVertices.size() >= 3) {
        // TODO
    }
}

bool GroundPlane::isPlaneDefined() const {
    return !mPlane.normal.isZeroLength();
}

Ogre::Plane GroundPlane::getOgrePlane() const {
    return mPlane;
}

Ogre::Plane& GroundPlane::ogrePlane() {
    return mPlane;
}

void GroundPlane::setOgrePlane(const Ogre::Plane& plane) {
    mPlane = plane;
}

std::vector<Ogre::Vector3> GroundPlane::getVertices() const {
    return mVertices;
}

std::vector<Ogre::Vector3>& GroundPlane::vertices() {
    return mVertices;
}

void GroundPlane::setVertices(const std::vector<Ogre::Vector3>& vertices) {
    mVertices = vertices;
}

std::string GroundPlane::getLabel() const {
    return mLabel;
}

void GroundPlane::setLabel(const std::string& label) {
    mLabel = label;
}

float GroundPlane::getRestitution() const {
    return mRestitution;
}

void GroundPlane::setRestitution(float restitution) {
    mRestitution = restitution;
}

float GroundPlane::getFriction() const {
    return mFriction;
}

void GroundPlane::setFriction(float friction) {
    mFriction = friction;
}

Ogre::Plane GroundPlane::createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points) {
    Ogre::Plane ret(points[0], points[1], points[2]);
    ret.normalise(); // required so the getDistance member function returns the correct distance
    return ret;
}

float GroundPlane::pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane) {
    // Check for invalid plane first (this can occur, probably when the RANSAC sampled points
    // lie on a line and no unique plane can be created)
    if(plane.normal == Ogre::Vector3::ZERO)
        return 1.0; // treat every point as outlier so this model will be discarded

    // Ogre::Plane::getDistance returns positive/negative values depending on which side of the plane the point lies
    if(std::abs(plane.getDistance(point)) < Constants::RansacConfidenceInterval)
        return 0.0;

    return 1.0;
}
