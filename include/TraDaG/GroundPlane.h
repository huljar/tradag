#ifndef GROUNDPLANE_H
#define GROUNDPLANE_H

#include <TraDaG/KDTree.h>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreVector3.h>

#include <string>
#include <vector>

namespace TraDaG {
    class GroundPlane;
}

class TraDaG::GroundPlane
{
public:
    GroundPlane();
    GroundPlane(const Ogre::Plane& plane, const std::vector<Ogre::Vector3>& vertices, const std::string& label, bool autoConstructKDTree = false);

    GroundPlane(const GroundPlane& other);
    GroundPlane& operator=(const GroundPlane& other);

    bool saveToFile(const std::string& filePath, bool overwrite = false) const;

    Ogre::Vector3 projectPointOntoPlane(const Ogre::Vector3& vector) const;

    std::vector<Ogre::Vector3> findKNearestNeighbors(const Ogre::Vector3& queryPoint, unsigned int k);

    bool isPlaneDefined() const;

    Ogre::Plane getOgrePlane() const;
    const Ogre::Plane& ogrePlane() const;
    void setOgrePlane(const Ogre::Plane& plane);

    std::vector<Ogre::Vector3> getVertices() const;
    const std::vector<Ogre::Vector3>& vertices();
    void setVertices(const std::vector<Ogre::Vector3>& vertices);

    std::string getLabel() const;
    void setLabel(const std::string& label);

    float getRestitution() const;
    void setRestitution(float restitution);

    float getFriction() const;
    void setFriction(float friction);

    // Create plane from file
    static GroundPlane readFromFile(const std::string& filePath);

    // Model construction and evaluation functions for RANSAC
    static Ogre::Plane createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points);
    static float pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane);

protected:
    Ogre::Plane mPlane;
    std::vector<Ogre::Vector3> mVertices;
    KDTree mKDTree;
    bool mKDTreeUpdated;

    std::string mLabel;

    float mRestitution;
    float mFriction;
};

#endif // GROUNDPLANE_H
