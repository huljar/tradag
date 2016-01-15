#ifndef GROUNDPLANE_H
#define GROUNDPLANE_H

#include <Ogre.h>

namespace TraDaG {
    class GroundPlane;
}

class TraDaG::GroundPlane
{
public:
    explicit GroundPlane();
    explicit GroundPlane(const Ogre::Plane& plane, const std::vector<Ogre::Vector3>& vertices, const std::string& label);

    virtual void leastSquaresFit();

    bool isPlaneDefined() const;

    Ogre::Plane getOgrePlane() const;
    Ogre::Plane& ogrePlane();
    void setOgrePlane(const Ogre::Plane& plane);

    std::vector<Ogre::Vector3> getVertices() const;
    std::vector<Ogre::Vector3>& vertices();
    void setVertices(const std::vector<Ogre::Vector3>& vertices);

    std::string getLabel() const;
    void setLabel(const std::string& label);

    float getRestitution() const;
    void setRestitution(float restitution);

    float getFriction() const;
    void setFriction(float friction);

    // Model construction and evaluation functions for RANSAC
    static Ogre::Plane createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points);
    static float pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane);

protected:
    Ogre::Plane mPlane;
    std::vector<Ogre::Vector3> mVertices;

    std::string mLabel;

    float mRestitution;
    float mFriction;
};

#endif // GROUNDPLANE_H
