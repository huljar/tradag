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
    virtual ~GroundPlane();

    virtual void leastSquaresFit();

    virtual bool isPlaneDefined() const;

    virtual Ogre::Plane getOgrePlane() const;
    virtual Ogre::Plane& ogrePlane();
    virtual void setOgrePlane(const Ogre::Plane& plane);

    virtual std::vector<Ogre::Vector3> getVertices() const;
    virtual std::vector<Ogre::Vector3>& vertices();
    virtual void setVertices(const std::vector<Ogre::Vector3>& vertices);

    virtual std::string getLabel() const;
    virtual void setLabel(const std::string& label);

    virtual float getRestitution() const;
    virtual void setRestitution(float restitution);

    virtual float getFriction() const;
    virtual void setFriction(float friction);

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
