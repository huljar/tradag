#ifndef DROPPABLEOBJECT_H
#define DROPPABLEOBJECT_H

#include <TraDaG/util.h>

#include <Ogre.h>

#include <opencv2/core/core.hpp>

#include <string>

namespace TraDaG {
    class DroppableObject;
}

class TraDaG::DroppableObject
{
public:
    DroppableObject(const std::string& meshName, Ogre::SceneManager* sceneManager);
    virtual ~DroppableObject();

    virtual Ogre::Entity* getOgreEntity() const;

    virtual std::pair<float, float> getDesiredOcclusion() const;
    virtual void setDesiredOcclusion(const std::pair<float, float>& occlusion);
    virtual void setDesiredOcclusion(float minOcclusion, float maxOcclusion);

    virtual float getFinalOcclusion() const;
    virtual void setFinalOcclusion(float finalOcclusion);

    virtual bool getMustBeUpright() const;
    virtual void setMustBeUpright(bool mustBeUpright);

    virtual bool getCastShadows() const;
    virtual void setCastShadows(bool castShadows);

    virtual cv::Vec3f getScale() const;
    virtual void setScale(const cv::Vec3f& scale);
    virtual void setScale(float scaleX, float scaleY, float scaleZ);
    virtual void setScale(float scale);

    virtual Auto<cv::Vec3f> getInitialPosition() const;
    virtual void setInitialPosition(const Auto<cv::Vec3f>& initialPosition);

    virtual cv::Vec3f getFinalPosition() const;
    virtual void setFinalPosition(const Ogre::Vector3& finalPosition);
    virtual void setFinalPosition(const cv::Vec3f& finalPosition);
    virtual void setFinalPosition(float finalPositionX, float finalPositionY, float finalPositionZ);

    virtual Auto<cv::Matx33f> getInitialRotation() const;
    virtual void setInitialRotation(const Auto<cv::Matx33f>& initialRotation);

    virtual cv::Matx33f getFinalRotation() const;
    virtual void setFinalRotation(const Ogre::Matrix3& finalRotation);
    virtual void setFinalRotation(const cv::Matx33f& finalRotation);

    virtual float getInitialAzimuth() const;
    virtual void setInitialAzimuth(float initialAzimuth);

    virtual cv::Vec3f getInitialVelocity() const;
    virtual void setInitialVelocity(const Ogre::Vector3& initialVelocity);
    virtual void setInitialVelocity(const cv::Vec3f& initialVelocity);
    virtual void setInitialVelocity(float initialVelocityX, float initialVelocityY, float initialVelocityZ);

    virtual cv::Vec3f getInitialTorque() const;
    virtual void setInitialTorque(const Ogre::Vector3& initialTorque);
    virtual void setInitialTorque(const cv::Vec3f& initialTorque);
    virtual void setInitialTorque(float initialTorqueX, float initialTorqueY, float initialTorqueZ);

    virtual float getRestitution() const;
    virtual void setRestitution(float restitution);

    virtual float getFriction() const;
    virtual void setFriction(float friction);

    virtual float getMass() const;
    virtual void setMass(float mass);

protected:
    virtual bool checkIntervalValid(float min, float max) const;

    Ogre::SceneManager* mSceneMgr;
    Ogre::Entity* mEntity;

    std::pair<float, float> mDesiredOcclusion;
    float mFinalOcclusion;

    bool mMustBeUpright;
    bool mCastShadows;

    cv::Vec3f mScale;

    Auto<cv::Vec3f> mInitialPosition;
    cv::Vec3f mFinalPosition;

    Auto<cv::Matx33f> mInitialRotation;
    float mInitialAzimuth;
    cv::Matx33f mFinalRotation;

    cv::Vec3f mInitialVelocity;
    cv::Vec3f mInitialTorque;

    float mRestitution;
    float mFriction;
    float mMass;
};

#endif // DROPPABLEOBJECT_H
