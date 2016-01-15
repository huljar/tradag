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

    Ogre::Entity* getOgreEntity() const;

    std::pair<float, float> getDesiredOcclusion() const;
    void setDesiredOcclusion(const std::pair<float, float>& occlusion);
    void setDesiredOcclusion(float minOcclusion, float maxOcclusion);

    float getFinalOcclusion() const;
    void setFinalOcclusion(float finalOcclusion);

    bool getMustBeUpright() const;
    void setMustBeUpright(bool mustBeUpright);

    bool getCastShadows() const;
    void setCastShadows(bool castShadows);

    cv::Vec3f getScale() const;
    void setScale(const cv::Vec3f& scale);
    void setScale(float scaleX, float scaleY, float scaleZ);
    void setScale(float scale);

    Auto<cv::Vec3f> getInitialPosition() const;
    void setInitialPosition(const Auto<cv::Vec3f>& initialPosition);

    cv::Vec3f getFinalPosition() const;
    void setFinalPosition(const cv::Vec3f& finalPosition);
    void setFinalPosition(float finalPositionX, float finalPositionY, float finalPositionZ);

    Auto<cv::Matx33f> getInitialRotation() const;
    void setInitialRotation(const Auto<cv::Matx33f>& initialRotation);

    cv::Matx33f getFinalRotation() const;
    void setFinalRotation(const cv::Matx33f& finalRotation);

    float getInitialAzimuth() const;
    void setInitialAzimuth(float initialAzimuth);

    cv::Vec3f getInitialVelocity() const;
    void setInitialVelocity(const cv::Vec3f& initialVelocity);
    void setInitialVelocity(float initialVelocityX, float initialVelocityY, float initialVelocityZ);

    cv::Vec3f getInitialTorque() const;
    void setInitialTorque(const cv::Vec3f& initialTorque);
    void setInitialTorque(float initialTorqueX, float initialTorqueY, float initialTorqueZ);

    float getRestitution() const;
    void setRestitution(float restitution);

    float getFriction() const;
    void setFriction(float friction);

    float getMass() const;
    void setMass(float mass);

protected:
    bool checkIntervalValid(float min, float max) const;

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
