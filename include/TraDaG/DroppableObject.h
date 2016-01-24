#ifndef DROPPABLEOBJECT_H
#define DROPPABLEOBJECT_H

#include <TraDaG/util.h>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>

#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <utility>

namespace TraDaG {
    class DroppableObject;
}

class TraDaG::DroppableObject
{
public:
    typedef std::map<cv::Point, std::pair<cv::Vec3s, bool>, bool(*)(const cv::Point&, const cv::Point&)> PixelInfoMap;

    DroppableObject(const std::string& meshName, Ogre::SceneManager* sceneManager);
    virtual ~DroppableObject();

    Ogre::Entity* getOgreEntity() const;

    PixelInfoMap getFinalObjectCoords() const;
    void setFinalObjectCoords(const PixelInfoMap& objectCoords);

    std::pair<float, float> getDesiredOcclusion() const;
    void setDesiredOcclusion(const std::pair<float, float>& occlusion);
    void setDesiredOcclusion(float minOcclusion, float maxOcclusion);

    float getFinalOcclusion() const;
    void setFinalOcclusion(float finalOcclusion);

    std::pair<unsigned short, unsigned short> getDesiredDistance() const;
    void setDesiredDistance(const std::pair<unsigned short, unsigned short>& distance);
    void setDesiredDistance(unsigned short minDistance, unsigned short maxDistance);

    unsigned short getFinalDistance() const;
    void setFinalDistance(unsigned short finalDistance);

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
    bool checkOcclusionValid(float min, float max) const;
    bool checkDistanceValid(unsigned short min, unsigned short max) const;

    Ogre::SceneManager* mSceneMgr;
    Ogre::Entity* mEntity;

    PixelInfoMap mFinalObjectCoords;

    std::pair<float, float> mDesiredOcclusion;
    float mFinalOcclusion;

    std::pair<unsigned short, unsigned short> mDesiredDistance;
    unsigned short mFinalDistance;

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
