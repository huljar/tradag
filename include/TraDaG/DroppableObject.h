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

/**
 * @brief Class representing a single object that can be dropped into RGB-D scenes.
 *
 * This class is used by Simulator for the objects that will be dropped into the scene. It contains
 * information about the object's mesh and has several parameters to specify object-specific
 * simulation parameters. In addition, the object-specific results of a simulation process can be
 * retrieved from here.
 *
 * @author Julian Harttung
 */
class TraDaG::DroppableObject
{
public:
    /// Map that contains the object coordinates and the visibility for object pixels.
    typedef std::map<cv::Point, std::pair<cv::Vec3s, bool>, bool(*)(const cv::Point&, const cv::Point&)> PixelInfoMap;

    /**
     * @brief Constructor.
     * @param meshName File name of the mesh that will be used for this object.
     * @param sceneManager Pointer to the active OGRE scene manager.
     *
     * @note Constructing an object this way will not register it with the Simulator instance. Use
     * Simulator::createObject instead.
     */
    DroppableObject(const std::string& meshName, Ogre::SceneManager* sceneManager);
    /**
     * @brief Destructor.
     *
     * @warning Destroying an object this way will will not unregister it with the Simulator instance (potentially
     * causing use-after-free errors). If the object was created through Simulator::createObject, use
     * Simulator::destroyObject instead.
     *
     * @sa Simulator::destroyAllObjects
     */
    virtual ~DroppableObject();

    /// Get the OGRE entity used internally.
    Ogre::Entity* getOgreEntity() const;

    /**
     * @brief Get the object coordinates that resulted from running a simulation.
     * @remarks Use this after running a simulation to retrieve the object coordinates for the resulting pose.
     *
     * @sa Simulator::execute
     */
    PixelInfoMap getFinalObjectCoords() const;
    /**
     * @brief Set the object coordinates that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalObjectCoords(const PixelInfoMap& objectCoords);

    /// Get the desired occlusion interval that is set for this object.
    std::pair<float, float> getDesiredOcclusion() const;
    /**
     * @brief Set the desired occlusion interval for this object.
     *
     * This occlusion interval will be used by the Simulator to calculate the score of a simulation attempt, and
     * to determine of a simulation attempt is optimal.
     */
    void setDesiredOcclusion(const std::pair<float, float>& occlusion);
    /**
     * @brief Set the desired occlusion interval for this object.
     *
     * This occlusion interval will be used by the Simulator to calculate the score of a simulation attempt, and
     * to determine of a simulation attempt is optimal.
     */
    void setDesiredOcclusion(float minOcclusion, float maxOcclusion);

    /**
     * @brief Get the occluded fraction of the object after running a simulation.
     * @remarks Use this after running a simulation to retrieve the occlusion for the simulation result.
     *
     * @sa Simulator::execute
     */
    float getFinalOcclusion() const;
    /**
     * @brief Set the occlusion that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalOcclusion(float finalOcclusion);

    /// Get the desired distance interval that is set for this object.
    std::pair<unsigned short, unsigned short> getDesiredDistance() const;
    /**
     * @brief Set the desired distance interval for this object.
     *
     * This distance interval will be used by the Simulator to calculate the score of a simulation attempt, and
     * to determine of a simulation attempt is optimal.
     */
    void setDesiredDistance(const std::pair<unsigned short, unsigned short>& distance);
    /**
     * @brief Set the desired distance interval for this object.
     *
     * This distance interval will be used by the Simulator to calculate the score of a simulation attempt, and
     * to determine of a simulation attempt is optimal.
     */
    void setDesiredDistance(unsigned short minDistance, unsigned short maxDistance);

    /**
     * @brief Get the distance of the object after running a simulation.
     * @remarks Use this after running a simulation to retrieve the distance for the simulation result.
     *
     * @sa Simulator::execute
     */
    unsigned short getFinalDistance() const;
    /**
     * @brief Set the distance that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalDistance(unsigned short finalDistance);

    /// Get if this object is allowed to spin/rotate during simulations.
    bool getMustBeUpright() const;
    /**
     * @brief Set if this object is allowed to spin/rotate during simulations.
     * @remarks Setting this to @c true will disable spinning/rotating completely, i.e. the initial object rotation
     * will always be the same as the final object rotation. In addition, enabling this will cause the object to
     * bounce and slide way less than otherwise (this is what the physics engine does).
     */
    void setMustBeUpright(bool mustBeUpright);

    /**
     * @brief Get if this object will cast shadows onto the scene.
     * @warning Shadows are currently not implemented, so this setting has no effect.
     */
    bool getCastShadows() const;
    /**
     * @brief Set if this object will cast shadows onto the scene.
     * @warning Shadows are currently not implemented, so this setting has no effect.
     */
    void setCastShadows(bool castShadows);

    /// Get scale of this object.
    cv::Vec3f getScale() const;
    /// Set scale of this object.
    void setScale(const cv::Vec3f& scale);
    /// Set scale of this object.
    void setScale(float scaleX, float scaleY, float scaleZ);
    /// Set scale of this object.
    void setScale(float scale);

    /**
     * @brief Get initial position of this object when appearing in the scene.
     * @remarks If this is set to automatic, the initial position will be a random point above the
     * ground plane (but with a fixed distance to the plane).
     */
    Auto<cv::Vec3f> getInitialPosition() const;
    /**
     * @brief Set initial position of this object when appearing in the scene.
     * @remarks If you set this to automatic, the initial position will be a random point above the
     * ground plane (but with a fixed distance to the plane).
     */
    void setInitialPosition(const Auto<cv::Vec3f>& initialPosition);

    /**
     * @brief Get the position of the object after running a simulation.
     * @remarks Use this after running a simulation to retrieve the position for the simulation result.
     *
     * @sa Simulator::execute
     */
    cv::Vec3f getFinalPosition() const;
    /**
     * @brief Set the position that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalPosition(const cv::Vec3f& finalPosition);
    /**
     * @brief Set the position that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalPosition(float finalPositionX, float finalPositionY, float finalPositionZ);

    /**
     * @brief Get initial rotation of this object when appearing in the scene.
     * @remarks If this is set to automatic, the initial rotation will be calculated such that the object's up-vector
     * is aligned with the ground plane normal.
     */
    Auto<cv::Matx33f> getInitialRotation() const;
    /**
     * @brief Set initial rotation of this object when appearing in the scene.
     * @remarks If you set this to automatic, the initial rotation will be calculated such that the object's up-vector
     * is aligned with the ground plane normal.
     */
    void setInitialRotation(const Auto<cv::Matx33f>& initialRotation);

    /**
     * @brief Get the rotation of the object after running a simulation.
     * @remarks Use this after running a simulation to retrieve the rotation for the simulation result.
     *
     * @sa Simulator::execute
     */
    cv::Matx33f getFinalRotation() const;
    /**
     * @brief Set the rotation that resulted from running a simulation.
     * @remarks This method is used internally by the Simulator to set the result of a simulation. You don't need
     * to use this function yourself.
     */
    void setFinalRotation(const cv::Matx33f& finalRotation);

    /// Get the azimuth angle (in degrees) that should be concatenated with the initial rotation.
    float getInitialAzimuth() const;
    /**
     * @brief Set the azimuth angle (in degrees) that should be concatenated with the initial rotation.
     * @note This value will be ignored if the rotation is not set to automatic. The purpose of this parameter is to make use
     * of the automatic alignment of the object's up-vector and the ground plane normal while still specifying a rotation around
     * this up-vector.
     *
     * @sa setInitialRotation
     */
    void setInitialAzimuth(float initialAzimuth);

    /// Get the initial velocity of the object when appearing in the scene.
    cv::Vec3f getInitialVelocity() const;
    /**
     * @brief Set the initial velocity of the object when appearing in the scene.
     * @remarks This only affects the velocity at the start of the simulation. After that, standard physics rules apply.
     */
    void setInitialVelocity(const cv::Vec3f& initialVelocity);
    /**
     * @brief Set the initial velocity of the object when appearing in the scene.
     * @remarks This only affects the velocity at the start of the simulation. After that, standard physics rules apply.
     */
    void setInitialVelocity(float initialVelocityX, float initialVelocityY, float initialVelocityZ);

    /// Get the initial torque of the object when appearing in the scene.
    cv::Vec3f getInitialTorque() const;
    /**
     * @brief Set the initial torque of the object when appearing in the scene.
     * @remarks This only affects the torque at the start of the simulation. After that, standard physics rules apply.
     */
    void setInitialTorque(const cv::Vec3f& initialTorque);
    /**
     * @brief Set the initial torque of the object when appearing in the scene.
     * @remarks This only affects the torque at the start of the simulation. After that, standard physics rules apply.
     */
    void setInitialTorque(float initialTorqueX, float initialTorqueY, float initialTorqueZ);

    /// Get the resitution of the object (physics engine parameter).
    float getRestitution() const;
    /// Set the resitution of the object (physics engine parameter).
    void setRestitution(float restitution);

    /// Get the friction of the object (physics engine parameter).
    float getFriction() const;
    /// Set the friction of the object (physics engine parameter).
    void setFriction(float friction);

    /// Get the mass of the object (physics engine parameter).
    float getMass() const;
    /// Set the mass of the object (physics engine parameter).
    void setMass(float mass);

    /// Get the influence of this object on the score of a simulation attempt.
    float getScoreWeight() const;
    /**
     * @brief Set the influence of this object on the score of a simulation.
     * @remarks Adjust this parameter if you have multiple objects registered with your Simulator, and you want a particular
     * object's result (like occlusion and distance) to have more influence on the score of the simulation attempt.
     */
    void setScoreWeight(float scoreWeight);

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

    float mScoreWeight;
};

#endif // DROPPABLEOBJECT_H
