#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <TraDaG/rgbdobject.h>
#include <TraDaG/util.h>

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

#include <OgreBullet/Dynamics/OgreBulletDynamicsRigidBody.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsStaticPlaneShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsConvexHullShape.h>

#include <vector>

namespace TraDaG {
    class OgreWindow;
}

class TraDaG::OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual SimulationResult startSimulation(
            const Ogre::String& meshName, const Ogre::Vector3& initialPosition, const Ogre::Matrix3& initialRotation, Ogre::Real scale,
            const Ogre::Vector3& linearVelocity, const Ogre::Vector3& angularVelocity, const Ogre::Vector3& angularFactor,
            Ogre::Real objectRestitution, Ogre::Real objectFriction, Ogre::Real objectMass,
            const Ogre::Plane& groundPlane, Ogre::Real planeRestitution, Ogre::Real planeFriction,
            const Ogre::Vector3& gravity, bool castShadows, bool drawBulletShapes, bool animate);

    virtual UserAction promptUserAction();

    virtual Ogre::Real queryCoveredFraction(Ogre::Real workPlaneDepth = 50.0) const;
    virtual bool queryObjectOnPlane() const;

    virtual bool render(cv::Mat& result, Ogre::Real workPlaneDepth = 50.0) const;

    virtual void resetCamera();

    virtual void setScene(RgbdObject* scene, bool updateCameraFOV = false, Ogre::Real workPlaneDepth = 50.0);

    virtual void markVertices(const std::vector<Ogre::Vector3>& vertices);
    virtual void unmarkVertices();

    virtual bool hidden() const;
    virtual void show();
    virtual void hide();

    virtual Ogre::SceneManager* getSceneManager();

    virtual Ogre::Entity* getObject();

    virtual Ogre::Vector3 getInitialCameraPosition() const;
    virtual Ogre::Vector3 getInitialCameraLookAt() const;

    // FrameListener methods
    virtual bool frameStarted(const Ogre::FrameEvent& evt);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // WindowEventListener methods
    virtual void windowResized(Ogre::RenderWindow* rw);
    virtual bool windowClosing(Ogre::RenderWindow* rw);

    // KeyListener methods
    virtual bool keyPressed(const OIS::KeyEvent& e);
    virtual bool keyReleased(const OIS::KeyEvent& e);

    // MouseListener methods
    virtual bool mouseMoved(const OIS::MouseEvent& e);
    virtual bool mousePressed(const OIS::MouseEvent& e, const OIS::MouseButtonID button);
    virtual bool mouseReleased(const OIS::MouseEvent& e, const OIS::MouseButtonID button);

protected:
    typedef enum {
        READY,
        SIMULATION_RUNNING,
        SIMULATION_FINISHED,
        SIMULATION_TIMEOUT,
        WINDOW_CLOSED,
        AWAITING_USER_INPUT
    } SimulationStatus;

    virtual bool stepSimulationWithIdleCheck(Ogre::Real timeElapsed);

    virtual bool getSceneIntersectionPoint(int mouseX, int mouseY, Ogre::Vector3& result);

    virtual OgreBulletCollisions::CollisionShape* createConvexHull(Ogre::Entity* object);

    // OGRE
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    OgreBites::SdkCameraMan* mCameraMan;

    Ogre::Entity* mObject;
    Ogre::SceneNode* mObjectNode;

    bool mHaltRendering;
    SimulationStatus mStatus;

    // OIS
    OIS::InputManager* mInputManager;
    OIS::Keyboard* mKeyboard;
    OIS::Mouse* mMouse;

    // Scene
    RgbdObject* mScene;
    Ogre::SceneNode* mSceneNode;
    Ogre::ManualObject* mVertexMarkings;

    // Camera
    Ogre::Vector3 mInitialCameraPosition;
    Ogre::Vector3 mInitialCameraLookAt;

    // Bullet physics
    OgreBulletDynamics::DynamicsWorld* mWorld;
    OgreBulletCollisions::DebugDrawer* mDebugDrawer;
    Ogre::AxisAlignedBox mBounds;

    std::vector<OgreBulletDynamics::RigidBody*> mRigidBodies;
    std::vector<OgreBulletCollisions::CollisionShape*> mCollisionShapes;

    Ogre::Real mIdleTime;
    Ogre::Real mTotalTime;

private:
    void initializeOgre();
    void shutDownOgre();

    void initializeOIS();
    void shutDownOIS();

    void initializeBullet(const Ogre::Vector3& gravity);
    void shutDownBullet();

    /**
     * This function is part of the OGRE 1.9 API documentation, but apparently not included in all packagings of OGRE 1.9
     */
    inline bool orientationEquals(const Ogre::Quaternion& lhs, const Ogre::Quaternion& rhs, Ogre::Real tolerance = 1e-3) const {
        Ogre::Real d = lhs.Dot(rhs);
        return 1 - d*d < tolerance;
    }
};

#endif // OGREWINDOW_H
