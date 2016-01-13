#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <TraDaG/RGBDScene.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/util.h>

#include <Ogre.h>
#include <OGRE/Overlay/OgreOverlaySystem.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

#include <OgreBullet/Dynamics/OgreBulletDynamicsWorld.h>
#include <OgreBullet/Dynamics/OgreBulletDynamicsRigidBody.h>
#include <OgreBullet/Collisions/Debug/OgreBulletCollisionsDebugDrawer.h>
#include <OgreBullet/Collisions/OgreBulletCollisionsShape.h>

#include <vector>

namespace TraDaG {
    class OgreWindow;
}

class TraDaG::OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual SimulationResult startSimulation(const ObjectVec& objects, const GroundPlane& plane,
                                             const Ogre::Vector3& gravity, bool drawBulletShapes, bool animate);

    virtual UserAction promptUserAction();

    virtual bool queryObjectInfo(const DroppableObject* object, Ogre::Real& occlusion, PixelInfoVec& pixelInfo, bool& onPlane);

    virtual bool render(cv::Mat& depthResult, cv::Mat& rgbResult, const DroppableObject* specificObject = NULL);

    virtual void resetCamera();

    virtual void setScene(RGBDScene* scene, bool updateCameraFOV = false);

    virtual void markVertices(const std::vector<Ogre::Vector3>& vertices = std::vector<Ogre::Vector3>());
    virtual void unmarkVertices(bool destroy = false);

    virtual bool hidden() const;
    virtual void show();
    virtual void hide();

    virtual Ogre::SceneManager* getSceneManager() const;
    virtual const std::vector<Ogre::Entity*>& objectEntities() const;
    virtual std::vector<Ogre::Entity*> getObjectEntities() const;

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

    virtual void loadSceneCollisionShapes(const std::vector<Ogre::Vector3>& excludeList = std::vector<Ogre::Vector3>());

    virtual OgreBulletCollisions::CollisionShape* createConvexHull(Ogre::Entity* object);

    virtual void setUpRenderSettings();
    virtual void invalidateRenderSettings();

    // OGRE
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mPreviewWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mPreviewCamera;
    OgreBites::SdkCameraMan* mPreviewCameraMan;

    Ogre::OverlaySystem* mOverlaySystem;

    // OIS
    OIS::InputManager* mInputManager;
    OIS::Keyboard* mKeyboard;
    OIS::Mouse* mMouse;

    // Objects
    std::vector<Ogre::Entity*> mObjects;

    // Scene
    RGBDScene* mRGBDScene;
    Ogre::SceneNode* mRGBDSceneNode;
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

    // Simulation
    Ogre::Real mIdleTime;
    Ogre::Real mTotalTime;

    bool mHaltRendering;
    SimulationStatus mStatus;

    // User action
    UserAction mActionChosen;

    // Rendering
    Ogre::RenderWindow* mRenderWindow;
    Ogre::Camera* mRenderCamera;

    Ogre::PixelBox* mRenderPixelBoxDepth;
    unsigned char* mRenderPixelBoxDepthData;
    Ogre::PixelBox* mRenderPixelBoxRGB;
    unsigned char* mRenderPixelBoxRGBData;

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
