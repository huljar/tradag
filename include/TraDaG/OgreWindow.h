#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <TraDaG/DroppableObject.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/RGBDScene.h>
#include <TraDaG/util.h>

#include <OgreBites/SdkCameraMan.h>

#include <OGRE/OgreAxisAlignedBox.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreFrameListener.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgrePixelFormat.h>
#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreWindowEventUtilities.h>
#include <OGRE/Overlay/OgreOverlaySystem.h>

#include <OIS/OISInputManager.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>

#include <opencv2/core/core.hpp>

#include <OgreBullet/Collisions/OgreBulletCollisionsShape.h>
#include <OgreBullet/Collisions/Debug/OgreBulletCollisionsDebugDrawer.h>
#include <OgreBullet/Dynamics/OgreBulletDynamicsRigidBody.h>
#include <OgreBullet/Dynamics/OgreBulletDynamicsWorld.h>

#include <vector>

namespace TraDaG {
    class OgreWindow;
}

class TraDaG::OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    static OgreWindow& getSingleton();
    static OgreWindow* getSingletonPtr();

    virtual ~OgreWindow();

    // no copy, assign, move
    OgreWindow(const OgreWindow& other) = delete;
    OgreWindow& operator=(const OgreWindow& other) = delete;

    OgreWindow(OgreWindow&& other) = delete;
    OgreWindow& operator=(OgreWindow&& other) = delete;

    SimulationResult startSimulation(const ObjectVec& objects, RGBDScene* scene, const GroundPlane& plane,
                                     const Ogre::Vector3& gravity, bool drawBulletShapes, bool animate);

    UserAction promptUserAction();

    bool queryObjectInfo(const DroppableObject* object, GroundPlane& plane, float& occlusion, unsigned short& distance,
                         DroppableObject::PixelInfoMap& pixelInfo, bool& onPlane);

    bool render(cv::Mat& depthResult, cv::Mat& rgbResult, const DroppableObject* specificObject = nullptr);

    void invalidate(const ObjectVec& objects, const RGBDScene* scene);
    void invalidate(const DroppableObject* object);
    void invalidate(const RGBDScene* scene);

    void resetCamera();

    void markVertices(const std::vector<Ogre::Vector3>& vertices = std::vector<Ogre::Vector3>());
    void unmarkVertices(bool destroy = false);

    bool hidden() const;
    void show();
    void hide();

    Ogre::SceneManager* getSceneManager() const;

    Ogre::Vector3 getInitialCameraPosition() const;
    Ogre::Vector3 getInitialCameraLookAt() const;

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

    bool stepSimulationWithIdleCheck(Ogre::Real timeElapsed);

    bool getSceneIntersectionPoint(int mouseX, int mouseY, Ogre::Vector3& result);

    void loadSceneCollisionShapes(const std::vector<Ogre::Vector3>& excludeList = std::vector<Ogre::Vector3>());

    OgreBulletCollisions::CollisionShape* createConvexHull(Ogre::Entity* object);

    void setUpRenderSettings();
    void invalidateRenderSettings();

    // OGRE
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mPreviewWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mPreviewCamera;
    OgreBites::SdkCameraMan* mPreviewCameraMan;

    Ogre::LogManager* mLogManager;
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
    Ogre::SceneNode* mVertexMarkingsNode;

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
    static OgreWindow* msSingleton;

    OgreWindow();

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
