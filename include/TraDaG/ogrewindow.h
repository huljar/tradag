#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

#include <OgreBullet/Dynamics/OgreBulletDynamicsRigidBody.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsStaticPlaneShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsBoxShape.h>

#include <TraDaG/rgbdobject.h>

#include <vector>

class OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void initializeOgre();
    virtual void initializeBullet(const Ogre::Vector3& gravity);

    virtual void renderOneFrame();
    virtual void enterRenderingLoop();

    virtual void resetCamera();

    virtual void setScene(RgbdObject* scene);

    virtual Ogre::SceneManager* getSceneManager();

    virtual Ogre::Vector3 getGravity() const;

    // FrameListener methods
    virtual bool frameStarted(const Ogre::FrameEvent& evt);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // WindowEventListener methods
    virtual void windowResized(Ogre::RenderWindow* rw);
    virtual void windowClosed(Ogre::RenderWindow* rw);

    // KeyListener methods
    virtual bool keyPressed(const OIS::KeyEvent& e);
    virtual bool keyReleased(const OIS::KeyEvent& e);

    // MouseListener methods
    virtual bool mouseMoved(const OIS::MouseEvent& e);
    virtual bool mousePressed(const OIS::MouseEvent& e, const OIS::MouseButtonID button);
    virtual bool mouseReleased(const OIS::MouseEvent& e, const OIS::MouseButtonID button);

protected:
    virtual void createCamera();

    virtual Ogre::Vector3 getSceneIntersectionPoint(int mouseX, int mouseY);

    // OGRE pointers
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    OgreBites::SdkCameraMan* mCameraMan;

    // Paths of config files
    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;

    // OIS pointers
    OIS::InputManager* mInputManager;
    OIS::Keyboard* mKeyboard;
    OIS::Mouse* mMouse;

    // Scene
    RgbdObject* mScene;
    Ogre::SceneNode* mSceneSceneNode;

    // Camera
    Ogre::Vector3 mDefaultCameraPosition;
    Ogre::Vector3 mDefaultCameraLookAt;

    // Bullet physics
    Ogre::Vector3 mGravity;
    Ogre::AxisAlignedBox mBounds;
    OgreBulletDynamics::DynamicsWorld* mWorld;
    OgreBulletCollisions::DebugDrawer* mDebugDrawer;
    std::vector<OgreBulletDynamics::RigidBody*> mBodies;
    std::vector<OgreBulletCollisions::CollisionShape*> mShapes;

    // Mouse click positions
    int mLastMouseDownPosX;
    int mLastMouseDownPosY;

    // Plane fitting
    std::queue<Ogre::Vector3> mPlaneVectors;

};

#endif // OGREWINDOW_H
