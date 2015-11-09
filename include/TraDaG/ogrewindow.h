#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <TraDaG/rgbdobject.h>

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

#include <OgreBullet/Dynamics/OgreBulletDynamicsRigidBody.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsStaticPlaneShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsBoxShape.h>

#include <vector>

namespace TraDaG {
    class OgreWindow;
}

class TraDaG::OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void renderOneFrame();
    virtual void startAnimation(const Ogre::String& meshName, const Ogre::Vector3& initialPosition, const Ogre::Matrix3& initialRotation,
                                const Ogre::Vector3& initialVelocity,
                                Ogre::Real objectRestitution, Ogre::Real objectFriction, Ogre::Real objectMass,
                                const Ogre::Plane& groundPlane, Ogre::Real planeRestitution, Ogre::Real planeFriction,
                                const Ogre::Vector3& gravity, bool castShadows);

    virtual void resetCamera();

    virtual void setScene(RgbdObject* scene);

    virtual bool hidden() const;
    virtual void show();
    virtual void hide();

    virtual Ogre::SceneManager* getSceneManager();

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
    virtual void createCamera();

    virtual bool getSceneIntersectionPoint(int mouseX, int mouseY, Ogre::Vector3& result);

    // OGRE
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    OgreBites::SdkCameraMan* mCameraMan;

    bool mHaltRendering;

    // OIS
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
    OgreBulletDynamics::DynamicsWorld* mWorld;
    Ogre::AxisAlignedBox mBounds;

    std::vector<OgreBulletDynamics::RigidBody*> mRigidBodies;
    std::vector<OgreBulletCollisions::CollisionShape*> mCollisionShapes;

    // Mouse click positions
    int mLastMouseDownPosX;
    int mLastMouseDownPosY;

    // Plane fitting
    std::queue<Ogre::Vector3> mPlaneVectors;

private:
    void initializeOgre();
    void shutDownOgre();

    void initializeOIS();
    void shutDownOIS();

    void initializeBullet(const Ogre::Vector3& gravity);
    void shutDownBullet();
};

#endif // OGREWINDOW_H
