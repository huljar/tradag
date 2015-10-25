#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

#include <TraDaG/rgbdobject.h>

class OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void initialize();
    virtual void createScene();
    virtual void enterRenderingLoop();

    virtual void resetCamera();

    virtual void setScene(RgbdObject* scene);

    // FrameListener method
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
    Ogre::Vector3 defaultCameraPosition;
    Ogre::Vector3 defaultCameraLookAt;

};

#endif // OGREWINDOW_H
