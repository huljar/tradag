#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>
#include <OIS.h>

class OgreWindow : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void initialize();
    virtual void createScene();

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
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    OgreBites::SdkCameraMan* mCameraMan;

    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;

    OIS::InputManager* mInputManager;
    OIS::Keyboard* mKeyboard;
    OIS::Mouse* mMouse;
};

#endif // OGREWINDOW_H
