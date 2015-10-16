#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <Ogre.h>
#include <OgreBites/SdkCameraMan.h>

class OgreWindow : public Ogre::FrameListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void initialize();
    virtual void createScene();

    // FrameListener method
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // KeyListener methods
    virtual bool keyPressed(const OIS::KeyEvent& e);
    virtual bool keyReleased(const OIS::KeyEvent& e);

    // MouseListener methods
    virtual bool mouseMoved(const OIS::MouseEvent& e);
    virtual bool mousePressed(const OIS::MouseEvent& e);
    virtual bool mouseReleased(const OIS::MouseEvent& e);

protected:
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    OgreBites::SdkCameraMan* mCameraMan;

    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;
};

#endif // OGREWINDOW_H
