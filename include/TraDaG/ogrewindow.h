#ifndef OGREWINDOW_H
#define OGREWINDOW_H

#include <Ogre.h>

class OgreWindow
{
public:
    OgreWindow();
    virtual ~OgreWindow();

    virtual void initialize();
    virtual void createScene();

protected:
    Ogre::Root* mRoot;
    Ogre::RenderWindow* mWindow;
    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;

    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;
};

#endif // OGREWINDOW_H
