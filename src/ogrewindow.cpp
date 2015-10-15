#include <TraDaG/ogrewindow.h>

OgreWindow::OgreWindow()
    : mRoot(NULL)
    , mResourcesCfg(Ogre::StringUtil::BLANK)
    , mPluginsCfg(Ogre::StringUtil::BLANK)
{
}

OgreWindow::~OgreWindow() {
    delete mRoot;
}

void OgreWindow::initialize() {
    // Resource and plugins config paths
    mResourcesCfg = "../config/resources.cfg";
    mPluginsCfg = "../config/plugins.cfg";

    // Create Ogre Root object
    mRoot = new Ogre::Root(mPluginsCfg);

    // Parse resources config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    Ogre::String name, locType;
    Ogre::ConfigFile::SectionIterator secIt = cf.getSectionIterator();

    while(secIt.hasMoreElements()) {
        Ogre::ConfigFile::SettingsMultiMap* settings = secIt.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator it;

        for(it = settings->begin(); it != settings->end(); ++it) {
            locType = it->first;
            name = it->second;

            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(name, locType);
        }
    }

    // Configure the render system to use
    const Ogre::RenderSystemList& rsList = mRoot->getAvailableRenderers();
    Ogre::RenderSystem* rs = NULL;

    Ogre::StringVector renderOrder;
#if defined(OS_WIN)
    renderOrder.push_back("Direct3D9");
    renderOrder.push_back("Direct3D11");
#endif
    renderOrder.push_back("OpenGL");
    renderOrder.push_back("OpenGL 3+");
    for (Ogre::StringVector::iterator iter = renderOrder.begin(); iter != renderOrder.end(); iter++)
    {
        for (Ogre::RenderSystemList::const_iterator it = rsList.begin(); it != rsList.end(); it++)
        {
            if ((*it)->getName().find(*iter) != Ogre::String::npos)
            {
                rs = *it;
                break;
            }
        }
        if (rs != NULL) break;
    }
    if (rs == NULL)
    {
        if (!mRoot->restoreConfig())
        {
            if (!mRoot->showConfigDialog())
                OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
                    "Abort render system configuration",
                    "OgreWindow::initialize");
        }
    }

    rs->setConfigOption("Video Mode", "1366 x 768 @ 32-bit colour");
    rs->setConfigOption("Full Screen", "No");
    mRoot->setRenderSystem(rs);

    // Init the render window
    mWindow = mRoot->initialise(true, "Training Data Generator");

    // Init resources
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    // Create scene manager
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

    // Create camera
    mCamera = mSceneMgr->createCamera("MainCamera");
    mCamera->setPosition(0, 0, 80);
    mCamera->lookAt(0, 0, -300);
    mCamera->setNearClipDistance(5);

    // Add viewport
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

    // Set up the scene
    createScene();

    // Enter rendering loop
    while(true) {
        Ogre::WindowEventUtilities::messagePump();
        if(mWindow->isClosed()) return;
        if(!mRoot->renderOneFrame()) return;
    }
}

void OgreWindow::createScene() {
    Ogre::Entity* ogreEntity = mSceneMgr->createEntity("ogrehead.mesh");

    Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode->attachObject(ogreEntity);

    mSceneMgr->setAmbientLight(Ogre::ColourValue(.5, .5, .5));

    Ogre::Light* light = mSceneMgr->createLight("MainLight");
    light->setPosition(20, 80, 50);
}
