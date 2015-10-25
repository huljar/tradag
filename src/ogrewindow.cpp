#include <TraDaG/ogrewindow.h>

OgreWindow::OgreWindow()
    : mRoot(NULL)
    , mWindow(NULL)
    , mSceneMgr(NULL)
    , mCamera(NULL)
    , mCameraMan(NULL)
    , mResourcesCfg(Ogre::StringUtil::BLANK)
    , mPluginsCfg(Ogre::StringUtil::BLANK)
    , mInputManager(NULL)
    , mKeyboard(NULL)
    , mMouse(NULL)
    , mScene(NULL)
    , mSceneSceneNode(NULL)
    , defaultCameraPosition(0, 0, 0)
    , defaultCameraLookAt(0, 0, -1)
{
}

OgreWindow::~OgreWindow() {
    if(mCameraMan) delete mCameraMan;
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
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
#if defined(Q_OS_WIN)
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

    rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit colour");
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
    createCamera();

    // Add viewport
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

    // Set up the scene
    createScene();

    // Initialize OIS
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
    OIS::ParamList pl;
    size_t hWnd = 0;
    std::ostringstream hWndStr;

    mWindow->getCustomAttribute("WINDOW", &hWnd);
    hWndStr << hWnd;
    pl.insert(std::make_pair(std::string("WINDOW"), hWndStr.str()));

    // Enable non-exclusive input so we can still see the mouse and use mouse and keyboard outside of the application
#if defined(OIS_WIN32_PLATFORM)
    pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )));
    pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
    pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
    pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
#elif defined(OIS_LINUX_PLATFORM)
    pl.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
    pl.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));
    pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
    pl.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
#endif

    mInputManager = OIS::InputManager::createInputSystem(pl);
    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));

    windowResized(mWindow);

    // Add OIS callbacks
    mKeyboard->setEventCallback(this);
    mMouse->setEventCallback(this);

    // Add window event listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    // Add frame listener
    mRoot->addFrameListener(this);

    // Render an initial frame
    mRoot->renderOneFrame();
}

void OgreWindow::createCamera() {
    mCamera = mSceneMgr->createCamera("MainCamera");
    mCamera->setPosition(0, 0, 0);
    mCamera->lookAt(0, 0, -1);
    mCamera->setNearClipDistance(0.01);

    // Create camera controller
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);
}

void OgreWindow::createScene() {
//    Ogre::Entity* ogreEntity = mSceneMgr->createEntity("ogrehead.mesh");

//    Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
//    ogreNode->attachObject(ogreEntity);

//    mSceneMgr->setAmbientLight(Ogre::ColourValue(.5, .5, .5));

//    Ogre::Light* light = mSceneMgr->createLight("MainLight");
//    light->setPosition(20, 80, 50);
}


void OgreWindow::enterRenderingLoop() {
    mRoot->startRendering();
}

void OgreWindow::resetCamera() {
    if(mCamera) {
        mCamera->setPosition(defaultCameraPosition);
        mCamera->lookAt(defaultCameraLookAt);
    }
}

void OgreWindow::setScene(RgbdObject* scene) {
    mScene = scene;
    if(mSceneSceneNode == NULL)
        mSceneSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    mSceneSceneNode->attachObject(mScene);
}

bool OgreWindow::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    mKeyboard->capture();
    mMouse->capture();

    mCameraMan->frameRenderingQueued(evt);
    return true;
}

void OgreWindow::windowResized(Ogre::RenderWindow* rw) {
    // Update OIS mouse area
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState& ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;

    // Update viewport aspect ratio

}

void OgreWindow::windowClosed(Ogre::RenderWindow* rw) {
    if(rw == mWindow) {
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Shutting down OIS ***");
        if(mInputManager) {
            mInputManager->destroyInputObject(mMouse);
            mInputManager->destroyInputObject(mKeyboard);

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = NULL;
        }
    }
}

bool OgreWindow::keyPressed(const OIS::KeyEvent& e) {
    mCameraMan->injectKeyDown(e);
    if(e.key == OIS::KC_SPACE)
        resetCamera();
    return true;
}

bool OgreWindow::keyReleased(const OIS::KeyEvent& e) {
    mCameraMan->injectKeyUp(e);
    return true;
}

bool OgreWindow::mouseMoved(const OIS::MouseEvent& e) {
    mCameraMan->injectMouseMove(e);
    return true;
}

bool OgreWindow::mousePressed(const OIS::MouseEvent& e, const OIS::MouseButtonID button) {
    mCameraMan->injectMouseDown(e, button);
    return true;
}

bool OgreWindow::mouseReleased(const OIS::MouseEvent& e, const OIS::MouseButtonID button) {
    mCameraMan->injectMouseUp(e, button);
    return true;
}
