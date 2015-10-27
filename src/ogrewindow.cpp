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
    , mDefaultCameraPosition(0, 0, 0)
    , mDefaultCameraLookAt(0, 0, -1)
    , mGravity(0, /*-9.81*/-100, 0)
    , mBounds(Ogre::Vector3(-1000, -1000, -1000), Ogre::Vector3(1000, 1000, 1000))
    , mWorld(NULL)
    , mDebugDrawer(NULL)
{
}

OgreWindow::~OgreWindow() {
    for(auto it = mBodies.begin(); it != mBodies.end(); ++it)
        delete *it;
    for(auto it = mShapes.begin(); it != mShapes.end(); ++it)
        delete *it;

    delete mDebugDrawer;
    delete mWorld;

    delete mCameraMan;
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    delete mRoot;
}

void OgreWindow::initializeOgre() {
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

    rs->setConfigOption("Video Mode", "1024 x 768 @ 32-bit colour");
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
}

void OgreWindow::createCamera() {
    mCamera = mSceneMgr->createCamera("MainCamera");
    mCamera->setPosition(mDefaultCameraPosition);
    mCamera->lookAt(mDefaultCameraLookAt);
    mCamera->setNearClipDistance(2.0);
    mCamera->setFOVy(Ogre::Degree(55.0));

    // Create camera controller
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);
    mCameraMan->setTopSpeed(300);
}

void OgreWindow::initializeBullet() {
    if(mRoot == NULL) {
        std::cerr << "ERROR: You need to initialize OGRE before Bullet!" << std::endl;
        return;
    }

    // Set up Bullet world
    mWorld = new OgreBulletDynamics::DynamicsWorld(mSceneMgr, mBounds, mGravity);

    // Set up debug info display tool
    mDebugDrawer = new OgreBulletCollisions::DebugDrawer();
    mDebugDrawer->setDrawWireframe(true);
    mWorld->setDebugDrawer(mDebugDrawer);
    //mWorld->setShowDebugShapes(true);

    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(mDebugDrawer);

    // Create Bullet plane and box (change later to implement plane fitting and real objects)
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 1000);
    OgreBulletDynamics::RigidBody* planeBody = new OgreBulletDynamics::RigidBody("FloorPlaneBody", mWorld);
    OgreBulletCollisions::CollisionShape* planeShape = new OgreBulletCollisions::StaticPlaneCollisionShape(plane.normal, plane.d);

    planeBody->setStaticShape(planeShape, 0.1, 0.8);

    mBodies.push_back(planeBody);
    mShapes.push_back(planeShape);

    Ogre::Entity* boxEntity = mSceneMgr->createEntity("cube.mesh");
    boxEntity->setCastShadows(true);
    Ogre::AxisAlignedBox boxBoundingBox = boxEntity->getBoundingBox();
    Ogre::Vector3 boxSize = boxBoundingBox.getSize();
    boxSize *= 0.5 * 0.96;
    boxEntity->setMaterialName("BaseWhiteNoLighting");
    Ogre::SceneNode* boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    boxNode->attachObject(boxEntity);

    OgreBulletDynamics::RigidBody* boxBody = new OgreBulletDynamics::RigidBody("BoxBody", mWorld);
    OgreBulletCollisions::BoxCollisionShape* boxShape = new OgreBulletCollisions::BoxCollisionShape(boxSize);

    boxBody->setShape(boxNode, boxShape, 0.6f, 0.6f, 1.0f, Ogre::Vector3(-800, 400, -3600));
    boxBody->setLinearVelocity(0, 100, 0);

    mBodies.push_back(boxBody);
    mShapes.push_back(boxShape);
}

void OgreWindow::renderOneFrame() {
    mRoot->renderOneFrame();
}

void OgreWindow::enterRenderingLoop() {
    mRoot->startRendering();
}

void OgreWindow::resetCamera() {
    if(mCamera) {
        mCamera->setPosition(mDefaultCameraPosition);
        mCamera->lookAt(mDefaultCameraLookAt);
    }
}

void OgreWindow::setScene(RgbdObject* scene) {
    mScene = scene;
    if(mSceneSceneNode == NULL)
        mSceneSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    mSceneSceneNode->attachObject(mScene);
}

bool OgreWindow::frameStarted(const Ogre::FrameEvent& evt) {
    if(mWindow->isClosed())
        return false;

    mWorld->stepSimulation(evt.timeSinceLastFrame);

    return true;
}

bool OgreWindow::frameRenderingQueued(const Ogre::FrameEvent& evt) {
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

    // TODO: Update viewport aspect ratio
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
