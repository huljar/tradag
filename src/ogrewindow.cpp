#include <TraDaG/ogrewindow.h>
#include <TraDaG/util.h>

#include <OgreBites/OgreRay.h>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

using namespace TraDaG;

OgreWindow::OgreWindow()
    : mRoot(NULL)
    , mWindow(NULL)
    , mSceneMgr(NULL)
    , mCamera(NULL)
    , mCameraMan(NULL)
    , mObject(NULL)
    , mObjectNode(NULL)
    , mHaltRendering(false)
    , mStatus(READY)
    , mInputManager(NULL)
    , mKeyboard(NULL)
    , mMouse(NULL)
    , mScene(NULL)
    , mSceneNode(NULL)
    , mVertexMarkings(NULL)
    , mDefaultCameraPosition(0, 0, 0)
    , mDefaultCameraLookAt(0, 0, -1)
    , mWorld(NULL)
    , mDebugDrawer(NULL)
    , mBounds(Ogre::Vector3(-10000, -10000, -10000), Ogre::Vector3(10000, 10000, 10000))
    , mIdleTime(0)
    , mTotalTime(0)
    , mSelectedAction(KEEP)
{
    initializeOgre();
}

OgreWindow::~OgreWindow() {
    shutDownBullet();
    shutDownOIS();
    shutDownOgre();
}

void OgreWindow::initializeOgre() {
    // Create Ogre Root object
    mRoot = new Ogre::Root(Strings::PluginsCfgPath);

    // Parse resources config file
    Ogre::ConfigFile cf;
    cf.load(Strings::ResourcesCfgPath);

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
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
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
        if (rs) break;
    }
    if (!rs)
    {
        if (!mRoot->restoreConfig())
        {
            if (!mRoot->showConfigDialog())
                OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
                    "Abort render system configuration",
                    "OgreWindow::initialize");
        }
    }

    rs->setConfigOption("Video Mode", "1024 x 768");
    rs->setConfigOption("Full Screen", "No");
    rs->setConfigOption("VSync", "Yes");

    mRoot->setRenderSystem(rs);

    // Init the render system
    mRoot->initialise(false);

    // Create a render window (hidden by default)
    Ogre::NameValuePairList windowParams;
    windowParams["vsync"] = "true";
    windowParams["hidden"] = "true";

    mWindow = mRoot->createRenderWindow(Strings::RenderWindowName, 1024, 768, false, &windowParams);

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
    mCamera->setAutoAspectRatio(true);

    // Add window event listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    // Add frame listener
    mRoot->addFrameListener(this);
}

void OgreWindow::shutDownOgre() {
    delete mCameraMan;
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    delete mRoot;
}

void OgreWindow::initializeOIS() {
    if(!mInputManager) {
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

        // Trigger resize event to update OIS mouse area
        windowResized(mWindow);

        // Add OIS callbacks
        mKeyboard->setEventCallback(this);
        mMouse->setEventCallback(this);
    }
}

void OgreWindow::shutDownOIS() {
    if(mInputManager) {
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Shutting down OIS ***");

        mInputManager->destroyInputObject(mMouse);
        mInputManager->destroyInputObject(mKeyboard);

        OIS::InputManager::destroyInputSystem(mInputManager);
        mInputManager = NULL;
    }
}

void OgreWindow::initializeBullet(const Ogre::Vector3& gravity) {
    if(!mWorld) {
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing Bullet ***");

        // Set up Bullet world
        mWorld = new OgreBulletDynamics::DynamicsWorld(mSceneMgr, mBounds, gravity);

        // Set up debug drawer
        mDebugDrawer = new OgreBulletCollisions::DebugDrawer();
        mDebugDrawer->setDrawWireframe(true);
        mWorld->setDebugDrawer(mDebugDrawer);
    }
}

void OgreWindow::shutDownBullet() {
    for(auto it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it)
        delete *it;
    for(auto it = mCollisionShapes.begin(); it != mCollisionShapes.end(); ++it)
        delete *it;

    mRigidBodies.clear();
    mCollisionShapes.clear();

    if(mWorld) {
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Shutting down Bullet ***");

        delete mWorld;
        mWorld = NULL;

        delete mDebugDrawer;
        mDebugDrawer = NULL;
    }
}

void OgreWindow::createCamera() {
    mCamera = mSceneMgr->createCamera("MainCamera");
    mCamera->setPosition(mDefaultCameraPosition);
    mCamera->lookAt(mDefaultCameraLookAt);
    mCamera->setNearClipDistance(5.0);
    mCamera->setFOVy(Ogre::Degree(55.0));

    // Create camera controller
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);
    mCameraMan->setTopSpeed(300);
}

void OgreWindow::renderOneFrame() {
    mRoot->renderOneFrame();
}

void OgreWindow::startSimulation(const Ogre::String& meshName, const Ogre::Vector3& initialPosition, const Ogre::Matrix3& initialRotation, Ogre::Real scale,
                                 const Ogre::Vector3& linearVelocity, const Ogre::Vector3& angularVelocity, const Ogre::Vector3& angularFactor,
                                 Ogre::Real objectRestitution, Ogre::Real objectFriction, Ogre::Real objectMass,
                                 const Ogre::Plane& groundPlane, Ogre::Real planeRestitution, Ogre::Real planeFriction,
                                 const Ogre::Vector3& gravity, bool castShadows, bool drawBulletShapes, bool animate) {

    // Initialize Bullet physics
    initializeBullet(gravity);

    // Draw debug shapes if requested
    mWorld->setShowDebugShapes(drawBulletShapes);

    // Load object
    if(mObject) {
        // Destroy any previously created objects
        mObject->detachFromParent();
        mSceneMgr->destroyEntity(mObject);
    }

    // Create the new object
    mObject = mSceneMgr->createEntity(meshName);
    mObject->setCastShadows(castShadows); // TODO: when initializing Ogre, set stencil shadow type (and ambient light)

    // Create a scene node for the object if it doesn't already exist
    if(!mObjectNode) {
        mObjectNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    }
    mObjectNode->setScale(scale, scale, scale);

    // Attach new object to the scene node
    mObjectNode->attachObject(mObject);

    // Register object with bullet
    OgreBulletDynamics::RigidBody* objRigidBody = new OgreBulletDynamics::RigidBody(Strings::ObjRigidBodyName, mWorld);
    OgreBulletCollisions::CollisionShape* objCollisionShape = createConvexHull(mObject);

    objRigidBody->setShape(mObjectNode, objCollisionShape, objectRestitution, objectFriction, objectMass, initialPosition, Ogre::Quaternion(initialRotation));
    objRigidBody->setLinearVelocity(linearVelocity);
    objRigidBody->setAngularVelocity(angularVelocity);
    objRigidBody->getBulletRigidBody()->setAngularFactor(btVector3(angularFactor.x, angularFactor.y, angularFactor.z));

    // Register plane with Bullet
    OgreBulletDynamics::RigidBody* planeRigidBody = new OgreBulletDynamics::RigidBody(Strings::PlaneRigidBodyName, mWorld);
    OgreBulletCollisions::CollisionShape* planeCollisionShape = new OgreBulletCollisions::StaticPlaneCollisionShape(groundPlane.normal, -groundPlane.d);

    planeRigidBody->setStaticShape(planeCollisionShape, planeRestitution, planeFriction);

    // Store pointers so they can be cleaned up later
    mRigidBodies.push_back(objRigidBody);
    mRigidBodies.push_back(planeRigidBody);
    mCollisionShapes.push_back(objCollisionShape);
    mCollisionShapes.push_back(planeCollisionShape);

    // Reset timestep counts
    mIdleTime = 0;
    mTotalTime = 0;

    // Run simulation
    mStatus = SIMULATION_RUNNING;
    if(animate && !hidden()) {
        // Enter rendering loop; the simulation steps will be performed in the rendering callbacks
        mRoot->startRendering(); // this method does not return until rendering is stopped
    }
    else {
        // Don't use real time for simulation
        while(mStatus == SIMULATION_RUNNING) {
            bool idle = stepSimulationWithIdleCheck(1.0 / 60.0);

            if(idle) {
                mStatus = SIMULATION_FINISHED;
                std::cout << "Simulation finished, object is idle" << std::endl;
            }
            else if(mTotalTime >= Constants::TimeoutTimeThreshold) {
                mStatus = SIMULATION_TIMEOUT;
                std::cout << "Simulation timed out" << std::endl;
            }
        }

        // Shut down Bullet after the simulation
        shutDownBullet();

        if(!hidden()) {
            // Start rendering so the user can move the camera around
            mRoot->startRendering(); // this method does not return until rendering is stopped
            // TODO: show appropriate commands overlay, no, do this in promptUserAction
        }
    }

    // TODO: return user-selected action; if window is hidden, auto-select action and return
}

UserAction OgreWindow::promptUserAction() {
    mStatus = AWAITING_USER_INPUT;
    // TODO: show overlay, start rendering, wait for key input

    mStatus = READY;
    return KEEP;
}

float OgreWindow::queryCoveredFraction() const {

}

bool OgreWindow::queryObjectStillOnPlane() const {

}

void OgreWindow::resetCamera() {
    if(mCamera) {
        mCamera->setPosition(mDefaultCameraPosition);
        mCamera->lookAt(mDefaultCameraLookAt);
    }
}

void OgreWindow::setScene(RgbdObject* scene) {
    if(!mSceneNode)
        mSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    else if(mScene)
        mScene->getManualObject()->detachFromParent();

    mScene = scene;
    mSceneNode->attachObject(mScene->getManualObject());
}

void OgreWindow::markVertices(const std::vector<Ogre::Vector3>& vertices) {
    if(mSceneNode && mScene) {
        if(mVertexMarkings) {
            mVertexMarkings->detachFromParent();
            mSceneMgr->destroyManualObject(mVertexMarkings);
        }

        mVertexMarkings = mSceneMgr->createManualObject();

        mVertexMarkings->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
        for(std::vector<Ogre::Vector3>::const_iterator it = vertices.cbegin(); it != vertices.cend(); ++it) {
            mVertexMarkings->position(*it);
        }
        mVertexMarkings->end();

        mSceneNode->attachObject(mVertexMarkings);
    }
}

bool OgreWindow::frameStarted(const Ogre::FrameEvent& evt) {
    if(mHaltRendering || mWindow->isClosed()) {
        mHaltRendering = false;
        return false;
    }

    if(mStatus == SIMULATION_RUNNING) {
        bool idle = stepSimulationWithIdleCheck(evt.timeSinceLastFrame);

        if(idle) {
            mStatus = SIMULATION_FINISHED;
            shutDownBullet();

            // TODO: show overlay with available commands
            std::cout << "Simulation finished, object is idle" << std::endl;
        }
        else if(mTotalTime >= Constants::TimeoutTimeThreshold) {
            mStatus = SIMULATION_TIMEOUT;
            shutDownBullet();

            // TODO: show appropriate overlay
            std::cout << "Simulation timed out" << std::endl;
        }
    }

    return true;
}

bool OgreWindow::frameRenderingQueued(const Ogre::FrameEvent& evt) {
    if(mHaltRendering || mWindow->isClosed()) {
        mHaltRendering = false;
        return false;
    }

    if(mInputManager) {
        mKeyboard->capture();
        mMouse->capture();
    }

    mCameraMan->frameRenderingQueued(evt);
    return true;
}

void OgreWindow::windowResized(Ogre::RenderWindow* rw) {
    // Update OIS mouse area
    if(mInputManager) {
        unsigned int width, height, depth;
        int left, top;
        rw->getMetrics(width, height, depth, left, top);

        const OIS::MouseState& ms = mMouse->getMouseState();
        ms.width = width;
        ms.height = height;
    }
}

bool OgreWindow::windowClosing(Ogre::RenderWindow* rw) {
    if(rw == mWindow) {
        // Allow the window to be closed if it is hidden
        if(hidden())
            return true;

        // If the window is visible, only hide it instead of closing it
        hide();

        // Halt rendering loop
        mHaltRendering = true;

        // Adjust status
        mStatus = WINDOW_CLOSED;

        return false;
    }

    return true;
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

bool OgreWindow::stepSimulationWithIdleCheck(Ogre::Real timeElapsed) {
    // Get current object position/rotation
    Ogre::Quaternion oldOrient = mObjectNode->getOrientation();
    Ogre::Vector3 oldPos = mObjectNode->getPosition();

    // Step the simulation
    mWorld->stepSimulation(timeElapsed, 4);

    // Add elapsed time
    mTotalTime += timeElapsed;

    // Check if the object is still moving
    Ogre::Quaternion newOrient = mObjectNode->getOrientation();
    Ogre::Vector3 newPos = mObjectNode->getPosition();

    if(orientationEquals(oldOrient, newOrient) && oldPos.positionEquals(newPos)) {
        // The object is idle
        mIdleTime += timeElapsed;

        if(mIdleTime >= Constants::IdleTimeThreshold)
            return true;
    }
    else {
        // The object is moving, reset idle counter
        mIdleTime = 0;
    }

    return false;
}

bool OgreWindow::getSceneIntersectionPoint(int mouseX, int mouseY, Ogre::Vector3& result) {
    Ogre::Viewport* vp = mCamera->getViewport();
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(
                             (Ogre::Real)mouseX / (Ogre::Real)vp->getActualWidth(),
                             (Ogre::Real)mouseY / (Ogre::Real)vp->getActualHeight());

    OgreBites::OgreRay polygonRayQuery(mSceneMgr);
    return polygonRayQuery.RaycastFromPoint(mouseRay.getOrigin(), mouseRay.getDirection(), result);
}

OgreBulletCollisions::CollisionShape* OgreWindow::createConvexHull(Ogre::Entity* object) {
    size_t vertexCount;
    Ogre::Vector3* vertices;
    size_t indexCount;
    unsigned long* indices;

    OgreBites::OgreRay::GetMeshInformation(object, vertexCount, vertices, indexCount, indices,
                                           object->getParentNode()->_getDerivedPosition(),
                                           object->getParentNode()->_getDerivedOrientation(),
                                           object->getParentNode()->_getDerivedScale());

    btConvexHullShape* bulletShape = new btConvexHullShape((btScalar*)vertices, (int)vertexCount, sizeof(Ogre::Vector3));

    // Reduce number of vertices for performance
    // (see http://www.bulletphysics.org/mediawiki-1.5.8/index.php/BtShapeHull_vertex_reduction_utility)
    btShapeHull* hull = new btShapeHull(bulletShape);
    btScalar margin = bulletShape->getMargin();
    hull->buildHull(margin);
    btConvexHullShape* simplifiedBulletShape = new btConvexHullShape((const btScalar*)hull->getVertexPointer(), hull->numVertices());

    OgreBulletCollisions::CollisionShape* shape = new OgreBulletCollisions::ConvexHullCollisionShape(simplifiedBulletShape);

    // Clean up
    delete hull;
    delete bulletShape;
    delete[] indices;
    delete[] vertices;

    return shape;
}

bool OgreWindow::hidden() const {
    return mWindow->isHidden();
}

void OgreWindow::show() {
    mWindow->setHidden(false);
    Ogre::WindowEventUtilities::messagePump(); // necessary so OIS initializes correctly
    initializeOIS();
}

void OgreWindow::hide() {
    shutDownOIS();
    mWindow->setHidden(true);
}

Ogre::SceneManager* OgreWindow::getSceneManager() {
    return mSceneMgr;
}
