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
    , mHaltRendering(false)
    , mInputManager(NULL)
    , mKeyboard(NULL)
    , mMouse(NULL)
    , mScene(NULL)
    , mSceneNode(NULL)
    , mDefaultCameraPosition(0, 0, 0)
    , mDefaultCameraLookAt(0, 0, -1)
    , mWorld(NULL)
    , mBounds(Ogre::Vector3(-10000, -10000, -10000), Ogre::Vector3(10000, 10000, 10000))
    , mLastMouseDownPosX(0)
    , mLastMouseDownPosY(0)
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
    }
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

void OgreWindow::renderOneFrame() {
    mRoot->renderOneFrame();
}

void OgreWindow::startAnimation(const Ogre::String& meshName, const Ogre::Vector3& initialPosition, const Ogre::Matrix3& initialRotation, Ogre::Real scale,
                                const Ogre::Vector3& linearVelocity, const Ogre::Vector3& angularVelocity, const Ogre::Vector3& angularFactor,
                                Ogre::Real objectRestitution, Ogre::Real objectFriction, Ogre::Real objectMass,
                                const Ogre::Plane& groundPlane, Ogre::Real planeRestitution, Ogre::Real planeFriction,
                                const Ogre::Vector3& gravity, bool castShadows) {

    // Initialize Bullet physics
    initializeBullet(gravity);

    // Debug
    OgreBulletCollisions::DebugDrawer dd;
    dd.setDrawWireframe(true);
    mWorld->setDebugDrawer(&dd);
    mWorld->setShowDebugShapes(true);

    // Load object
    Ogre::Entity* object = mSceneMgr->createEntity(meshName);
    object->setCastShadows(castShadows); // TODO: when initializing Ogre, set stencil shadow type (and ambient light)

    // Register object with Ogre and Bullet
    Ogre::SceneNode* objNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    objNode->scale(scale, scale, scale);
    objNode->attachObject(object);

    OgreBulletDynamics::RigidBody* objRigidBody = new OgreBulletDynamics::RigidBody(Strings::ObjRigidBodyName, mWorld);
    OgreBulletCollisions::CollisionShape* objCollisionShape = createConvexHull(object);

    objRigidBody->setShape(objNode, objCollisionShape, objectRestitution, objectFriction, objectMass, initialPosition, Ogre::Quaternion(initialRotation));
    objRigidBody->setLinearVelocity(linearVelocity);
    objRigidBody->setAngularVelocity(angularVelocity);
    objRigidBody->getBulletRigidBody()->setAngularFactor(btVector3(angularFactor.x, angularFactor.y, angularFactor.z));

    // Register plane with Bullet
    OgreBulletDynamics::RigidBody* planeRigidBody = new OgreBulletDynamics::RigidBody(Strings::PlaneRigidBodyName, mWorld);
    OgreBulletCollisions::CollisionShape* planeCollisionShape = new OgreBulletCollisions::StaticPlaneCollisionShape(groundPlane.normal, groundPlane.d);

    planeRigidBody->setStaticShape(planeCollisionShape, planeRestitution, planeFriction);

    // Store pointers so they can be cleaned up later
    mRigidBodies.push_back(objRigidBody);
    mRigidBodies.push_back(planeRigidBody);
    mCollisionShapes.push_back(objCollisionShape);
    mCollisionShapes.push_back(planeCollisionShape);

    // Enter rendering loop
    mRoot->startRendering(); // this method does not return until rendering is stopped

    // This is executed after rendering was stopped
    shutDownBullet();
}

void OgreWindow::resetCamera() {
    if(mCamera) {
        mCamera->setPosition(mDefaultCameraPosition);
        mCamera->lookAt(mDefaultCameraLookAt);
    }
}

void OgreWindow::setScene(RgbdObject* scene) {
    mScene = scene;
    if(mSceneNode == NULL)
        mSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    mSceneNode->attachObject(mScene->getManualObject());
}

bool OgreWindow::frameStarted(const Ogre::FrameEvent& evt) {
    if(mHaltRendering || mWindow->isClosed()) {
        mHaltRendering = false;
        return false;
    }

    mWorld->stepSimulation(evt.timeSinceLastFrame, 4);

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

    // Record position to check if the mouse was moved when it is released
    mLastMouseDownPosX = e.state.X.abs;
    mLastMouseDownPosY = e.state.Y.abs;

    return true;
}

bool OgreWindow::mouseReleased(const OIS::MouseEvent& e, const OIS::MouseButtonID button) {
    mCameraMan->injectMouseUp(e, button);

    // If mouse was not moved since last mousePressed, handle the mouse click
    if(e.state.X.abs == mLastMouseDownPosX && e.state.Y.abs == mLastMouseDownPosY) {
        // Cast camera ray into the scene
        Ogre::Vector3 point;
        if(getSceneIntersectionPoint(e.state.X.abs, e.state.Y.abs, point)) {
            // Push the point into the queue that stores the plane support vectors
            mPlaneVectors.push(point);
            std::cout << "Point added! Coords: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

            // If the queue contains more than 3 vectors, remove the oldest one(s)
            while(mPlaneVectors.size() > 3)
                mPlaneVectors.pop();

            // Test: draw plane with 3 points
            if(mPlaneVectors.size() == 3) {
                Ogre::Vector3 p1 = mPlaneVectors.front(); mPlaneVectors.pop();
                Ogre::Vector3 p2 = mPlaneVectors.front(); mPlaneVectors.pop();
                Ogre::Vector3 p3 = mPlaneVectors.front(); mPlaneVectors.pop();
                Ogre::Plane testPlane(p1, p2, p3);

                // Adjust plane normal to face the camera if necessary
                std::cout << "Dot product: " << testPlane.normal.dotProduct(Ogre::Vector3::UNIT_Z) << std::endl;
                if(testPlane.normal.dotProduct(Ogre::Vector3::UNIT_Z) < 0) {
                    testPlane.normal = -testPlane.normal;
                    testPlane.d = -testPlane.d;
                }
                std::cout << "Plane normal: (" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ")" << std::endl
                          << "Plane distance: " << testPlane.d << std::endl;

                Ogre::MeshManager::getSingleton().createPlane("testPlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, testPlane, 500, 500);
                Ogre::Entity* planeEntity = mSceneMgr->createEntity("testPlane");
                planeEntity->setMaterialName("BaseWhiteNoLighting");
                mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(planeEntity);
            }
        }
    }

    return true;
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
    Ogre::WindowEventUtilities::messagePump(); // Necessary so OIS initializes correctly
    initializeOIS();
}

void OgreWindow::hide() {
    shutDownOIS();
    mWindow->setHidden(true);
}

Ogre::SceneManager* OgreWindow::getSceneManager() {
    return mSceneMgr;
}
