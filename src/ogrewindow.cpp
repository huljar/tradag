#include <TraDaG/ogrewindow.h>
#include <TraDaG/util.h>

#include <OgreBites/OgreRay.h>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <limits>

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
    , mInitialCameraPosition(Constants::DefaultCameraPosition)
    , mInitialCameraLookAt(Constants::DefaultCameraLookAt)
    , mWorld(NULL)
    , mDebugDrawer(NULL)
    , mBounds(Ogre::Vector3(-10000, -10000, -10000), Ogre::Vector3(10000, 10000, 10000))
    , mIdleTime(0)
    , mTotalTime(0)
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

    mWindow = mRoot->createRenderWindow(Strings::PreviewWindowName, 1024, 768, false, &windowParams);

    // Init resources
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    // Create scene manager
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

    // Create camera
    mCamera = mSceneMgr->createCamera("MainCamera");
    mCamera->setPosition(mInitialCameraPosition);
    mCamera->lookAt(mInitialCameraLookAt);
    mCamera->setNearClipDistance(5.0);
    mCamera->setFOVy(Ogre::Degree(55.0));

    // Create camera controller
    mCameraMan = new OgreBites::SdkCameraMan(mCamera);
    mCameraMan->setTopSpeed(300);

    // Add viewport
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue::Black);

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

SimulationResult OgreWindow::startSimulation(
        const Ogre::String& meshName, const Ogre::Vector3& initialPosition,
        const Ogre::Matrix3& initialRotation, Ogre::Real scale,
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
    mObject = mSceneMgr->createEntity(Strings::ObjName, meshName);
    mObject->setCastShadows(castShadows); // TODO: when initializing Ogre, set stencil shadow type (and ambient light)

    // Create a new scene node
    if(mObjectNode) {
        mSceneMgr->destroySceneNode(mObjectNode);
    }

    mObjectNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
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
    SimulationResult ret = SR_SUCCESS;

    if(animate && !hidden()) {
        // Enter rendering loop; the simulation steps will be performed in the rendering callbacks
        mRoot->startRendering(); // this method does not return until rendering is stopped

        if(mStatus == SIMULATION_TIMEOUT) {
            ret = SR_TIMEOUT;
        }
        else if(mStatus == WINDOW_CLOSED) {
            ret = SR_ABORTED;
        }
    }
    else {
        // Don't use real time for simulation
        while(mStatus == SIMULATION_RUNNING) {
            bool idle = stepSimulationWithIdleCheck(1.0 / 60.0);

            if(idle) {
                mStatus = SIMULATION_FINISHED;
            }
            else if(mTotalTime >= Constants::TimeoutTimeThreshold) {
                mStatus = SIMULATION_TIMEOUT;
                ret = SR_TIMEOUT;
            }
        }
    }

    // Shut down Bullet after the simulation
    shutDownBullet();

    return ret;
}

UserAction OgreWindow::promptUserAction() {
    mStatus = AWAITING_USER_INPUT;
    // TODO: show overlay, start rendering, wait for key input
    // TODO: when window is hidden, somehow auto-select action?

    mStatus = READY;
    return UA_KEEP;
}

Ogre::Real OgreWindow::queryCoveredFraction(Ogre::Real workPlaneDepth) const {
    if(mObject) {
        // Find bounding box of object in image coordinates
        const Ogre::AxisAlignedBox& boundingBox3D = mObject->getWorldBoundingBox(true);

        std::pair<Ogre::Vector2, Ogre::Vector2> boundingBox2D(
            Ogre::Vector2(std::numeric_limits<Ogre::Real>::max(), std::numeric_limits<Ogre::Real>::max()),
            Ogre::Vector2(-std::numeric_limits<Ogre::Real>::max(), -std::numeric_limits<Ogre::Real>::max())
        );

        // Iterate over all corners of the 3D box
        for(const Ogre::Vector3* it = boundingBox3D.getAllCorners(); it < boundingBox3D.getAllCorners() + 8; ++it) {

            // Project the 3D point onto the 2D plane at a specific depth
            Ogre::Vector3 pointXYZ = (*it) * workPlaneDepth / std::abs(it->z);

            // Transform to (u, v, d)
            Ogre::Vector3 pointUVD = mScene->worldToDepth(pointXYZ);

            // Update bounding box
            if(pointUVD.x < boundingBox2D.first.x) boundingBox2D.first.x = pointUVD.x;
            if(pointUVD.y < boundingBox2D.first.y) boundingBox2D.first.y = pointUVD.y;

            if(pointUVD.x > boundingBox2D.second.x) boundingBox2D.second.x = pointUVD.x;
            if(pointUVD.y > boundingBox2D.second.y) boundingBox2D.second.y = pointUVD.y;
        }

        // Round bounding box
        boundingBox2D.first.x = std::floor(boundingBox2D.first.x);
        boundingBox2D.first.y = std::floor(boundingBox2D.first.y);
        boundingBox2D.second.x = std::ceil(boundingBox2D.second.x);
        boundingBox2D.second.y = std::ceil(boundingBox2D.second.y);

        // Debug start
//        std::cout << "Bounding box: (" << boundingBox2D.first.x << ", " << boundingBox2D.first.y
//                  << "), (" << boundingBox2D.second.x << ", " << boundingBox2D.second.y << ")" << std::endl;
//        Ogre::ManualObject* bbObj = mSceneMgr->createManualObject();
//        bbObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
//        bbObj->position(mScene->depthToWorld(boundingBox2D.first.x, boundingBox2D.first.y, workPlaneDepth));
//        bbObj->position(mScene->depthToWorld(boundingBox2D.second.x, boundingBox2D.first.y, workPlaneDepth));
//        bbObj->position(mScene->depthToWorld(boundingBox2D.second.x, boundingBox2D.second.y, workPlaneDepth));
//        bbObj->position(mScene->depthToWorld(boundingBox2D.first.x, boundingBox2D.second.y, workPlaneDepth));
//        bbObj->position(mScene->depthToWorld(boundingBox2D.first.x, boundingBox2D.first.y, workPlaneDepth));
//        bbObj->end();
//        mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(bbObj);
//        mRoot->renderOneFrame();
        // Debug end

        // Temporarily detach RGBD scene from scene manager
        mScene->getManualObject()->detachFromParent();

        // Cast rays through all pixels inside the bounding box
        std::vector<Ogre::Vector3> objectImgCoords;
        OgreBites::OgreRay polygonRayCaster(mSceneMgr);

        for(Ogre::Real v = boundingBox2D.first.y; v <= boundingBox2D.second.y; v += 1.0) {
            for(Ogre::Real u = boundingBox2D.first.x; u <= boundingBox2D.second.x; u += 1.0) {
                Ogre::Vector3 result;
                Ogre::MovableObject* object;

                if(polygonRayCaster.RaycastFromPoint(mInitialCameraPosition, mScene->depthToWorld(u, v, workPlaneDepth) - mInitialCameraPosition, result, object)
                        && object == mObject) {
                    objectImgCoords.push_back(mScene->worldToDepth(result));
                }
            }
        }

        // Reattach the RGBD scene
        mSceneNode->attachObject(mScene->getManualObject());

        // Test for all object image coordinates if the scene depth is smaller than the object depth
        cv::Mat depthImg = mScene->getDepthImage();
        size_t coveredPixels = 0;

        for(std::vector<Ogre::Vector3>::iterator it = objectImgCoords.begin(); it != objectImgCoords.end(); ++it) {
            int u = (int)std::round(it->x);
            int v = (int)std::round(it->y);
            if(it->z >= (Ogre::Real)depthImg.at<unsigned short>(v, u)) {
                ++coveredPixels;
            }
        }

        // Compute and return fraction
        return (Ogre::Real)coveredPixels / (Ogre::Real)objectImgCoords.size();
    }

    return -1.0;
}

bool OgreWindow::queryObjectOnPlane() const {

}

bool OgreWindow::renderToImage(cv::Mat& result, Ogre::Real workPlaneDepth) const {
    // Determine aspect ratio
    const cv::Mat depthImg = mScene->getDepthImage();
    Ogre::Vector3 topLeft = mScene->depthToWorld(0.0, 0.0, workPlaneDepth); // Explicit type statement, otherwise call is ambiguous
    Ogre::Vector3 topRight = mScene->depthToWorld((Ogre::Real)(depthImg.cols - 1), 0.0, workPlaneDepth);
    Ogre::Vector3 bottomLeft = mScene->depthToWorld(0.0, (Ogre::Real)(depthImg.rows - 1), workPlaneDepth);

    Ogre::Real width = topRight.x - topLeft.x;
    Ogre::Real height = topLeft.y - bottomLeft.y;
    if(height == 0.0)
        return false;

    // Determine vertical field of view (horizontal fov is automatically adjusted according to aspect ratio)
    Ogre::Vector3 top(0, topLeft.y, topLeft.z);
    Ogre::Vector3 bottom(0, bottomLeft.y, bottomLeft.z);
    Ogre::Radian verticalFOV(std::max(
        top.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians(),
        bottom.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians()
    ) * 2.0);

    // Create a hidden render window
    Ogre::NameValuePairList windowParams;
    windowParams["vsync"] = "true";
    windowParams["hidden"] = "true";

    Ogre::uint32 imgWidth = depthImg.cols * 1.2; // render in larger resolution because we crop black edges later
    Ogre::uint32 imgHeight = depthImg.rows * 1.2;

    Ogre::RenderWindow* renderWin = mRoot->createRenderWindow(Strings::RenderWindowName, imgWidth, imgHeight, false, &windowParams);

    // Create camera to use for rendering
    Ogre::Camera* renderCam = mSceneMgr->createCamera("RenderCamera");

    // Set camera parameters
    renderCam->setPosition(mInitialCameraPosition);
    renderCam->lookAt(mInitialCameraLookAt);
    renderCam->setNearClipDistance(5.0);
    renderCam->setFOVy(verticalFOV);
    renderCam->setAspectRatio(width / height);

    // Add viewport
    Ogre::Viewport* renderViewport = renderWin->addViewport(renderCam);
    renderViewport->setBackgroundColour(Ogre::ColourValue::Black);
    renderViewport->setOverlaysEnabled(false);

    // Render current frame
    mRoot->renderOneFrame(); // ensure that we are up to date
    renderWin->update(); // necessary, otherwise the captured screenshot may contain junk data

    // Allocate storage space
    Ogre::PixelFormat renderFormat = renderWin->suggestPixelFormat();
    size_t bytesPerPixel = Ogre::PixelUtil::getNumElemBytes(renderFormat);
    unsigned char* pixelData = new unsigned char[imgWidth * imgHeight * bytesPerPixel];

    // Get window contents
    Ogre::PixelBox renderPixelBox(imgWidth, imgHeight, 1, renderFormat, pixelData);
    renderWin->copyContentsToMemory(renderPixelBox);

    // Convert and copy to cv::Mat
    cv::Mat renderImage(renderPixelBox.getHeight(), renderPixelBox.getWidth(), CV_8UC3);
    for(int y = 0; y < renderImage.rows; ++y) {
        for(int x = 0; x < renderImage.cols; ++x) {
            Ogre::ColourValue value = renderPixelBox.getColourAt(x, y, 0);
            // OpenCV uses BGR color channel ordering and (row, col) pixel addresses
            renderImage.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(value.b * 255.0),
                static_cast<unsigned char>(value.g * 255.0),
                static_cast<unsigned char>(value.r * 255.0)
            );
        }
    }

    // Crop and resize image
    // Cropping is necessary because the image has black borders if the principal point of the camera is not the middle of the image
    cv::Mat renderImageGray;
    cv::cvtColor(renderImage, renderImageGray, CV_BGR2GRAY);

    // Gather all non-black points
    std::vector<cv::Point> points;
    for (cv::Mat_<unsigned char>::iterator it = renderImageGray.begin<unsigned char>(); it != renderImageGray.end<unsigned char>(); ++it) {
        if(*it) points.push_back(it.pos());
    }

    // Compute bounding rectangle of points
    cv::Rect roi = cv::boundingRect(points);

    // Resize the remaining area and copy to output parameter
    cv::resize(renderImage(roi), result, cv::Size(depthImg.cols, depthImg.rows));

    // Clean up
    delete[] pixelData;
    renderWin->removeAllViewports();
    mSceneMgr->destroyCamera(renderCam);
    mRoot->destroyRenderTarget(renderWin);

    return true;
}

void OgreWindow::resetCamera() {
    if(mCamera) {
        mCamera->setPosition(mInitialCameraPosition);
        mCamera->lookAt(mInitialCameraLookAt);
    }
}

void OgreWindow::setScene(RgbdObject* scene, bool updateCameraFOV, Ogre::Real workPlaneDepth) {
    if(!mSceneNode)
        mSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    else if(mScene)
        mScene->getManualObject()->detachFromParent();

    mScene = scene;
    mSceneNode->attachObject(mScene->getManualObject());

    if(updateCameraFOV) {
        const cv::Mat depthImg = mScene->getDepthImage();
        Ogre::Vector3 topLeft = mScene->depthToWorld(0.0, 0.0, workPlaneDepth);
        Ogre::Vector3 bottomLeft = mScene->depthToWorld(0.0, (Ogre::Real)(depthImg.rows - 1), workPlaneDepth);

        Ogre::Vector3 top(0, topLeft.y, topLeft.z);
        Ogre::Vector3 bottom(0, bottomLeft.y, bottomLeft.z);
        Ogre::Radian angle(std::max(
            top.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians(),
            bottom.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians()
        ) * 2.0 + 0.04);
        mCamera->setFOVy(angle);
    }
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

void OgreWindow::unmarkVertices() {
    if(mVertexMarkings) {
        mVertexMarkings->detachFromParent();
        mSceneMgr->destroyManualObject(mVertexMarkings);
        mVertexMarkings = NULL;
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
            return false;
        }
        else if(mTotalTime >= Constants::TimeoutTimeThreshold) {
            mStatus = SIMULATION_TIMEOUT;
            return false;
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
    Ogre::MovableObject* obj;
    return polygonRayQuery.RaycastFromPoint(mouseRay.getOrigin(), mouseRay.getDirection(), result, obj);
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
    Ogre::WindowEventUtilities::messagePump(); // necessary for OIS to initialize correctly
    initializeOIS();
}

void OgreWindow::hide() {
    shutDownOIS();
    mWindow->setHidden(true);
}

Ogre::SceneManager* OgreWindow::getSceneManager() {
    return mSceneMgr;
}

Ogre::Entity* OgreWindow::getObject() {
    return mObject;
}

Ogre::Vector3 OgreWindow::getInitialCameraPosition() const {
    return mInitialCameraPosition;
}

Ogre::Vector3 OgreWindow::getInitialCameraLookAt() const {
    return mInitialCameraLookAt;
}
