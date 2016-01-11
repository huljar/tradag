#include <TraDaG/ogrewindow.h>
#include <TraDaG/util.h>

#include <OgreBites/OgreRay.h>

#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsStaticPlaneShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsConvexHullShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsSphereShape.h>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <limits>
#include <string>

using namespace TraDaG;

OgreWindow::OgreWindow()
    : mRoot(NULL)
    , mWindow(NULL)
    , mSceneMgr(NULL)
    , mCamera(NULL)
    , mCameraMan(NULL)
    , mHaltRendering(false)
    , mStatus(READY)
    , mInputManager(NULL)
    , mKeyboard(NULL)
    , mMouse(NULL)
    , mRGBDScene(NULL)
    , mRGBDSceneNode(NULL)
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

void OgreWindow::loadSceneCollisionShapes(const std::vector<Ogre::Vector3>& excludeList) {
    // Retrieve vertices of the scene
    Ogre::ManualObject* scene = mRGBDScene->getManualObject();
    size_t vertexCount;
    Ogre::Vector3* vertices;
    size_t indexCount;
    unsigned long* indices;

    OgreBites::OgreRay::GetMeshInformation(scene, vertexCount, vertices, indexCount, indices,
                                           scene->getParentNode()->_getDerivedPosition(),
                                           scene->getParentNode()->_getDerivedOrientation(),
                                           scene->getParentNode()->_getDerivedScale());

    // Iterate over vertices
    mRigidBodies.reserve(vertexCount);
    mCollisionShapes.reserve(vertexCount);
    for(size_t i = 0; i < vertexCount; i += 10) {
        // Check if the vertex is part of the exclude list
        bool exclude = false;
        for(std::vector<Ogre::Vector3>::const_iterator it = excludeList.cbegin(); it != excludeList.cend(); ++it) {
            if(vertices[i].positionEquals(*it)) {
                exclude = true;
                break;
            }
        }

        // Add rigid body and collision shape for this vertex
        if(!exclude) {
            OgreBulletDynamics::RigidBody* rigidBody = new OgreBulletDynamics::RigidBody(
                                                               Strings::VertexRigidBodyName + std::to_string(i),
                                                               mWorld);
            OgreBulletCollisions::CollisionShape* collisionShape = new OgreBulletCollisions::SphereCollisionShape(50);

            rigidBody->setStaticShape(collisionShape, 0.9, 0.1, vertices[i]);

            // Store pointers so they can be cleaned up later
            mRigidBodies.push_back(rigidBody);
            mCollisionShapes.push_back(collisionShape);
        }
    }

    // Clean up
    delete[] vertices;
    delete[] indices;
}

SimulationResult OgreWindow::startSimulation(const ObjectVec& objects, const GroundPlane& plane,
                                             const Ogre::Vector3& gravity, bool drawBulletShapes, bool animate) {
    // Initialize Bullet physics
    initializeBullet(gravity);

    // Draw debug shapes if requested
    mWorld->setShowDebugShapes(drawBulletShapes);

    // Clear any objects from previous simulations
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        Ogre::SceneNode* node = (*it)->getParentSceneNode();
        if(node) {
            node->detachAllObjects();
            mSceneMgr->destroySceneNode(node);
        }
    }
    mObjects.clear();

    // Load objects
    for(ObjectVec::const_iterator it = objects.cbegin(); it != objects.cend(); ++it) {
        Ogre::Entity* entity = (*it)->getOgreEntity();
        mObjects.push_back(entity);

        Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        node->attachObject(entity);
        cv::Vec3f scale = (*it)->getScale();
        node->setScale(Ogre::Vector3(scale[0], scale[1], scale[2]));

        Auto<cv::Vec3f> tmpPos = (*it)->getInitialPosition();
        Ogre::Vector3 position(tmpPos.manualValue[0], tmpPos.manualValue[1], tmpPos.manualValue[2]);
        Ogre::Quaternion rotation(convertCvMatToOgreMat((*it)->getInitialRotation().manualValue));

        // Register object with Bullet
        OgreBulletDynamics::RigidBody* rigidBody = new OgreBulletDynamics::RigidBody(
                                                           Strings::ObjRigidBodyName + std::to_string(std::distance(objects.cbegin(), it)),
                                                           mWorld);
        OgreBulletCollisions::CollisionShape* collisionShape = createConvexHull(entity);

        rigidBody->setShape(node, collisionShape, (*it)->getRestitution(), (*it)->getFriction(), (*it)->getMass(), position, rotation);
        cv::Vec3f velocity = (*it)->getInitialVelocity();
        rigidBody->setLinearVelocity(velocity[0], velocity[1], velocity[2]);

        // If the object shall be upright, ignore any torque passed to the function (anything else wouldn't make sense... right?).
        // Pass zero as angular factor to prevent random infinite object spinning (this can happen if you allow rotation only around
        // the y-axis, i.e. if you pass (0, 1, 0) vector).
        if((*it)->getMustBeUpright()) {
            rigidBody->getBulletRigidBody()->setAngularFactor(0);
        }
        else {
            cv::Vec3f torque = (*it)->getInitialTorque();
            rigidBody->setAngularVelocity(torque[0], torque[1], torque[2]);
        }

        // Store pointers so they can be cleaned up later
        mRigidBodies.push_back(rigidBody);
        mCollisionShapes.push_back(collisionShape);
    }
    // TODO: when initializing Ogre, set stencil shadow type (and ambient light)

    // Register plane with Bullet
    OgreBulletDynamics::RigidBody* planeRigidBody = new OgreBulletDynamics::RigidBody(Strings::PlaneRigidBodyName, mWorld);
    OgreBulletCollisions::CollisionShape* planeCollisionShape = new OgreBulletCollisions::StaticPlaneCollisionShape(plane.getOgrePlane().normal, -plane.getOgrePlane().d);

    planeRigidBody->setStaticShape(planeCollisionShape, plane.getRestitution(), plane.getFriction());

    // Store pointers so they can be cleaned up later
    mRigidBodies.push_back(planeRigidBody);
    mCollisionShapes.push_back(planeCollisionShape);

    // Register scene vertex collisions
    //std::cout << "Loading collision shapes..." << std::flush;
    //loadSceneCollisionShapes(plane.getVertices());
    //std::cout << " done!" << std::endl;

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

bool OgreWindow::queryObjectInfo(const DroppableObject* object, Ogre::Real& occlusion, std::vector<std::pair<Ogre::Vector3, bool>>& pixelInfo, bool& onPlane) const {
    Ogre::Entity* entity = object->getOgreEntity();

    if(std::find(mObjects.begin(), mObjects.end(), entity) != mObjects.end()) {
        // Find bounding box of object in image coordinates
        const Ogre::AxisAlignedBox& boundingBox3D = entity->getWorldBoundingBox(true);

        std::pair<Ogre::Vector2, Ogre::Vector2> boundingBox2D(
            Ogre::Vector2(std::numeric_limits<Ogre::Real>::max(), std::numeric_limits<Ogre::Real>::max()),
            Ogre::Vector2(-std::numeric_limits<Ogre::Real>::max(), -std::numeric_limits<Ogre::Real>::max())
        );

        // Iterate over all corners of the 3D box
        for(const Ogre::Vector3* it = boundingBox3D.getAllCorners(); it < boundingBox3D.getAllCorners() + 8; ++it) {

            // Project the 3D point onto the 2D plane at a specific depth
            Ogre::Vector3 pointXYZ = (*it) * Constants::WorkPlaneDepth / std::abs(it->z);

            // Transform to (u, v, d)
            Ogre::Vector3 pointUVD = mRGBDScene->worldToDepth(pointXYZ);

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
        mRGBDScene->getManualObject()->detachFromParent();

        // Cast rays through all pixels inside the bounding box
        std::vector<Ogre::Vector3> objectImgCoords;
        OgreBites::OgreRay polygonRayCaster(mSceneMgr);

        for(Ogre::Real v = boundingBox2D.first.y; v <= boundingBox2D.second.y; v += 1.0) {
            for(Ogre::Real u = boundingBox2D.first.x; u <= boundingBox2D.second.x; u += 1.0) {
                Ogre::Vector3 result;
                Ogre::MovableObject* objectHit;

                if(polygonRayCaster.RaycastFromPoint(mInitialCameraPosition, mRGBDScene->depthToWorld(u, v, Constants::WorkPlaneDepth) - mInitialCameraPosition, result, objectHit)
                        && objectHit == entity) {
                    objectImgCoords.push_back(mRGBDScene->worldToDepth(result));
                }
            }
        }

        // Reattach the RGBD scene
        mRGBDSceneNode->attachObject(mRGBDScene->getManualObject());

        // Test for all object image coordinates if the scene depth is smaller than the object depth
        cv::Mat depthImg = mRGBDScene->getDepthImage();
        size_t coveredPixels = 0;

        for(std::vector<Ogre::Vector3>::iterator it = objectImgCoords.begin(); it != objectImgCoords.end(); ++it) {
            int u = (int)std::round(it->x);
            int v = (int)std::round(it->y);
            if(it->z >= (Ogre::Real)depthImg.at<unsigned short>(v, u)) {
                ++coveredPixels;
            }
        }

        // Compute and return fraction
        occlusion = (Ogre::Real)coveredPixels / (Ogre::Real)objectImgCoords.size();
        return true;
    }

    return false;
}

bool OgreWindow::render(cv::Mat& depthResult, cv::Mat& rgbResult, const DroppableObject* specificObject) {
    // Determine aspect ratio
    const cv::Mat depthImg = mRGBDScene->getDepthImage();
    Ogre::Vector3 topLeft = mRGBDScene->depthToWorld(0, 0, Constants::WorkPlaneDepth);
    Ogre::Vector3 topRight = mRGBDScene->depthToWorld(depthImg.cols - 1, 0, Constants::WorkPlaneDepth);
    Ogre::Vector3 bottomLeft = mRGBDScene->depthToWorld(0, depthImg.rows - 1, Constants::WorkPlaneDepth);

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

    // Check for specific object
    if(specificObject) {
        // Hide all other objects
        for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
            if(*it != specificObject->getOgreEntity()) {
                (*it)->setVisible(false);
            }
        }
    }

    // Render current frame (RGB)
    mRoot->renderOneFrame(); // ensure that we are up to date
    renderWin->update(); // necessary, otherwise the captured screenshot may contain junk data

    // Allocate storage space (RGB)
    Ogre::PixelFormat renderFormat = renderWin->suggestPixelFormat();
    size_t bytesPerPixel = Ogre::PixelUtil::getNumElemBytes(renderFormat);
    unsigned char* pixelDataRGB = new unsigned char[imgWidth * imgHeight * bytesPerPixel];

    // Get window contents (RGB)
    Ogre::PixelBox pixelBoxRGB(imgWidth, imgHeight, 1, renderFormat, pixelDataRGB);
    renderWin->copyContentsToMemory(pixelBoxRGB);

    // Set material for depth rendering
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        (*it)->setMaterialName(Strings::DepthMapMaterialName);
    }
    mRGBDScene->getManualObject()->setMaterialName(0, Strings::DepthMapMaterialName);

    // Render current frame (Depth)
    mRoot->renderOneFrame();
    renderWin->update();

    // Allocate storage space (Depth)
    unsigned char* pixelDataDepth = new unsigned char[imgWidth * imgHeight * bytesPerPixel];

    // Get window contents (Depth)
    Ogre::PixelBox pixelBoxDepth(imgWidth, imgHeight, 1, renderFormat, pixelDataDepth);
    renderWin->copyContentsToMemory(pixelBoxDepth);

    // Reset material to normal
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        (*it)->setMaterialName("BaseWhiteNoLighting");
    }
    mRGBDScene->getManualObject()->setMaterialName(0, "BaseWhiteNoLighting");

    // Reattach hidden objects
    if(specificObject) {
        for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
            if(*it != specificObject->getOgreEntity()) {
                (*it)->setVisible(true);
            }
        }
    }

    // Render again to update the preview window
    mRoot->renderOneFrame();
    mWindow->update();

    // Convert and copy RGB to cv::Mat
    cv::Mat renderImageRGB(pixelBoxRGB.getHeight(), pixelBoxRGB.getWidth(), CV_8UC3);
    for(int y = 0; y < renderImageRGB.rows; ++y) {
        for(int x = 0; x < renderImageRGB.cols; ++x) {
            Ogre::ColourValue value = pixelBoxRGB.getColourAt(x, y, 0);
            // OpenCV uses BGR color channel ordering and (row, col) pixel addresses
            renderImageRGB.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(value.b * 255.0),
                static_cast<unsigned char>(value.g * 255.0),
                static_cast<unsigned char>(value.r * 255.0)
            );
        }
    }

    // Convert and copy Depth to cv::Mat
    cv::Mat renderImageDepth(pixelBoxDepth.getHeight(), pixelBoxDepth.getWidth(), CV_16U);
    for(int y = 0; y < renderImageDepth.rows; ++y) {
        for(int x = 0; x < renderImageDepth.cols; ++x) {
            Ogre::ColourValue value = pixelBoxDepth.getColourAt(x, y, 0);
            renderImageDepth.at<unsigned short>(y, x) = (static_cast<unsigned short>(value.g * 255.0) << 8)
                                                        + static_cast<unsigned short>(value.b * 255.0);
        }
    }

    // Crop and resize image
    // Cropping is necessary because the image has black borders if the principal point of the camera is not the image center
    // TODO: what if object slided partly outside of the image? calculate cropping from principal point/camera fov angles
    cv::Mat renderImageGray;
    cv::cvtColor(renderImageRGB, renderImageGray, CV_BGR2GRAY);

    // Gather all non-black points
    std::vector<cv::Point> points;
    for(cv::Mat_<unsigned char>::iterator it = renderImageGray.begin<unsigned char>(); it != renderImageGray.end<unsigned char>(); ++it) {
        if(*it) points.push_back(it.pos());
    }

    // Compute bounding rectangle of points
    cv::Rect roi = cv::boundingRect(points);

    // Resize the remaining area and copy to output parameter
    cv::Size resultSize(depthImg.cols, depthImg.rows);
    cv::resize(renderImageDepth(roi), depthResult, resultSize);
    cv::resize(renderImageRGB(roi), rgbResult, resultSize);

    // Clean up
    delete[] pixelDataDepth;
    delete[] pixelDataRGB;
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

void OgreWindow::setScene(RGBDScene* scene, bool updateCameraFOV) {
    if(!mRGBDSceneNode)
        mRGBDSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    else if(mRGBDScene)
        mRGBDScene->getManualObject()->detachFromParent();

    mRGBDScene = scene;
    mRGBDSceneNode->attachObject(mRGBDScene->getManualObject());

    if(updateCameraFOV) {
        const cv::Mat depthImg = mRGBDScene->getDepthImage();
        Ogre::Vector3 topLeft = mRGBDScene->depthToWorld(0.0, 0.0, Constants::WorkPlaneDepth);
        Ogre::Vector3 bottomLeft = mRGBDScene->depthToWorld(0.0, (Ogre::Real)(depthImg.rows - 1), Constants::WorkPlaneDepth);

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
    if(mRGBDSceneNode && mRGBDScene) {
        if(vertices.size() == 0) {
            if(mVertexMarkings && !mVertexMarkings->isAttached()) {
                mRGBDSceneNode->attachObject(mVertexMarkings);
            }
        }
        else {
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

            mRGBDSceneNode->attachObject(mVertexMarkings);
        }
    }
}

void OgreWindow::unmarkVertices(bool destroy) {
    if(mVertexMarkings) {
        mVertexMarkings->detachFromParent();
        if(destroy) {
            mSceneMgr->destroyManualObject(mVertexMarkings);
            mVertexMarkings = NULL;
        }
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
    // Get current object positions/rotations
    std::vector<Ogre::Quaternion> oldRotations;
    std::vector<Ogre::Vector3> oldPositions;
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        Ogre::SceneNode* node = (*it)->getParentSceneNode();
        oldRotations.push_back(node->getOrientation());
        oldPositions.push_back(node->getPosition());
    }

    // Step the simulation
    mWorld->stepSimulation(timeElapsed, 4);

    // Add elapsed time
    mTotalTime += timeElapsed;

    // Check if the objects are still moving
    bool allIdle = true;
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        Ogre::SceneNode* node = (*it)->getParentSceneNode();
        Ogre::Quaternion newRotation = node->getOrientation();
        Ogre::Vector3 newPosition = node->getPosition();

        unsigned int idx = std::distance(mObjects.begin(), it);

        if(!orientationEquals(oldRotations[idx], newRotation) || !oldPositions[idx].positionEquals(newPosition)) {
            allIdle = false;
            mIdleTime = 0;
            break;
        }
    }

    if(allIdle) {
        mIdleTime += timeElapsed;
        if(mIdleTime >= Constants::IdleTimeThreshold) {
            return true;
        }
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

Ogre::Vector3 OgreWindow::getInitialCameraPosition() const {
    return mInitialCameraPosition;
}

Ogre::Vector3 OgreWindow::getInitialCameraLookAt() const {
    return mInitialCameraLookAt;
}
