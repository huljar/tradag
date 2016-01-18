#include <TraDaG/OgreWindow.h>
#include <TraDaG/debug.h>
#include <TraDaG/interop.h>

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreConfigFile.h>
#include <OGRE/OgreException.h>
#include <OGRE/OgreMath.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreStringVector.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/Overlay/OgreOverlay.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreBorderPanelOverlayElement.h>
#include <OGRE/Overlay/OgreTextAreaOverlayElement.h>

#include <OgreBites/OgreRay.h>

#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsConvexHullShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsSphereShape.h>
#include <OgreBullet/Collisions/Shapes/OgreBulletCollisionsStaticPlaneShape.h>

#include <bullet/BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <bullet/BulletCollision/CollisionShapes/btShapeHull.h>
#include <bullet/LinearMath/btScalar.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <limits>
#include <stdexcept>
#include <string>

using namespace TraDaG;

OgreWindow* OgreWindow::msSingleton = nullptr;

OgreWindow& OgreWindow::getSingleton() {
    if(!msSingleton)
        msSingleton = new OgreWindow();
    return *msSingleton;
}

OgreWindow* OgreWindow::getSingletonPtr() {
    if(!msSingleton)
        msSingleton = new OgreWindow();
    return msSingleton;
}

OgreWindow::OgreWindow()
    : mRoot(nullptr)
    , mPreviewWindow(nullptr)
    , mSceneMgr(nullptr)
    , mPreviewCamera(nullptr)
    , mPreviewCameraMan(nullptr)
    , mLogManager(nullptr)
    , mOverlaySystem(nullptr)
    , mInputManager(nullptr)
    , mKeyboard(nullptr)
    , mMouse(nullptr)
    , mRGBDScene(nullptr)
    , mRGBDSceneNode(nullptr)
    , mVertexMarkings(nullptr)
    , mVertexMarkingsNode(nullptr)
    , mInitialCameraPosition(Constants::DefaultCameraPosition)
    , mInitialCameraLookAt(Constants::DefaultCameraLookAt)
    , mWorld(nullptr)
    , mDebugDrawer(nullptr)
    , mBounds(Ogre::Vector3(-10000, -10000, -10000), Ogre::Vector3(10000, 10000, 10000))
    , mIdleTime(0)
    , mTotalTime(0)
    , mHaltRendering(false)
    , mStatus(READY)
    , mActionChosen(UA_KEEP)
    , mRenderWindow(nullptr)
    , mRenderCamera(nullptr)
    , mRenderPixelBoxDepth(nullptr)
    , mRenderPixelBoxDepthData(nullptr)
    , mRenderPixelBoxRGB(nullptr)
    , mRenderPixelBoxRGBData(nullptr)
{
    initializeOgre();
}

OgreWindow::~OgreWindow() {
    shutDownBullet();
    shutDownOIS();
    shutDownOgre();
}

void OgreWindow::initializeOgre() {
    // Create logger that logs to a file and not to console
    mLogManager = new Ogre::LogManager();
    mLogManager->createLog(Strings::LogfilePath, true, false, false);

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
    Ogre::RenderSystem* rs = nullptr;

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

    rs->setConfigOption("Full Screen", "No");
    rs->setConfigOption("VSync", "Yes");

    mRoot->setRenderSystem(rs);

    // Create overlay system (this must be done before OGRE initializes)
    mOverlaySystem = new Ogre::OverlaySystem();

    // Init the render system
    mRoot->initialise(false);

    // Create a render window (hidden by default)
    Ogre::NameValuePairList windowParams;
    windowParams["vsync"] = "true";
    windowParams["hidden"] = "true";

    mPreviewWindow = mRoot->createRenderWindow(Strings::PreviewWindowName, 1024, 768, false, &windowParams);

    // Init resources
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    // Create scene manager
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

    // Create camera
    mPreviewCamera = mSceneMgr->createCamera("MainCamera");
    mPreviewCamera->setPosition(mInitialCameraPosition);
    mPreviewCamera->lookAt(mInitialCameraLookAt);
    mPreviewCamera->setNearClipDistance(5.0);
    mPreviewCamera->setFOVy(Ogre::Degree(55.0));

    // Create camera controller
    mPreviewCameraMan = new OgreBites::SdkCameraMan(mPreviewCamera);
    mPreviewCameraMan->setTopSpeed(300);

    // Add viewport
    Ogre::Viewport* vp = mPreviewWindow->addViewport(mPreviewCamera);
    vp->setBackgroundColour(Ogre::ColourValue::Black);

    mPreviewCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
    mPreviewCamera->setAutoAspectRatio(true);

    // Add window event listener
    Ogre::WindowEventUtilities::addWindowEventListener(mPreviewWindow, this);

    // Add frame listener
    mRoot->addFrameListener(this);

    // Register overlay system
    mSceneMgr->addRenderQueueListener(mOverlaySystem);
}

void OgreWindow::shutDownOgre() {
    delete[] mRenderPixelBoxDepthData;
    delete[] mRenderPixelBoxRGBData;
    delete mRenderPixelBoxDepth;
    delete mRenderPixelBoxRGB;

    delete mOverlaySystem;
    delete mPreviewCameraMan;
    Ogre::WindowEventUtilities::removeWindowEventListener(mPreviewWindow, this);
    delete mRoot;
    delete mLogManager;
}

void OgreWindow::initializeOIS() {
    if(!mInputManager) {
        Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
        OIS::ParamList pl;
        size_t hWnd = 0;
        std::ostringstream hWndStr;

        mPreviewWindow->getCustomAttribute("WINDOW", &hWnd);
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
        windowResized(mPreviewWindow);

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
        mInputManager = nullptr;
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
        mWorld = nullptr;

        delete mDebugDrawer;
        mDebugDrawer = nullptr;
    }
}

SimulationResult OgreWindow::startSimulation(const ObjectVec& objects, RGBDScene* scene, const GroundPlane& plane,
                                             const Ogre::Vector3& gravity, bool drawBulletShapes, bool animate) {

    // Ensure that we have a scene node for our scene
    if(!mRGBDSceneNode)
        mRGBDSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

    // Set the new scene if necessary
    if(mRGBDScene != scene) {
        mRGBDSceneNode->detachAllObjects();
        mRGBDSceneNode->attachObject(scene->getManualObject());
        mRGBDScene = scene;

        // Adjust FOV of preview camera to view the whole scene
        cv::Mat depthImg = mRGBDScene->getDepthImage();
        CameraManager& camMgr = mRGBDScene->cameraManager();

        Ogre::Vector3 top = cvToOgre(camMgr.getWorldForDepth(0, 0, Constants::WorkPlaneDepth));
        Ogre::Vector3 bottom = cvToOgre(camMgr.getWorldForDepth(0, depthImg.rows - 1, Constants::WorkPlaneDepth));
        top.x = 0.0;
        bottom.x = 0.0;

        Ogre::Radian angle(std::max(
            top.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians(),
            bottom.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians()
        ) * 2.0 + 0.04);
        mPreviewCamera->setFOVy(angle);

        // Ensure that the render settings are recreated on the next call to render()
        invalidateRenderSettings();
    }

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
        Ogre::Quaternion rotation(cvToOgre((*it)->getInitialRotation().manualValue));

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
    mActionChosen = UA_KEEP;

    // Create overlay
    Ogre::OverlayManager& overlayMgr = Ogre::OverlayManager::getSingleton();
    Ogre::Overlay* overlay = overlayMgr.create(Strings::UserInputPromptOverlayName);

    // Create elements
    Ogre::BorderPanelOverlayElement* panel = static_cast<Ogre::BorderPanelOverlayElement*>(overlayMgr.createOverlayElement("BorderPanel", Strings::UserInputPromptPanelName));
    overlay->add2D(panel);

    Ogre::TextAreaOverlayElement* textArea = static_cast<Ogre::TextAreaOverlayElement*>(overlayMgr.createOverlayElement("TextArea", Strings::UserInputPromptTextAreaName));
    panel->addChild(textArea);

    // Set properties of panel
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setMaterialName(Strings::InfoTrayMaterialName);
    panel->setUV(0.4, 0.4, 0.6, 0.6);

    panel->setBorderMaterialName(Strings::InfoTrayMaterialName);
    panel->setBorderSize(13, 13, 13, 13);
    panel->setTopLeftBorderUV(0.0, 0.0, 0.4, 0.4);
    panel->setTopBorderUV(0.4, 0.0, 0.6, 0.4);
    panel->setTopRightBorderUV(0.6, 0.0, 1.0, 0.4);
    panel->setLeftBorderUV(0.0, 0.4, 0.4, 0.6);
    panel->setRightBorderUV(0.6, 0.4, 1.0, 0.6);
    panel->setBottomLeftBorderUV(0.0, 0.6, 0.4, 1.0);
    panel->setBottomBorderUV(0.4, 0.6, 0.6, 1.0);
    panel->setBottomRightBorderUV(0.6, 0.6, 1.0, 1.0);

    panel->setPosition(10, 10);
    panel->setDimensions(230, 125);

    // Set properties of text area
    textArea->setMetricsMode(Ogre::GMM_PIXELS);
    textArea->setPosition(15, 15);
    textArea->setDimensions(200, 95);
    textArea->setFontName(Strings::UserInputPromptFontName);
    textArea->setCharHeight(19);
    textArea->setSpaceWidth(6);
    textArea->setColour(Ogre::ColourValue::Black);
    textArea->setCaption(Strings::UserInputPromptText);

    // Display overlay
    overlay->show();

    // Start rendering (key presses are parsed in rendering callbacks)
    if(!hidden())
        mRoot->startRendering();

    // Destroy overlay
    overlayMgr.destroyOverlayElement(textArea);
    overlayMgr.destroyOverlayElement(panel);
    overlayMgr.destroy(overlay);

    // Return action
    mStatus = READY;
    return mActionChosen;
}

bool OgreWindow::queryObjectInfo(const DroppableObject* object, float& occlusion, unsigned short& distance,
                                 PixelInfoMap& pixelInfo, bool& onPlane) {

    Ogre::Entity* entity = object->getOgreEntity();
    Ogre::SceneNode* node = entity->getParentSceneNode();

    if(std::find(mObjects.begin(), mObjects.end(), entity) != mObjects.end()) {
        // Render scene (once only the object and once the whole scene)
        cv::Mat objectDepthRender, objectRGBRender, sceneDepthRender, sceneRGBRender;
        if(render(objectDepthRender, objectRGBRender, object) && render(sceneDepthRender, sceneRGBRender)) {

            // Gather all object pixels
            PixelInfoMap objectPixels(
                [] (const cv::Point& lhs, const cv::Point& rhs) -> bool {
                    return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
                }
            );
            for(cv::Mat_<unsigned short>::iterator it = objectDepthRender.begin<unsigned short>(); it != objectDepthRender.end<unsigned short>(); ++it) {
                // TODO: Calculate object coordinates for this pixel
                cv::Vec3s objCoords(0, 0, 0);
                if(*it) objectPixels.insert(std::make_pair(it.pos(), std::make_pair(objCoords, true)));
            }

            // Check object pixels for visibility in the scene
            size_t occludedPixels = 0;
            for(PixelInfoMap::iterator it = objectPixels.begin(); it != objectPixels.end(); ++it) {
                if(sceneDepthRender.at<unsigned short>(it->first) < objectDepthRender.at<unsigned short>(it->first)) {
                    ++occludedPixels;
                    it->second.second = false;
                }
            }

            // Calculate occlusion and set output parameter
            if(objectPixels.size() > 0)
                occlusion = static_cast<Ogre::Real>(occludedPixels) / static_cast<Ogre::Real>(objectPixels.size());
            else
                occlusion = 1.0;

            // Calculate distance of object center to camera plane and set output parameter
            DepthPixel depthPx = mRGBDScene->cameraManager().getActualDepthForWorld(ogreToCv(node->getPosition()));
            distance = depthPx.second;

            // Set pixel info output parameter
            pixelInfo = std::move(objectPixels);

            // TODO: Check if object is on plane
            onPlane = true;

            return true;
        }
    }

    return false;
}

bool OgreWindow::render(cv::Mat& depthResult, cv::Mat& rgbResult, const DroppableObject* specificObject) {
    // Ensure that everything is set up correctly
    setUpRenderSettings();

    // Check for specific object
    if(specificObject) {
        // Hide all other objects
        for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
            if(*it != specificObject->getOgreEntity()) {
                (*it)->setVisible(false);
            }
        }
        mRGBDScene->getManualObject()->setVisible(false);
    }

    // Render current frame (RGB)
    mRenderWindow->update(); // ensure that we are up to date

    // Get window contents (RGB)
    mRenderWindow->copyContentsToMemory(*mRenderPixelBoxRGB);

    // Set materials for depth rendering
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        (*it)->setMaterialName(Strings::DepthMapMaterialName);
    }
    mRGBDScene->getManualObject()->setMaterialName(0, Strings::DepthMapMaterialName);

    // Render current frame (Depth)
    mRenderWindow->update(); // apply the material changes

    // Get window contents (Depth)
    mRenderWindow->copyContentsToMemory(*mRenderPixelBoxDepth);

    // Reset materials to normal
    for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
        (*it)->setMaterialName(Strings::StandardMaterialName);
    }
    mRGBDScene->getManualObject()->setMaterialName(0, Strings::StandardMaterialName);

    // Show hidden objects again if they were hidden before
    if(specificObject) {
        for(std::vector<Ogre::Entity*>::iterator it = mObjects.begin(); it != mObjects.end(); ++it) {
            if(*it != specificObject->getOgreEntity()) {
                (*it)->setVisible(true);
            }
        }
        mRGBDScene->getManualObject()->setVisible(true);
    }

    // Convert and copy RGB to cv::Mat
    cv::Mat renderImageRGB(mRenderPixelBoxRGB->getHeight(), mRenderPixelBoxRGB->getWidth(), CV_8UC3);
    for(int y = 0; y < renderImageRGB.rows; ++y) {
        for(int x = 0; x < renderImageRGB.cols; ++x) {
            Ogre::ColourValue value = mRenderPixelBoxRGB->getColourAt(x, y, 0);
            // OpenCV uses BGR color channel ordering and (row, col) pixel addresses
            renderImageRGB.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(value.b * 255.0),
                static_cast<unsigned char>(value.g * 255.0),
                static_cast<unsigned char>(value.r * 255.0)
            );
        }
    }

    // Convert and copy Depth to cv::Mat
    cv::Mat renderImageDepth(mRenderPixelBoxDepth->getHeight(), mRenderPixelBoxDepth->getWidth(), CV_16U);
    for(int y = 0; y < renderImageDepth.rows; ++y) {
        for(int x = 0; x < renderImageDepth.cols; ++x) {
            Ogre::ColourValue value = mRenderPixelBoxDepth->getColourAt(x, y, 0);
            renderImageDepth.at<unsigned short>(y, x) = (static_cast<unsigned short>(value.g * 255.0) << 8)
                                                        + static_cast<unsigned short>(value.b * 255.0);
        }
    }

    // Get screenspace bounding box of the scene
    Ogre::Vector2 screenTopLeft, screenBottomRight;
    if(!mRGBDScene->screenspaceCoords(mRenderCamera, screenTopLeft, screenBottomRight))
        return false;

    // Compute region of interest
    cv::Rect roi(cv::Point(screenTopLeft.x * renderImageDepth.cols, screenTopLeft.y * renderImageDepth.rows),
                 cv::Point(screenBottomRight.x * renderImageDepth.cols, screenBottomRight.y * renderImageDepth.rows));

    // Resize the remaining area and copy to output parameter
    // Use nearest neighbor interpolation to ensure no new (and incorrect) depth values are created
    cv::Mat depthImg = mRGBDScene->getDepthImage();
    cv::Size resultSize(depthImg.cols, depthImg.rows);
    cv::resize(renderImageDepth(roi), depthResult, resultSize, 0, 0, cv::INTER_NEAREST);
    cv::resize(renderImageRGB(roi), rgbResult, resultSize);

    return true;
}

void OgreWindow::invalidate(const ObjectVec& objects, const RGBDScene* scene) {
    for(ObjectVec::const_iterator it = objects.cbegin(); it != objects.cend(); ++it) {
        invalidate(*it);
    }

    invalidate(scene);
}

void OgreWindow::invalidate(const DroppableObject* object) {
    // Check if the invalidated object is stored here
    std::vector<Ogre::Entity*>::iterator pos = std::find(mObjects.begin(), mObjects.end(), object->getOgreEntity());
    if(pos != mObjects.end()) {
        Ogre::SceneNode* node = (*pos)->getParentSceneNode();
        if(node) {
            node->detachAllObjects();
            mSceneMgr->destroySceneNode(node);
        }
        mObjects.erase(pos);
    }
}

void OgreWindow::invalidate(const RGBDScene* scene) {
    // Check if the invalidated scene is active
    if(scene && mRGBDScene == scene) {
        mRGBDScene = nullptr;
        unmarkVertices(true);
    }
}

void OgreWindow::resetCamera() {
    if(mPreviewCamera) {
        mPreviewCamera->setPosition(mInitialCameraPosition);
        mPreviewCamera->lookAt(mInitialCameraLookAt);
    }
}

void OgreWindow::markVertices(const std::vector<Ogre::Vector3>& vertices) {
    if(!mVertexMarkingsNode)
        mVertexMarkingsNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

    if(vertices.size() == 0) {
        if(mVertexMarkings && !mVertexMarkings->isAttached()) {
            mVertexMarkingsNode->attachObject(mVertexMarkings);
        }
    }
    else {
        if(mVertexMarkings) {
            unmarkVertices(true);
        }

        mVertexMarkings = mSceneMgr->createManualObject();

        mVertexMarkings->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
        for(std::vector<Ogre::Vector3>::const_iterator it = vertices.cbegin(); it != vertices.cend(); ++it) {
            mVertexMarkings->position(*it);
        }
        mVertexMarkings->end();

        mVertexMarkingsNode->attachObject(mVertexMarkings);
    }
}

void OgreWindow::unmarkVertices(bool destroy) {
    if(mVertexMarkings) {
        mVertexMarkings->detachFromParent();
        if(destroy) {
            mSceneMgr->destroyManualObject(mVertexMarkings);
            mVertexMarkings = nullptr;
        }
    }
}

bool OgreWindow::frameStarted(const Ogre::FrameEvent& evt) {
    if(mHaltRendering || mPreviewWindow->isClosed()) {
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
    if(mHaltRendering || mPreviewWindow->isClosed()) {
        mHaltRendering = false;
        return false;
    }

    if(mInputManager) {
        mKeyboard->capture();
        mMouse->capture();
    }

    mPreviewCameraMan->frameRenderingQueued(evt);
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
    if(rw == mPreviewWindow) {
        // Allow the window to be closed if it is hidden
        if(hidden())
            return true;

        // If the window is visible, only hide it instead of closing it
        hide();

        // Halt rendering loop
        mHaltRendering = true;

        // Adjust status
        if(mStatus == AWAITING_USER_INPUT)
            mActionChosen = UA_ABORT;

        mStatus = WINDOW_CLOSED;

        return false;
    }

    return true;
}

bool OgreWindow::keyPressed(const OIS::KeyEvent& e) {
    mPreviewCameraMan->injectKeyDown(e);
    if(e.key == OIS::KC_SPACE) {
        resetCamera();
    }

    if(mStatus == AWAITING_USER_INPUT) {
        if(e.key == OIS::KC_RETURN) {
            mActionChosen = UA_KEEP;
            mHaltRendering = true;
        }
        else if(e.key == OIS::KC_ESCAPE) {
            mActionChosen = UA_ABORT;
            mHaltRendering = true;
        }
        else if(e.key == OIS::KC_R) {
            mActionChosen = UA_RESTART;
            mHaltRendering = true;
        }
    }

    return true;
}

bool OgreWindow::keyReleased(const OIS::KeyEvent& e) {
    mPreviewCameraMan->injectKeyUp(e);
    return true;
}

bool OgreWindow::mouseMoved(const OIS::MouseEvent& e) {
    mPreviewCameraMan->injectMouseMove(e);
    return true;
}

bool OgreWindow::mousePressed(const OIS::MouseEvent& e, const OIS::MouseButtonID button) {
    mPreviewCameraMan->injectMouseDown(e, button);
    return true;
}

bool OgreWindow::mouseReleased(const OIS::MouseEvent& e, const OIS::MouseButtonID button) {
    mPreviewCameraMan->injectMouseUp(e, button);
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
    Ogre::Viewport* vp = mPreviewCamera->getViewport();
    Ogre::Ray mouseRay = mPreviewCamera->getCameraToViewportRay(
                             (Ogre::Real)mouseX / (Ogre::Real)vp->getActualWidth(),
                             (Ogre::Real)mouseY / (Ogre::Real)vp->getActualHeight());

    OgreBites::OgreRay polygonRayQuery(mSceneMgr);
    Ogre::MovableObject* obj;
    return polygonRayQuery.RaycastFromPoint(mouseRay.getOrigin(), mouseRay.getDirection(), result, obj);
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

void OgreWindow::setUpRenderSettings() {
    if(!mRenderWindow) {
        // Determine aspect ratio
        const cv::Mat depthImg = mRGBDScene->getDepthImage();
        CameraManager& camMgr = mRGBDScene->cameraManager();

        Ogre::Vector3 topLeft = cvToOgre(camMgr.getWorldForDepth(0, 0, Constants::WorkPlaneDepth));
        Ogre::Vector3 bottomRight = cvToOgre(camMgr.getWorldForDepth(depthImg.cols - 1, depthImg.rows - 1, Constants::WorkPlaneDepth));

        Ogre::Real width = bottomRight.x - topLeft.x;
        Ogre::Real height = topLeft.y - bottomRight.y;
        if(height == 0.0)
            throw std::runtime_error("Division by zero error while setting up render settings");

        // Determine vertical field of view (horizontal fov is automatically adjusted according to aspect ratio)
        Ogre::Vector3 top(0, topLeft.y, topLeft.z);
        Ogre::Vector3 bottom(0, bottomRight.y, bottomRight.z);
        Ogre::Radian verticalFOV(std::max(
            top.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians(),
            bottom.angleBetween(Ogre::Vector3::NEGATIVE_UNIT_Z).valueRadians()
        ) * 2.0 + 0.04);

        // Create a hidden render window
        Ogre::NameValuePairList windowParams;
        windowParams["vsync"] = "true";
        windowParams["hidden"] = "true";

        Ogre::uint32 imgWidth = depthImg.cols * 1.1; // render in larger resolution because we crop black edges later
        Ogre::uint32 imgHeight = depthImg.rows * 1.1;

        mRenderWindow = mRoot->createRenderWindow(Strings::RenderWindowName, imgWidth, imgHeight, false, &windowParams);

        // Create camera to use for rendering
        mRenderCamera = mSceneMgr->createCamera("RenderCamera");

        // Set camera parameters
        mRenderCamera->setPosition(mInitialCameraPosition);
        mRenderCamera->lookAt(mInitialCameraLookAt);
        mRenderCamera->setNearClipDistance(5.0);
        mRenderCamera->setFOVy(verticalFOV);
        mRenderCamera->setAspectRatio(width / height);

        // Add viewport
        Ogre::Viewport* renderViewport = mRenderWindow->addViewport(mRenderCamera);
        renderViewport->setBackgroundColour(Ogre::ColourValue::Black);
        renderViewport->setOverlaysEnabled(false);

        // Allocate storage space
        Ogre::PixelFormat renderFormat = mRenderWindow->suggestPixelFormat();
        size_t bytesPerPixel = Ogre::PixelUtil::getNumElemBytes(renderFormat);

        mRenderPixelBoxDepthData = new unsigned char[imgWidth * imgHeight * bytesPerPixel];
        mRenderPixelBoxRGBData = new unsigned char[imgWidth * imgHeight * bytesPerPixel];

        mRenderPixelBoxDepth = new Ogre::PixelBox(imgWidth, imgHeight, 1, renderFormat, mRenderPixelBoxDepthData);
        mRenderPixelBoxRGB = new Ogre::PixelBox(imgWidth, imgHeight, 1, renderFormat, mRenderPixelBoxRGBData);
    }
}

void OgreWindow::invalidateRenderSettings() {
    if(mRenderWindow) {
        delete[] mRenderPixelBoxDepthData;
        delete[] mRenderPixelBoxRGBData;
        delete mRenderPixelBoxDepth;
        delete mRenderPixelBoxRGB;

        mSceneMgr->destroyCamera(mRenderCamera);
        mRoot->destroyRenderTarget(mRenderWindow);

        mRenderWindow = nullptr;
        mRenderCamera = nullptr;
        mRenderPixelBoxDepth = nullptr;
        mRenderPixelBoxDepthData = nullptr;
        mRenderPixelBoxRGB = nullptr;
        mRenderPixelBoxRGBData = nullptr;
    }
}

bool OgreWindow::hidden() const {
    return mPreviewWindow->isHidden();
}

void OgreWindow::show() {
    mPreviewWindow->setHidden(false);
    Ogre::WindowEventUtilities::messagePump(); // necessary for OIS to initialize correctly
    initializeOIS();
}

void OgreWindow::hide() {
    shutDownOIS();
    mPreviewWindow->setHidden(true);
    Ogre::WindowEventUtilities::messagePump();
}

Ogre::SceneManager* OgreWindow::getSceneManager() const {
    return mSceneMgr;
}

Ogre::Vector3 OgreWindow::getInitialCameraPosition() const {
    return mInitialCameraPosition;
}

Ogre::Vector3 OgreWindow::getInitialCameraLookAt() const {
    return mInitialCameraLookAt;
}
