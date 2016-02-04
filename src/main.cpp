#include <TraDaG/Simulator.h>
#include <TraDaG/OgreWindow.h>
#include <TraDaG/RGBDScene.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/PlaneInfo.h>
#include <TraDaG/util.h>
#include <TraDaG/SceneAnalyzer.h>
#include <TraDaG/interop.h>
#include <TraDaG/CVLDWrapper/CVLDWrapper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

#include <iostream>
#include <string>

using namespace TraDaG;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
    // Parse command line using Boost Program Options library
    po::options_description argDesc("Available options");
    argDesc.add_options()
            ("depth-path,d", po::value<std::string>(), "Depth input path")
            ("rgb-path,c", po::value<std::string>(), "RGB (color) input path")
            ("label-path,l", po::value<std::string>(), "Label input path")
            ("mesh-name,m", po::value<std::string>(), "Object mesh name")
            ("label-name,n", po::value<std::string>(), "Label name")
            ("show-preview,p", "Display a preview window")
            ("animate,a", "Show the object falling into the scene")
            ("help", "Display this help and exit")
            ("version", "Display version information and exit");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, argDesc), vm);
    po::notify(vm);

    // If the help option was given (or no options), display available options and exit
    if(vm.size() == 0 || vm.count("help") || !vm.count("depth-path") || !vm.count("rgb-path") || !vm.count("label-path")
            || !vm.count("mesh-name") || !vm.count("label-name")) {
        std::cout << argDesc << std::endl;
        return 0;
    }

    // If the version option was given, display version information and exit
    if(vm.count("version")) {
        std::cout << "Training Data Generator 0.01+dfsg3+deb8u1" << std::endl;
        return 0;
    }

    // Get file paths
    std::string depthPath = vm["depth-path"].as<std::string>();
    std::string rgbPath = vm["rgb-path"].as<std::string>();
    std::string labelPath = vm["label-path"].as<std::string>();

    // Get label map
    LabelMap labelMap = Labels::NYUDepthV1;

    // Get camera data
    cv::Vec2f depthPrincipalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
    cv::Vec2f depthFocalLength(5.7616540758591043e+02, 5.7375619782082447e+02);

    // Get additional parameters
    std::string meshName = vm["mesh-name"].as<std::string>();
    std::string labelName = vm["label-name"].as<std::string>();

    bool preview = vm.count("show-preview");
    bool animate = vm.count("animate");

    // Create camera manager
    CameraManager camManager(depthPrincipalPoint, depthFocalLength, depthPrincipalPoint, depthFocalLength);

    // --- BEGIN TESTING --- //
    SceneAnalyzer sa(depthPath, rgbPath, labelPath, "../resources/scenes/plane", camManager, labelMap);

//    std::map<unsigned int, std::string> ids = sa.findScenesByLabel(labelName);

//    Simulator tradag = sa.createSimulator(ids.begin()->first);
//    Simulator tradag2 = sa.createSimulator((++ids.begin())->first);

//    tradag.setShowPreviewWindow(preview);
//    tradag.setShowPhysicsAnimation(animate);
//    tradag.setDebugMarkInlierSet(true);
//    //tradag.setDebugDrawBulletShapes(true);
//    //tradag.setGravity(Auto<cv::Vec3f>(false, cv::Vec3f(0, -1000, 0)));
//    tradag.setMaxAttempts(5);

//    // Create an object
//    DroppableObject* obj = tradag.createObject(meshName);
//    obj->setDesiredOcclusion(0.2, 0.4);
//    obj->setInitialAzimuth(270);
//    obj->setInitialTorque(5, 5, 5);
//    //obj->setInitialVelocity(400, 100, -400);
//    obj->setMustBeUpright(true);

//    tradag2.setShowPreviewWindow(preview);
//    tradag2.setShowPhysicsAnimation(animate);
//    tradag2.setDebugMarkInlierSet(true);

//    // Create another object
//    DroppableObject* obj2 = tradag2.createObject("003.mesh");
//    obj2->setDesiredOcclusion(0.0, 0.5);
//    //obj2->setInitialVelocity(400, 100, -400);
//    obj2->setInitialAzimuth(90);
//    obj2->setInitialVelocity(0, 500, 0);
//    obj2->setInitialTorque(8, 8 ,8);

//    // Compute ground plane
//    GroundPlane plane;
//    ImageLabeling labeling = sa.createImageLabeling(ids.begin()->first);
//    if(labeling.findPlaneForLabel(labelName, plane, cv::Vec3f(0, 1, 0), 14, 3500, 4000) != PF_SUCCESS) {
//        std::cerr << "Error: unable to compute plane for label \"" << labelName << "\"" << std::endl;
//        return 1;
//    }

//    // Save plane
//    plane.saveToFile("../resources/scenes/plane/" + sa.getFileName(ids.begin()->first), true);

//    // Read plane
//    GroundPlane np = GroundPlane::readFromFile("../resources/scenes/plane/" + sa.getFileName(ids.begin()->first));
//    if(!np.isPlaneDefined()) {
//        std::cerr << "Error: plane was not read correctly!" << std::endl;
//        return 1;
//    }

//    // Set actual plane
//    PlaneInfo pi;
//    labeling.findPlaneForLabel(labelName, pi);

//    tradag.setGroundPlane(pi.createGroundPlane());

//    // Compute ground plane info
//    PlaneInfo planeInfo;
//    labeling = sa.createImageLabeling((++ids.begin())->first);
//    if(labeling.findPlaneForLabel(labelName, planeInfo) != PF_SUCCESS) {
//        std::cerr << "Error: unable to compute plane info for label \"" << labelName << "\"" << std::endl;
//        return 1;
//    }

//    // Save plane info
//    planeInfo.saveToFile("../resources/scenes/plane/" + sa.getFileName((++ids.begin())->first), true);

//    // Read plane info
//    PlaneInfo npi  = PlaneInfo::readFromFile("../resources/scenes/plane/" + sa.getFileName((++ids.begin())->first));

//    // Create ground plane from info
//    GroundPlane createdPlane = npi.createGroundPlane(3000, 5000, PlaneInfo::PickMode::WEIGHTED_RANDOM);
//    if(!createdPlane.isPlaneDefined()) {
//        std::cerr << "Error: plane was not read correctly" << std::endl;
//        return 1;
//    }

//    tradag2.setGroundPlane(createdPlane);

//    // Execute simulation
//    Simulator::DropResult result = tradag.execute();

//    // Evaluate result
//    if(result.status == Simulator::DropStatus::SUCCESS) {
//        std::cout << "Success!" << std::endl
//                  << "Occlusion: " << obj->getFinalOcclusion() << std::endl
//                  << "Rotation: " << obj->getFinalRotation() << std::endl
//                  << "Position: " << obj->getFinalPosition() << std::endl;

//        // Compare depth values
//        cv::Mat depthImg, rgbImg, labelImg;
//        sa.readImages(ids.begin()->first, depthImg, rgbImg, labelImg);

//        cv::Mat compareMat(depthImg.rows, depthImg.cols, CV_16U);
//        for(int y = 0; y < depthImg.rows; y += 1) {
//            for(int x = 0; x < depthImg.cols; x += 1) {
//                unsigned short diff = std::abs(depthImg.at<unsigned short>(y, x) - result.depthImage.at<unsigned short>(y, x));
//                if(diff > 80)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max();
//                else if(diff > 45)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.75;
//                else if(diff > 20)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.5;
//                else if(diff > 5)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.25;
//                else
//                    compareMat.at<unsigned short>(y, x) = 0;
//            }
//        }
//        cv::namedWindow("Depth Comparison");
//        cv::imshow("Depth Comparison", compareMat);

//        cv::namedWindow("Depth");
//        cv::imshow("Depth", result.depthImage);
//        cv::namedWindow("RGB");
//        cv::imshow("RGB", result.rgbImage);
//        cv::waitKey();
//    }

//    // Execute simulation
//    result = tradag2.execute();

//    // Evaluate result
//    if(result.status == Simulator::DropStatus::SUCCESS) {
//        std::cout << "Success!" << std::endl
//                  << "Occlusion: " << obj2->getFinalOcclusion() << std::endl
//                  << "Rotation: " << obj2->getFinalRotation() << std::endl
//                  << "Position: " << obj2->getFinalPosition() << std::endl;

//        // Compare depth values
//        cv::Mat depthImg, rgbImg, labelImg;
//        sa.readImages((++ids.begin())->first, depthImg, rgbImg, labelImg);

//        cv::Mat compareMat(depthImg.rows, depthImg.cols, CV_16U);
//        for(int y = 0; y < depthImg.rows; y += 1) {
//            for(int x = 0; x < depthImg.cols; x += 1) {
//                unsigned short diff = std::abs(depthImg.at<unsigned short>(y, x) - result.depthImage.at<unsigned short>(y, x));
//                if(diff > 80)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max();
//                else if(diff > 45)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.75;
//                else if(diff > 20)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.5;
//                else if(diff > 5)
//                    compareMat.at<unsigned short>(y, x) = std::numeric_limits<unsigned short>::max() * 0.25;
//                else
//                    compareMat.at<unsigned short>(y, x) = 0;
//            }
//        }
//        cv::namedWindow("Depth Comparison");
//        cv::imshow("Depth Comparison", compareMat);

//        // Display result
//        cv::namedWindow("Depth");
//        cv::imshow("Depth", result.depthImage);
//        cv::namedWindow("RGB");
//        cv::imshow("RGB", result.rgbImage);
//        cv::waitKey();
//    }

//    Simulator sim5 = sa.createSimulator(5);
//    ImageLabeling lab5 = sa.createImageLabeling(5);
//    GroundPlane plane5;
//    lab5.findPlaneForLabel(labelName, plane5, cv::Vec3f(0, 0, 1), 45);
//    if(!plane5.isPlaneDefined()) {
//        std::cerr << "Not able to fit image 5" << std::endl;
//        exit(1);
//    }
//    sim5.setGroundPlane(plane5);
//    sim5.setShowPreviewWindow(true);
//    sim5.setShowPhysicsAnimation(true);
//    sim5.setDebugMarkInlierSet(true);
//    sim5.createObject("001.mesh");
//    sim5.execute();

//    // Test precomputation of planes
//    //sa.precomputePlaneInfoForAllScenes(labelName, cv::Vec3f(0, 1, 0));

//    // Test finding scenes by plane
//    std::map<unsigned int, GroundPlane> scenes = sa.findScenesByPlane(labelName, cv::Vec3f(0, 1, 0), 10, 1500, 6000);
//    std::cout << "Found " << scenes.size() << " scenes" << std::endl;
//    for(auto it = scenes.begin(); it != scenes.end(); ++it) {
//        // Test onPlane test
//        Simulator sim = sa.createSimulator(it->first, it->second);
//        DroppableObject* obj = sim.createObject(meshName);
//        //obj->setInitialVelocity(0, 0, -100);
//        obj->setInitialTorque(20, 0, 0);
//        sim.setShowPreviewWindow(preview);
//        sim.setShowPhysicsAnimation(animate);
//        sim.setDebugMarkInlierSet(true);
//        sim.execute();
//    }

//    // Test object coordinate calculation
//    std::map<unsigned int, GroundPlane> scenes = sa.findScenesByPlane(labelName, cv::Vec3f(0, 1, 0), 15);
//    for(auto it = scenes.begin(); it != scenes.end(); ++it) {
//        Simulator sim = sa.createSimulator(it->first, it->second);
//        DroppableObject* obj = sim.createObject(meshName);
//        obj->setMustBeUpright(true);
//        obj->setInitialAzimuth(180);
//        sim.setShowPreviewWindow(preview);
//        sim.setShowPhysicsAnimation(animate);
//        Simulator::DropResult res = sim.execute();

//        if(res.status == Simulator::DropStatus::SUCCESS) {
//            cv::namedWindow("RGB image");
//            cv::imshow("RGB image", res.rgbImage);

//            // Print object coordinates
//            DroppableObject::PixelInfoMap objCoords = obj->getFinalObjectCoords();
//            std::cout << "\nObject coordinates:\n";
//            for(auto jt = objCoords.begin(); jt != objCoords.end(); ++jt) {
//                std::cout << "(" << jt->first.x << ", " << jt->first.y << "): "
//                          << jt->second.first << ", " << std::boolalpha << jt->second.second << std::noboolalpha << "\n";
//            }
//            std::cout << std::flush;

////            cv::Mat visibility(res.depthImage.rows, res.depthImage.cols, CV_16U, cv::Scalar(0));
////            for(auto jt = objCoords.begin(); jt != objCoords.end(); ++jt) {
////                if(jt->second.second) visibility.at<unsigned short>(jt->first) = std::numeric_limits<unsigned short>::max();
////                else visibility.at<unsigned short>(jt->first) = std::numeric_limits<unsigned short>::max() * 0.4;
////            }
////            cv::namedWindow("Visibility");
////            cv::imshow("Visibility", visibility);
////            cv::waitKey();
//            cv::Mat testObjCoords = res.rgbImage.clone();
//            for(auto jt = objCoords.begin(); jt != objCoords.end(); ++jt) {
//                testObjCoords.at<cv::Vec3b>(jt->first) = cv::Vec3b(255, 255, 255);
//            }
//            cv::namedWindow("Object Coordinate Pixel Accuracy Test");
//            cv::imshow("Object Coordinate Pixel Accuracy Test", testObjCoords);
//            cv::waitKey();
//        }
//        break;
//    }

//    // Test nearest neighbor algorithm and KD tree
//    std::map<unsigned int, GroundPlane> scenes2 = sa.findScenesByPlane(labelName, cv::Vec3f(0, 1, 0), 15);
//    for(auto it = scenes2.begin(); it != scenes2.end(); ++it) {
//        Simulator sim = sa.createSimulator(it->first, it->second);
//        DroppableObject* obj = sim.createObject(meshName);
//        sim.setShowPreviewWindow(preview);
//        sim.setShowPhysicsAnimation(animate);
//        sim.setDebugMarkInlierSet(true);
//        Simulator::DropResult res = sim.execute();

//        if(res.status == Simulator::DropStatus::SUCCESS) {
//            OgreWindow& ogreWin = OgreWindow::getSingleton();
//            Ogre::SceneManager* sceneMgr = ogreWin.getSceneManager();

//            DroppableObject::PixelInfoMap objCoords = obj->getFinalObjectCoords();

//            for(auto jt = objCoords.begin(); jt != objCoords.end(); ++jt) {
//                if(jt->second.second) {
//                    Ogre::Vector3 visPx = cvToOgre(camManager.getWorldForDepth(jt->first, res.depthImage.at<unsigned short>(jt->first)));
//                    Ogre::Vector3 projPx = it->second.projectVectorOntoPlane(visPx);

//                    std::vector<Ogre::Vector3> nearestNeighbors = it->second.findKNearestNeighbors(projPx, 20);
//                    if(nearestNeighbors.size() < 3) {
//                        std::cerr << "ERROR: only " << nearestNeighbors.size() << " neighbors found!" << std::endl;
//                        exit(1);
//                    }

//                    Ogre::ManualObject* manObj = sceneMgr->createManualObject();
//                    manObj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
//                    manObj->position(visPx);
//                    manObj->colour(0.0, 1.0, 1.0);
//                    manObj->position(projPx);
//                    manObj->colour(1.0, 0.0, 0.0);
//                    for(auto kt = nearestNeighbors.begin(); kt != nearestNeighbors.end(); ++kt) {
//                        manObj->position(projPx);
//                        manObj->colour(1.0, 1.0, 0.0);
//                        manObj->position(*kt);
//                        manObj->colour(1.0, 0.0, 1.0);
//                    }
//                    manObj->end();

//                    sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manObj);

//                    break;
//                }
//            }

//            ogreWin.show();
//            ogreWin.promptUserAction();
//        }
//        break;
//    }

    // Test CVLD wrapper
    std::vector<std::string>& objects = CVLDWrapper::availableObjects();
    objects.clear();
    objects.push_back("003.mesh");
    objects.push_back("007.mesh");

    CVLDWrapper wrapper("../resources/scenes", camManager, labelMap, 10);
    wrapper.labelsToUse().push_back(labelName);
    wrapper.setActiveObject(1);
    wrapper.setShowPreviewWindow(preview);
    wrapper.setShowPhysicsAnimation(animate);
    std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> result = wrapper.getTrainingImage(0.2, 0.6);
    if(result.second == Simulator::DropStatus::SUCCESS || result.second == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
        cv::namedWindow("Depth");
        cv::namedWindow("RGB");
        cv::namedWindow("Object");
        cv::imshow("Depth", result.first.depth);
        cv::imshow("RGB", result.first.bgr);
        cv::Mat obj = result.first.obj.clone();
        for(cv::Mat_<cv::Vec3s>::iterator it = obj.begin<cv::Vec3s>(); it != obj.end<cv::Vec3s>(); ++it) {
            if(*it != cv::Vec3s(0, 0, 0)) *it = cv::Vec3s(std::numeric_limits<short>::max(), std::numeric_limits<short>::max(), std::numeric_limits<short>::max());
        }
        cv::imshow("Object", obj);

        std::cout << "Result:" << std::endl
                  << "    Image ID: " << result.first.imageID << std::endl
                  << "    Occlusion: " << result.first.occlusion << std::endl
                  << "    Translation: " << result.first.translation << std::endl
                  << "    Rotation: " << result.first.rotation << std::endl;

        cv::waitKey();
    }

    std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> result2 = wrapper.getTrainingImage(result.first.rotation, cv::Point3d(200, 0, 200));
    if(result2.second == Simulator::DropStatus::SUCCESS || result.second == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
        cv::namedWindow("Depth");
        cv::namedWindow("RGB");
        cv::namedWindow("Object");
        cv::imshow("Depth", result2.first.depth);
        cv::imshow("RGB", result2.first.bgr);
        cv::Mat obj = result2.first.obj.clone();
        for(cv::Mat_<cv::Vec3s>::iterator it = obj.begin<cv::Vec3s>(); it != obj.end<cv::Vec3s>(); ++it) {
            if(*it != cv::Vec3s(0, 0, 0)) *it = cv::Vec3s(std::numeric_limits<short>::max(), std::numeric_limits<short>::max(), std::numeric_limits<short>::max());
        }
        cv::imshow("Object", obj);

        std::cout << "Result:" << std::endl
                  << "    Image ID: " << result2.first.imageID << std::endl
                  << "    Occlusion: " << result2.first.occlusion << std::endl
                  << "    Translation: " << result2.first.translation << std::endl
                  << "    Rotation: " << result2.first.rotation << std::endl;

        cv::waitKey();
    }

    std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> result3 = wrapper.getTrainingImage(result2.first.rotation, 45, 1000, 2500);
    if(result3.second == Simulator::DropStatus::SUCCESS || result.second == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
        cv::namedWindow("Depth");
        cv::namedWindow("RGB");
        cv::namedWindow("Object");
        cv::imshow("Depth", result3.first.depth);
        cv::imshow("RGB", result3.first.bgr);
        cv::Mat obj = result3.first.obj.clone();
        for(cv::Mat_<cv::Vec3s>::iterator it = obj.begin<cv::Vec3s>(); it != obj.end<cv::Vec3s>(); ++it) {
            if(*it != cv::Vec3s(0, 0, 0)) *it = cv::Vec3s(std::numeric_limits<short>::max(), std::numeric_limits<short>::max(), std::numeric_limits<short>::max());
        }
        cv::imshow("Object", obj);

        std::cout << "Result:" << std::endl
                  << "    Image ID: " << result3.first.imageID << std::endl
                  << "    Occlusion: " << result3.first.occlusion << std::endl
                  << "    Translation: " << result3.first.translation << std::endl
                  << "    Rotation: " << result3.first.rotation << std::endl;

        cv::waitKey();
    }

    return 0;
}
