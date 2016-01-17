#include <TraDaG/TradagMain.h>
#include <TraDaG/OgreWindow.h>
#include <TraDaG/RGBDScene.h>
#include <TraDaG/DroppableObject.h>
#include <TraDaG/GroundPlane.h>
#include <TraDaG/util.h>
#include <TraDaG/SceneAnalyzer.h>

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
    SceneAnalyzer sa(depthPath, rgbPath, labelPath, camManager, labelMap);

    std::vector<unsigned int> floorIDs = sa.findScenesByLabel("floor");

    for(std::vector<unsigned int>::iterator it = floorIDs.begin(); it != floorIDs.end(); ++it) {
        std::cout << "Simulating for ID " << *it << std::endl;
        TradagMain tradag = sa.createSimulator(*it);

        tradag.setShowPreviewWindow(preview);
        tradag.setShowPhysicsAnimation(animate);
        //tradag.setDebugMarkInlierSet(true);
        //tradag.setDebugDrawBulletShapes(true);
        //tradag.setGravity(Auto<cv::Vec3f>(false, cv::Vec3f(0, -1000, 0)));
        tradag.setMaxAttempts(5);

        // Create an object
        DroppableObject* obj = tradag.createObject(meshName);
        obj->setDesiredOcclusion(0.2, 0.4);
        obj->setInitialAzimuth(M_PI_2);
        obj->setInitialTorque(5, 5, 5);
        //obj->setInitialVelocity(400, 100, -400);

        // Create another object
        DroppableObject* obj2 = tradag.createObject("003.mesh");
        obj2->setDesiredOcclusion(0.0, 0.5);
        //obj2->setInitialVelocity(400, 100, -400);
        //obj2->setInitialAzimuth(M_PI_2);

        // Compute ground plane
        // TODO: sa.createImageLabeling(ID)
        cv::Mat depthImg, rgbImg, labelImg;
        sa.readImages(*it, depthImg, rgbImg, labelImg);

        ImageLabeling labeling(depthImg, labelImg, labelMap, camManager);
        GroundPlane plane;
        if(labeling.findPlaneForLabel(labelName, plane) != PF_SUCCESS) {
            std::cerr << "Error: unable to compute plane for label \"" << labelName << "\"" << std::endl;
            return 1;
        }
        tradag.setGroundPlane(plane);

        // Execute simulation
        ObjectDropResult result = tradag.execute();

        // Evaluate result
        if(result.status == OD_SUCCESS) {
            std::cout << "Success!" << std::endl
                      << "Occlusion: " << obj->getFinalOcclusion() << std::endl
                      << "Rotation: " << obj->getFinalRotation() << std::endl
                      << "Position: " << obj->getFinalPosition() << std::endl;

            cv::namedWindow("Depth");
            cv::imshow("Depth", result.depthImage);
            cv::namedWindow("RGB");
            cv::imshow("RGB", result.rgbImage);
            cv::waitKey();
        }
    }

    return 0;
}
