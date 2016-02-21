/************************************************************//**
 * @mainpage
 *
 * @tableofcontents
 *
 * @section intro_sec Introduction
 *
 * @b TraDaG (short for <em>Training Data Generator</em>) is a framework for dropping objects
 * into scenes which are given only by an RGB image, a depth image, and a labeling of the scene contents
 * in form of a label image. The goal is to provide a way to insert objects into these scenes in a
 * physically plausible way (i.e. to not end up with floating objects or impossible resting poses, where
 * the object would normally tilt over). This is achieved by using a physics engine to drop these
 * objects on a plane extracted from the depth and label images. The resulting scene with the objects
 * inside can then be rendered into new RGB and depth images. In addition, the framework will provide
 * the following information for each object:
 *  - Final object pose (rotation and translation),
 *  - Amount of occlusion
 *  - Object-local coordinates and visibility of every object pixel
 *
 * The rendered images and the additional information can then be used in whatever way the user wishes.
 * The main purpose at the time of writing this framework is to provide physically plausible training
 * and testing data for machine learning algorithms, which are supposed to robustly recognize objects in
 * images and (optionally) detect the 6D pose (rotation and translation) for each object.
 *
 * The framework allows for fine-grained control over the whole process and can easily be used with big
 * data sets of RGB-D scenes. It supports reading in whole directories of RGB, depth and label images
 * and searching them for scenes suitable for specific ground planes (e.g. scenes which contain a table
 * that is viewed by the camera from a bird's eye view). See the @ref features_sec "features section"
 * for a more detailed list of features.
 *
 * @section install_sec Installation
 *
 * @subsection ogre_sec OGRE
 * @b OGRE (short for <em>Object-oriented Graphics Rendering Engine</em>) is the underlying rendering
 * engine used to create and manage the 3D scene, the objects to be dropped into the scene and to render
 * the result images. TraDaG was created and tested with OGRE 1.9 \"Ghadamon\".
 *
 * @subsubsection ogre_source_sec Compiling from source
 * - Go to the <a href="http://www.ogre3d.org/download/sdk">OGRE downloads website</a>
 * - Follow the instructions in the @a Source section
 *
 * @subsubsection ogre_prebuilt_sec Installing an official prebuilt SDK
 * - Go to the <a href="http://www.ogre3d.org/download/sdk">OGRE downloads website</a>
 * - Follow the instructions in the <em>Pre-built SDK</em> section
 *
 * @subsubsection ogre_repo_sec Installing from a repository (Debian, Ubuntu)
 * - Install the package @a libogre-1.9-dev and its dependencies (e.g. by executing \"<tt>apt-get
 *   install libogre-1.9-dev</tt>\" as root)
 *
 * @subsection ois_sec OIS (Object-oriented Input System)
 * @b OIS (short for <em>Object-oriented Input System</em>) is the library used to process mouse and
 * keyboard inputs when having a preview window open (see @ref getting_started_sec "Getting Started" for
 * details on preview windows). TraDaG was created and tested with OIS 1.3.
 *
 * @subsubsection ois_source_sec Compiling from source
 * - Go to the <a href="https://github.com/wgois/OIS">GitHub repository</a> of OIS
 * - Grab a copy of the source code (e.g. by cloning the repository with @c git)
 * - Follow the instructions in the @a README.md file
 *
 * @subsubsection ois_repo_sec Installing from a repository (Debian, Ubuntu)
 * - Install the package @a libois-dev and its dependencies (e.g. by executing \"<tt>apt-get install
 *   libois-dev</tt>\" as root)
 *
 * @subsection bullet_sec Bullet
 * @b Bullet is the physics engine that is responsible for dropping objects into the scenes by
 * calculating their trajectories, how they bounce from the ground, how they collide with other objects
 * and so forth. TraDaG was created and tested with Bullet 2.82/2.83.
 *
 * @subsubsection bullet_source_sec Compiling from source
 * - Go to the <a href="https://github.com/bulletphysics/bullet3/releases">GitHub repository</a> of Bullet
 * - Grab a copy of the source code (e.g. by downloading the latest release)
 * - Follow the instructions in the @a README.md file
 *
 * @subsubsection bullet_repo_sec Installing from a repository (Debian, Ubuntu)
 * - Install the packages @a libbullet-dev and @a libbullet-extras-dev and their dependencies (e.g. by
 *   executing \"<tt>apt-get install libbullet-dev libbullet-extras-dev</tt>\" as root)
 *
 * @subsection ogrebullet_sec OgreBullet
 *
 * @subsection opencv_sec OpenCV
 *
 * @subsection boost_sec Boost
 *
 * @section features_sec Features
 *
 * @section getting_started_sec Getting Started
 *
*//*************************************************************/

#include <TraDaG/Simulator.h>
#include <TraDaG/util.h>
#include <TraDaG/CVLDWrapper/CVLDWrapper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace TraDaG;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
//    // Parse command line using Boost Program Options library
//    po::options_description argDesc("Available options");
//    argDesc.add_options()
//            ("scenes-path,s", po::value<std::string>(), "Path containing the depth, rgb, label subdirectories")
//            ("object-id,o", po::value<unsigned int>(), "Object mesh ID")
//            ("label-names,l", po::value<std::vector<std::string>>()->multitoken(), "Label names")
//            ("occlusion,n", po::value<std::vector<float>>()->multitoken(), "Desired object occlusion (min max) (default: 0 0.95)")
//            ("max-attempts,m", po::value<unsigned int>(), "Maximum number of total simulations (default: 50000)")
//            ("max-scenes", po::value<unsigned int>(), "Maximum number of scenes to load (default: 0 = load all)")
//            ("dont-compute-planes", "Do not try to find a plane using RANSAC if no suitable planeinfo file exists (default: false)")
//            ("precompute-only", "Only precompute planes for the given parameters, don't do any simulations (default: false)")
//            ("object-scale", po::value<float>(), "Object scaling factor (default: 1000)")
//            ("object-mass", po::value<float>(), "Object mass (default: 1)")
//            ("object-restitution", po::value<float>(), "Object restitution (default: 0.4)")
//            ("object-friction", po::value<float>(), "Object friction (default: 0.7)")
//            ("plane-restitution", po::value<float>(), "Plane restitution (default: 0.1)")
//            ("plane-friction", po::value<float>(), "Plane friction (default: 0.9)")
//            ("show-preview,p", "Display a preview window (default: false)")
//            ("animate,a", "Show the object falling into the scene (default: false)")
//            ("help", "Display this help and exit")
//            ("version", "Display version information and exit");

//    po::variables_map vm;
//    po::store(po::parse_command_line(argc, argv, argDesc), vm);
//    po::notify(vm);

//    // If the version option was given, display version information and exit
//    if(vm.count("version")) {
//        std::cout << "Training Data Generator 0.1\nAuthor: Julian Harttung\nCreated for the Computer Vision Lab Dresden" << std::endl;
//        return 0;
//    }

//    // If the help option was given (or no valid options), display available options and exit
//    if(vm.size() == 0 || vm.count("help") || !vm.count("scenes-path") || !vm.count("object-id") || !vm.count("label-names")) {
//        std::cout << argDesc << std::endl;
//        return 0;
//    }

//    // Get file paths
//    std::string scenesPath = vm["scenes-path"].as<std::string>();

//    // Get label map
//    LabelMap labelMap = Labels::NYUDepthV1;

//    // Get camera data
//    cv::Vec2f principalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
//    cv::Vec2f focalLength(5.7616540758591043e+02, 5.7375619782082447e+02);

//    // Get additional parameters
//    unsigned int objectID = vm["object-id"].as<unsigned int>();

//    std::vector<std::string> labelNames = vm["label-names"].as<std::vector<std::string>>();

//    std::pair<float, float> occlusion(0.0, 0.95);
//    if(vm.count("occlusion")) {
//        std::vector<float> tmpOcclusion = vm["occlusion"].as<std::vector<float>>();
//        if(tmpOcclusion.size() >= 1) occlusion.first = tmpOcclusion[0];
//        if(tmpOcclusion.size() >= 2) occlusion.second = tmpOcclusion[1];
//    }

//    unsigned int maxScenes = (vm.count("max-scenes") ? vm["max-scenes"].as<unsigned int>() : 0);

//    // Create camera manager
//    CameraManager camManager(principalPoint, focalLength, principalPoint, focalLength);

//    // --- BEGIN --- //
//    CVLDWrapper wrapper(scenesPath, camManager, labelMap, maxScenes);
//    wrapper.setLabelsToUse(labelNames);
//    wrapper.setActiveObject(objectID);

//    wrapper.setComputePlaneIfNoFile(!vm.count("dont-compute-planes"));
//    wrapper.setShowPreviewWindow(vm.count("show-preview"));
//    wrapper.setShowPhysicsAnimation(vm.count("animate"));

//    if(vm.count("max-attempts")) wrapper.setMaxAttempts(vm["max-attempts"].as<unsigned int>());
//    if(vm.count("object-scale")) wrapper.setObjectScale(vm["object-scale"].as<float>());
//    if(vm.count("object-mass")) wrapper.setObjectMass(vm["object-mass"].as<float>());
//    if(vm.count("object-restitution")) wrapper.setObjectRestitution(vm["object-restitution"].as<float>());
//    if(vm.count("object-friction")) wrapper.setObjectFriction(vm["object-friction"].as<float>());
//    if(vm.count("plane-restitution")) wrapper.setPlaneRestitution(vm["plane-restitution"].as<float>());
//    if(vm.count("plane-friction")) wrapper.setPlaneFriction(vm["plane-friction"].as<float>());

//    if(vm.count("precompute-only")) {
//        // Precompute planes
//        if(wrapper.precomputePlaneInfo(labelNames))
//            std::cout << "Successfully computed planes for all scenes and each label" << std::endl;
//        else
//            std::cerr << "Plane computation finished, but somewhere an error occured.\n"
//                      << "This is probably caused by some scenes not containing one of the labels." << std::endl;
//    }
//    else {
//        // Get a training image
//        std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> result = wrapper.getTrainingImage(occlusion.first, occlusion.second);

//        // Show result
//        if(result.second == Simulator::DropStatus::SUCCESS || result.second == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
//            // Display stats
//            std::cout << "Result:\n    Image ID: " << result.first.imageID
//                      << "\n    Optimal: " << std::boolalpha << (result.second == Simulator::DropStatus::SUCCESS) << std::noboolalpha
//                      << "\n    Occlusion: " << result.first.occlusion
//                      << "\n    Translation: " << result.first.translation
//                      << "\n    Rotation: " << result.first.rotation << std::endl;

//            cv::namedWindow("Depth");
//            cv::namedWindow("RGB");
//            cv::imshow("Depth", result.first.depth);
//            cv::imshow("RGB", result.first.bgr);

//            cv::waitKey();
//        }
//    }

    LabelMap labelMap = Labels::NYUDepthV1;
    CameraManager camManager(cv::Vec2f(3.2442516903961865e+02, 2.3584766381177013e+02),
                             cv::Vec2f(5.7616540758591043e+02, 5.7375619782082447e+02),
                             cv::Vec2f(3.2442516903961865e+02, 2.3584766381177013e+02),
                             cv::Vec2f(5.7616540758591043e+02, 5.7375619782082447e+02));

    std::string basePath("/home/julian/Forschungsprojekt/datasets/NYU-Labeled-V1/");
    SceneAnalyzer sa(basePath + "depth", basePath + "rgb", basePath + "label",
                     basePath + "plane", camManager, labelMap);

    // Precompute planes
    //bool precompResult = sa.precomputePlaneInfoForAllScenes("floor", cv::Vec3f(0, 1, 0), 20.0);
    //std::cout << std::boolalpha << "Precompute result (floor): " << precompResult << std::endl;
    //bool precompResult2 = sa.precomputePlaneInfoForAllScenes("table", cv::Vec3f(0, 1, 0), 20.0);
    //std::cout << "Precompute result (table): " << precompResult2 << std::noboolalpha << std::endl;

    std::vector<std::string> objects({"003.mesh", "007.mesh", "005.mesh", "010.mesh", "013.mesh"});

    for(SceneAnalyzer::PlaneIterator it = sa.beginByPlane(std::vector<std::string>({"floor", "table"}), cv::Vec3f(0, 1, 0), 15.0, 0, 60000, false, false);
            it != sa.endByPlane(); ++it) {
        Simulator sim = sa.createSimulator(it->first, it->second);
        sim.setMaxAttempts(20);

        for(size_t i = 0; i < objects.size(); ++i) {
            DroppableObject* obj = sim.createObject(objects[i]);
            for(float j = 0.0; j < 0.9; j += 0.2) {
                obj->setDesiredOcclusion(j, j + 0.2);

                Simulator::DropResult result = sim.execute();
                std::ostringstream fileName;
                fileName << sa.getFileName(it->first) << "_" << it->second.getLabel() << "_" << objects[i] << "_"
                         << std::setprecision(3) << j << "-" << j + 0.2 << "_" << obj->getFinalOcclusion();
                if(result.status == Simulator::DropStatus::SUCCESS || result.status == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
                    if(result.status == Simulator::DropStatus::SUCCESS)
                        fileName << "_optimal";

                    cv::imwrite(basePath + "result/" + fileName.str() + "_depth.png", result.depthImage);
                    cv::imwrite(basePath + "result/" + fileName.str() + "_rgb.png", result.rgbImage);
                }
            }
            sim.destroyAllObjects();
        }
    }

    return 0;
}
