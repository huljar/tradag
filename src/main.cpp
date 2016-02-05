#include <TraDaG/Simulator.h>
#include <TraDaG/util.h>
#include <TraDaG/CVLDWrapper/CVLDWrapper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

#include <iostream>
#include <limits>
#include <string>

using namespace TraDaG;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
    // Parse command line using Boost Program Options library
    po::options_description argDesc("Available options");
    argDesc.add_options()
            ("scenes-path,s", po::value<std::string>(), "Path containing the depth, rgb, label subdirectories")
            ("object-id,o", po::value<unsigned int>(), "Object mesh ID")
            ("label-names,l", po::value<std::vector<std::string>>()->multitoken(), "Label names")
            ("occlusion,n", po::value<std::vector<float>>()->multitoken(), "Desired object occlusion (min max) (default: 0 0.95)")
            ("max-attempts,m", po::value<unsigned int>(), "Maximum number of total simulations (default: 50000)")
            ("max-scenes", po::value<unsigned int>(), "Maximum number of scenes to load (default: 0 = load all)")
            ("compute-nofile", "Try to find a plane using RANSAC if no suitable planeinfo file exists (default: false)")
            ("precompute-only", "Only precompute planes for the given parameters, don't do any simulations (default: false)")
            ("object-scale", po::value<float>(), "Object scaling factor (default: 1000)")
            ("object-mass", po::value<float>(), "Object mass (default: 1)")
            ("object-restitution", po::value<float>(), "Object restitution (default: 0.4)")
            ("object-friction", po::value<float>(), "Object friction (default: 0.7)")
            ("plane-restitution", po::value<float>(), "Plane restitution (default: 0.1)")
            ("plane-friction", po::value<float>(), "Plane friction (default: 0.9)")
            ("show-preview,p", "Display a preview window (default: false)")
            ("animate,a", "Show the object falling into the scene (default: false)")
            ("help", "Display this help and exit")
            ("version", "Display version information and exit");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, argDesc), vm);
    po::notify(vm);

    // If the version option was given, display version information and exit
    if(vm.count("version")) {
        std::cout << "Training Data Generator 0.1\nAuthor: Julian Harttung\nCreated for the Computer Vision Lab Dresden" << std::endl;
        return 0;
    }

    // If the help option was given (or no valid options), display available options and exit
    if(vm.size() == 0 || vm.count("help") || !vm.count("scenes-path") || !vm.count("object-id") || !vm.count("label-names")) {
        std::cout << argDesc << std::endl;
        return 0;
    }

    // Get file paths
    std::string scenesPath = vm["scenes-path"].as<std::string>();

    // Get label map
    LabelMap labelMap = Labels::NYUDepthV1;

    // Get camera data
    cv::Vec2f principalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
    cv::Vec2f focalLength(5.7616540758591043e+02, 5.7375619782082447e+02);

    // Get additional parameters
    unsigned int objectID = vm["object-id"].as<unsigned int>();

    std::vector<std::string> labelNames = vm["label-names"].as<std::vector<std::string>>();

    std::pair<float, float> occlusion(0.0, 0.95);
    if(vm.count("occlusion")) {
        std::vector<float> tmpOcclusion = vm["occlusion"].as<std::vector<float>>();
        if(tmpOcclusion.size() >= 1) occlusion.first = tmpOcclusion[0];
        if(tmpOcclusion.size() >= 2) occlusion.second = tmpOcclusion[1];
    }

    unsigned int maxScenes = (vm.count("max-scenes") ? vm["max-scenes"].as<unsigned int>() : 0);

    // Create camera manager
    CameraManager camManager(principalPoint, focalLength, principalPoint, focalLength);

    // --- BEGIN --- //
    CVLDWrapper wrapper(scenesPath, camManager, labelMap, maxScenes);
    wrapper.setLabelsToUse(labelNames);
    wrapper.setActiveObject(objectID);

    wrapper.setComputePlaneIfNoFile(vm.count("compute-nofile"));
    wrapper.setShowPreviewWindow(vm.count("show-preview"));
    wrapper.setShowPhysicsAnimation(vm.count("animate"));

    if(vm.count("max-attempts")) wrapper.setMaxAttempts(vm["max-attempts"].as<unsigned int>());
    if(vm.count("object-scale")) wrapper.setObjectScale(vm["object-scale"].as<float>());
    if(vm.count("object-mass")) wrapper.setObjectMass(vm["object-mass"].as<float>());
    if(vm.count("object-restitution")) wrapper.setObjectRestitution(vm["object-restitution"].as<float>());
    if(vm.count("object-friction")) wrapper.setObjectFriction(vm["object-friction"].as<float>());
    if(vm.count("plane-restitution")) wrapper.setPlaneRestitution(vm["plane-restitution"].as<float>());
    if(vm.count("plane-friction")) wrapper.setPlaneFriction(vm["plane-friction"].as<float>());

    if(vm.count("precompute-only")) {
        // Precompute planes
        if(wrapper.precomputePlaneInfo(labelNames))
            std::cout << "Successfully computed planes for all scenes and each label" << std::endl;
        else
            std::cerr << "Plane computation finished, but somewhere an error occured.\n"
                      << "This is probably caused by some scenes not containing one of the labels." << std::endl;
    }
    else {
        // Get a training image
        std::pair<CVLDWrapper::TrainingImage, Simulator::DropStatus> result = wrapper.getTrainingImage(occlusion.first, occlusion.second);

        // Show result
        if(result.second == Simulator::DropStatus::SUCCESS || result.second == Simulator::DropStatus::MAX_ATTEMPTS_REACHED) {
            // Display stats
            std::cout << "Result:\n    Image ID: " << result.first.imageID
                      << "\n    Optimal: " << std::boolalpha << (result.second == Simulator::DropStatus::SUCCESS) << std::noboolalpha
                      << "\n    Occlusion: " << result.first.occlusion
                      << "\n    Translation: " << result.first.translation
                      << "\n    Rotation: " << result.first.rotation << std::endl;

            cv::namedWindow("Depth");
            cv::namedWindow("RGB");
            cv::imshow("Depth", result.first.depth);
            cv::imshow("RGB", result.first.bgr);

            cv::waitKey();
        }
    }

    return 0;
}
