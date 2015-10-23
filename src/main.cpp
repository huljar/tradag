#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <string>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    // Parse command line using Boost Program Options library
    po::options_description argDesc("Available options");
    argDesc.add_options()
            ("rgb-file,c", po::value<std::string>(), "RGB (color) input file")
            ("depth-file,d", po::value<std::string>(), "Depth input file")
            ("label-file,l", po::value<std::string>(), "Label input file")
            ("verbose,v", "Display result in a window")
            ("animate,a", "Show the object dropping into the scene")
            ("help", "Display this help")
            ("version", "Display version information");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, argDesc), vm);
    po::notify(vm);

    // If the help option was given (or no options), display available options and exit
    if(vm.size() == 0 || vm.count("help")) {
        std::cout << argDesc << std::endl;
        return 0;
    }

    // If the version option was given, display version information and exit
    if(vm.count("version")) {
        std::cout << "Training Data Generator 0.01+dfsg3+deb8u1" << std::endl;
        return 0;
    }

    // Get file paths
    std::string rgbFile = vm["rgb-file"].as<std::string>();
    std::string depthFile = vm["depth-file"].as<std::string>();
    std::string labelFile = vm["label-file"].as<std::string>();

    // Init OGRE
    OgreWindow* testWindow = new OgreWindow();
    testWindow->initialize();

    // Create RGBD object
    RgbdObject* scene = new RgbdObject("sceneObject");
    if(!(scene->loadRgbFromFile(rgbFile)
            && scene->loadDepthFromFile(depthFile)
            && scene->loadLabelsFromFile(labelFile))) {
        std::cerr << "ERROR: unable to load the required images" << std::endl;
        return 1;
    }

    scene->setCameraParams(
                Ogre::Vector2(3.2850951551345941e+02, 2.5282555217253503e+02),
                Ogre::Vector2(5.1930334103339817e+02, 5.1816401430246583e+02));
    scene->meshify();

    testWindow->setScene(scene);

    testWindow->enterRenderingLoop();

    // Clean up
    delete scene;
    delete testWindow;

    return 0;
}
