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
            ("preview,p", "Preview result in a window")
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

    // Init window
    OgreWindow* previewWindow = new OgreWindow();
    previewWindow->initializeOgre();
    previewWindow->initializeBullet(Ogre::Vector3(0, -100, 0));

    // Create RGBD object
    RgbdObject* scene = new RgbdObject("sceneObject", previewWindow->getSceneManager());
    if(!(scene->loadRgbFromFile(rgbFile)
            && scene->loadDepthFromFile(depthFile)
            && scene->loadLabelsFromFile(labelFile))) {
        std::cerr << "ERROR: unable to load the required images" << std::endl;
        return 1;
    }

    scene->setDepthPrincipalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
    scene->setDepthFocalLength(5.7616540758591043e+02, 5.7375619782082447e+02);
    scene->setRgbPrincipalPoint(3.2850951551345941e+02, 2.5282555217253503e+02);
    scene->setRgbFocalLength(5.1930334103339817e+02, 5.1816401430246583e+02);

    scene->setDepthToRgbRotation(Ogre::Matrix3( 9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03,
                                               -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03,
                                                4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01));
    scene->setDepthToRgbTranslation(2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03);

    scene->meshify();

    previewWindow->setScene(scene);

    previewWindow->enterRenderingLoop();

    // Clean up
    delete scene;
    delete previewWindow;

    return 0;
}
