#include <TraDaG/tradagmain.h>
#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>

#include <opencv2/core/core.hpp>

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
            ("depth-file,d", po::value<std::string>(), "Depth input file")
            ("rgb-file,c", po::value<std::string>(), "RGB (color) input file")
            ("label-file,l", po::value<std::string>(), "Label input file")
            ("mesh-name,m", po::value<std::string>(), "Object mesh name")
            ("help", "Display this help and exit")
            ("version", "Display version information and exit");

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
    std::string depthFile = vm["depth-file"].as<std::string>();
    std::string rgbFile = vm["rgb-file"].as<std::string>();
    std::string labelFile = vm["label-file"].as<std::string>();

    // Get camera data
    cv::Vec2f depthPrincipalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
    cv::Vec2f depthFocalLength(5.7616540758591043e+02, 5.7375619782082447e+02);

    // Get mesh name
    std::string meshName = vm["mesh-name"].as<std::string>();

    // From here, let TradagMain take over
    TradagMain tradag(depthFile, rgbFile, labelFile, depthPrincipalPoint, depthFocalLength);
    tradag.dropObjectIntoScene(meshName, 0, false, Auto<float>(true), true, 20, true, true, -981 * Ogre::Vector3::UNIT_Y,
                               Auto<Ogre::Vector3>(true), Auto<Ogre::Matrix3>(true), Ogre::Vector3(200, 200, 0));

    return 0;
}
