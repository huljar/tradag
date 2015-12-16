#include <TraDaG/tradagmain.h>
#include <TraDaG/ogrewindow.h>
#include <TraDaG/rgbdobject.h>
#include <TraDaG/util.h>

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
            ("label-name,n", po::value<std::string>(), "Label name")
            ("show-preview,p", "Display a preview window")
            ("animate,a", "Show the object falling into the scene")
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

    // Get label map
    LabelMap labelMap = Labels::NyuDepthV1;

    // Get camera data
    cv::Vec2f depthPrincipalPoint(3.2442516903961865e+02, 2.3584766381177013e+02);
    cv::Vec2f depthFocalLength(5.7616540758591043e+02, 5.7375619782082447e+02);

    // Get additional parameters
    std::string meshName = vm["mesh-name"].as<std::string>();
    std::string labelName = vm["label-name"].as<std::string>();

    bool preview = vm.count("show-preview");
    bool animate = vm.count("animate");

    // From here, let TradagMain take over
    TradagMain tradag(depthFile, rgbFile, labelFile, labelMap, depthPrincipalPoint, depthFocalLength);
    tradag.setShowPreviewWindow(preview);
    tradag.setShowPhysicsAnimation(animate);
    tradag.setDebugMarkInlierSet(true);
    //tradag.setDebugDrawBulletShapes(true);
    tradag.setObjectMustBeUpright(true);
    tradag.setGravity(-40, -90, -5);
    tradag.dropObjectIntoScene(meshName, labelName, Auto<cv::Vec3f>(true), Auto<cv::Matx33f>(true), M_PI_2,
                               cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0));

    return 0;
}
