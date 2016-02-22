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
 * Below is a list of dependencies which are required to be installed on the system to compile and run
 * TraDaG or other projects working with the TraDaG framework. The instructions in the <em>Installing
 * from a repository</em> sections below are confirmed to work with Debian and Ubuntu, but for other
 * Linux distributions and different package management systems, similar packages might exist.
 *
 * @subsection ogre_sec OGRE
 * OGRE (short for <em>Object-oriented Graphics Rendering Engine</em>) is the underlying rendering
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
 * OIS (short for <em>Object-oriented Input System</em>) is the library used to process mouse and
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
 * Bullet is the physics engine that is responsible for dropping objects into the scenes by
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
 * OgreBullet is a thin wrapper library for Bullet to integrate with OGRE and make Bullet's functions
 * callable with OGRE types.
 *
 * @subsubsection ogrebullet_precomp_sec Using the precompiled binaries
 * - TraDaG provides binaries compiled for i386 in the @a libs/OgreBullet/i386 directory and for x86_64 in
 *   @a libs/OgreBullet/x86_64.
 * - You need to make sure that the linker is able to find @a libOgreBulletCollisions.so and
 *   @a libOgreBulletDynamics.so, which are located in the above directories.
 *
 * @subsubsection ogrebullet_source_sec Compiling from source
 * - Go to the <a href="https://bitbucket.org/alexeyknyshev/ogrebullet">BitBucket repository</a> of
 *   OgreBullet
 * - Grab a copy of the source code (e.g. by cloning the repository with @c git)
 * - Follow the instructions in the @a OgreBullet_readme.txt file
 * - It may happen that an error similar to the following occurs during compilation: <tt>error: prototype
 *   for 'OgreBulletCollisions::StaticMeshToShapeConverter::StaticMeshToShapeConverter(const Ogre::Entity*,
 *   const Ogre::Matrix4&)' does not match any in class
 *   'OgreBulletCollisions::StaticMeshToShapeConverter'</tt>
 *   - If this is the case, open the file
 *     @a Collisions/include/Utils/OgreBulletCollisionsMeshToShapeConverter.h and go to line 100
 *   - Change \"<tt>Ogre::Entity* entity</tt>\" to \"<tt>const Ogre::Entity* entity</tt>\"
 *
 * @subsection opencv_sec OpenCV
 * OpenCV (short for <em>Open Computer Vision</em>) is a large, generic library that can be used for all
 * kinds of tasks related to computer vision. In this framework, it is mainly used for image manipulation
 * and loading/storing those images from/to files. TraDaG was created and tested with OpenCV 2.4.9.
 *
 * @subsubsection opencv_source_sec Compiling from source
 * - Go to the <a href="http://opencv.org/downloads.html">OpenCV downloads page</a>
 * - Download a version appropriate for your operating system (perferably from the 2.4.x branch)
 * - Follow the instructions
 *   <a href="http://docs.opencv.org/2.4/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html">here</a>
 *   to set up OpenCV
 *
 * @subsubsection opencv_repo_sec Installing from a repository (Debian, Ubuntu)
 * - Install the package @a libopencv-dev and its dependencies (e.g. by executing \"<tt>apt-get install
 *   libopencv-dev</tt>\" as root)
 * - If you don't want to install all components of OpenCV, it should be sufficient to only install the
 *   required modules @a libopencv-core-dev, @a libopencv-imgproc-dev and @a libopencv-highgui-dev
 *
 * @subsection boost_sec Boost
 * Boost is one of the most popular C++ libraries providing a lot of very useful functions. TraDaG was
 * created and tested with Boost 1.55/1.58.
 *
 * @subsubsection boost_source_sec Compiling from source
 * - Download Boost from the <a href="http://www.boost.org/users/download/">official download page</a>
 * - Follow the instructions in the @a INSTALL and @a index.html files
 *
 * @subsubsection boost_repo_sec Installing from a repository (Debian, Ubuntu)
 * - Install the package @a libboost-all-dev and its dependencies (e.g. by executing \"<tt>apt-get install
 *   libboost-all-dev</tt>\" as root)
 * - If you don't want to install all components of Boost, it should be sufficient to only install the
 *   required modules @a libboost-system-dev, @a libboost-filesystem-dev and @a libboost-regex-dev
 *
 * @section features_sec Features
 *
 * - Reading in whole data sets of labeled RGB-D scenes or using just a single specified scene
 * - Searching for scenes by ground plane label, orientation and distance
 * - Robust finding and fitting of planes into scenes using a RANSAC-based approach
 * - Storing planes to disk in a textual format
 * - Automatically parsing stored planes first before trying to compute a new plane
 * - Rendering an RGB-D scene in 3D
 * - Creating and registering one or more objects (of the same or different types) to drop into a scene
 * - Executing a physics simulation to plausibly drop these objects onto a plane in the scene
 * - Optional preview of the result (after the simulation) with a controllable CameraManager
 * - Selection whether to keep/discard the result or to restart/abort the simulation when having the
 *   preview window enabled
 * - Optional animation of the physics simulation in real-time
 * - Control over the way objects are initially dropped (speed, torque, gravity, ...)
 * - Specifying constraints for a \"good\" result on a per-object level, like desired occlusion or
 *   distance
 * - Automatic restarts of the simulation if the result is not within the specified constraints, up to a
 *   user-defined limit
 * - Optional preview of the plane vertices that were computed to be inliers by RANSAC
 * - Optional preview of the objects' collision shapes used by the physics engine
 * - Detailed information about the state of the objects after the simulation, including 6D pose, object
 *   coordinates per pixel, occluded fraction and an occlusion flag for each object pixel
 *
 * This list of features is not exhaustive, but should give a good overview about the capabilities of the
 * framework. For more in-depth and detailed information, please refer to the API reference of this
 * documentation.
 *
 * @section getting_started_sec Getting Started
 *
 * @subsection getting_started_wrapper_sec The Wrapper Way
 * The \ref TraDaG::CVLDWrapper "CVLDWrapper" class provides a comfortable interface for the most common
 * use cases of the CVLD members. Simply construct an object of the class, use the various setter
 * functions to define your parameters, and start generating scenes.
 *
 * To construct the wrapper object, you need to provide the absolute or relative path to your data set.
 * This directory is expected to contain the subdirectories @a depth, @a rgb and @a label, which contain
 * the depth, color and label images, respectively. For %TraDaG to know which depth, color and label images
 * belong together, they need to have the same file name, e.g. the files @a depth/myscene.png,
 * @a rgb/myscene.png and @a label/myscene.png will be interpreted to represent the same scene and will be
 * used together. Files without an equivalent in both of the other directories will not be considered as
 * a valid scene and will be ignored.
 *
 * If you want to make use of precomputing planes and storing them to disk or to read existing planes from
 * disk, the data set directory also needs to contain a @a plane subdirectory. This is not required, though;
 * if it does not exist, you just will not be able to make use of plane precomputation through this wrapper.
 *
 * In addition, the wrapper requires a \ref TraDaG::CameraManager "CameraManager" instance. This is a very
 * simple class which holds all the camera parameters that were used to record the data set. %TraDaG needs
 * to know these parameters to accurately reconstruct the 3D scene from the point of view of the recording
 * cameras. To construct a \ref TraDaG::CameraManager "CameraManager" instance, you need to pass the camera
 * parameters to its constructor. For more information, refer to its
 * \ref TraDaG::CameraManager "API documentation".
 *
 * The wrapper also requires a label map. This is a \ref TraDaG::LabelMap "mapping" from strings to grey
 * values with 16 bit depth, i.e. in the [0, 65535] interval. These grey values are the values stored in the
 * label images to represent the different labels. For the <em>NYU Depth V1</em> and <em>NYU Depth V2</em>
 * data sets, sensible label maps are already predefined in util.h. If you include this header, you can use
 * the \ref TraDaG::Labels::NYUDepthV1 "NYUDepthV1" and
 * \ref TraDaG::Labels::NYUDepthV2 "NYUDepthV2" constants, if you wish.
 *
 * The last parameter is to limit the amount of scenes that will be processed. It is mostly there to have
 * some kind of an upper bound on the computation time when searching through all the scenes, and is most
 * useful when working with huge data sets. In most cases, it should be okay to leave this at @a unlimited.
 * For more information, see the CVLDWrapper::CVLDWrapper API documentation.
 *
 * When you have successfully constructed a wrapper object, you have access to various setters to define
 * the parameters of your simulations. Most values are set to a reasonable default, but you may change them
 * as you wish (again, refer to the CVLDWrapper API documentation for more details). To start generating
 * training images, you must define which labels you want to use, however; you can do this using the
 * \ref TraDaG::CVLDWrapper::labelsToUse "labelsToUse" and
 * \ref TraDaG::CVLDWrapper::setLabelsToUse "setLabelsToUse" functions. You must define at least one label.
 *
 * You probably also want to set the active object, i.e. which object will be dropped into the scene. You
 * can do this using the \ref TraDaG::CVLDWrapper::setActiveObject "setActiveObject" function, which takes
 * a numeric object ID as a parameter (it is also possible to use multiple objects at the same time, but not
 * through this wrapper). The object IDs are 1-based and assigned contiguously in the
 * [1, \ref TraDaG::CVLDWrapper::getNumObjects "getNumObjects"] interval. By default, the 13 objects from
 * the @a Hinterstoisser data set are available. If you want to add new objects or remove some, you can
 * use the \ref TraDaG::CVLDWrapper::availableObjects "availableObjects" vector. Keep in mind that making
 * this vector smaller also invalidates some object IDs.
 *
 * Now you are ready to generate some training images! There are three methods of doing so through the wrapper,
 * which are defined through the overloads of the \ref TraDaG::CVLDWrapper::getTrainingImage "getTrainingImage"
 * function, providing different levels of control over the object pose and the selected scenes. They all have
 * in common that they will select a random scene (which matches your specifications), fit a plane into the
 * scene using one of the \ref TraDaG::CVLDWrapper::labelsToUse "allowed labels" (or use a precomputed plane,
 * if available), and drop the object onto the plane (according to the restrictions passed to the function).
 * The simulation is preformed up the the \ref TraDaG::CVLDWrapper::getMaxAttempts "maximum attempts" per scene.
 * If no good result is produced, it will move on to the next (randomly selected) scene. If all scenes fail to
 * produce a good result, the overall best attempt will be returned, and a flag will indicate that the result is
 * not optimal.
 *
 * The \ref TraDaG::CVLDWrapper::getTrainingImage(double, double) "simplest overload" only requires you to
 * specify what occlusion you will accept as a \"good\" result. No restrictions on the scene (aside from the
 * allowed labels) or on the object pose are enforced.
 *
 * The
 * \ref TraDaG::CVLDWrapper::getTrainingImage(const cv::Mat_<double>&, const cv::Point3d&, double, double) "next overload"
 * does not restrict the scene, either, but allows you to define the initial rotation of the object and an
 * initial velocity that the object will have when it \"appears\" in the scene. The final object pose is not
 * affected by this.
 *
 * The
 * \ref TraDaG::CVLDWrapper::getTrainingImage(const cv::Mat_<double>&, double, unsigned short, unsigned short, double, double) "final overload"
 * is the most restrictive of the three. It allows you to define the final pose of the object by specifying its
 * final rotation (with a tolerance) and its distance to the camera, in addition to the usual occlusion. The
 * scenes are also restricted by the final object rotation, because only scenes containing planes with a normal
 * that matches the object's up-vector (within the given tolerance) and a distance within the specified interval
 * will be selected.
 *
 * All three overloads return a \ref TraDaG::CVLDWrapper::TrainingImage "TrainingImage" containing the rendered
 * scene with the object and additional information, such as object coordinates and occlusion. In addition, a
 * \ref TraDaG::Simulator::DropStatus "status" is returned to indicate success and failure, amongst others.
 *
 * @subsubsection getting_started_wrapper_precomputing_sec Precomputing planes
 *
 * As mentioned a few times \ref getting_started_wrapper_sec "above", you can precompute planes for your scenes
 * and store them to files on the hard drive. The wrapper provides this functionality in the
 * \ref TraDaG::CVLDWrapper::precomputePlaneInfo "precomputePlaneInfo" function. You must pass one or more labels
 * (and optionally a plane normal with a tolerance) to this function; it will then search the data set for
 * matching scenes and try to fit a plane for each scene and each label (that is contained in the current scene).
 * The computed planes will be stored in <tt>.planeinfo</tt> files in the @a plane subdirectory of the data set path
 * that was specified in the \ref TraDaG::CVLDWrapper::CVLDWrapper "constructor". If this subdirectory does not
 * exist, the computation will fail.
 *
 * When precomputed planes are available at the time of \ref TraDaG::CVLDWrapper::getTrainingImage "requesting" a
 * training image, they will always be parsed first before attempting to fit a new plane (if the saved ones do
 * not match the current requirements). If you want you simulation to only rely on the precomputed planes and
 * discard a scene if those planes are not adequate (e.g. to save time if you know that you precomputed all the
 * relevant planes), you can set the \ref TraDaG::CVLDWrapper::setComputePlaneIfNoFile "parameter" for this to
 * @c false.
 *
 * @subsection getting_started_generic_sec The Generic Way
 * If you decided to not use the wrapper, you have a lot more and powerful tools at your disposal; however, you will
 * probably need to consult the API documentation a lot more, too.
 *
 * The most important classes here are
 * - \ref TraDaG::SceneAnalyzer "SceneAnalyzer"
 * - \ref TraDaG::Simulator "Simulator"
 * - \ref TraDaG::DroppableObject "DroppableObject"
 * - \ref TraDaG::GroundPlane "GroundPlane"
 *
 * @subsubsection sceneanalyzer_sec SceneAnalyzer
 * The \ref TraDaG::SceneAnalyzer "SceneAnalyzer" class manages your data set and allows you to search for scenes
 * by labels or by planes (see e.g. \ref TraDaG::SceneAnalyzer::beginByLabel "beginByLabel" and
 * \ref TraDaG::SceneAnalyzer::beginByPlane "beginByPlane"). It also allows you to iterate over the search results.
 * It has functions for precomputing planes similar to \ref getting_started_wrapper_precomputing_sec "the wrapper"
 * and will also parse saved plane files before attempting to fit a new plane into a scene (which is obvious if you
 * consider that the wrapper uses this class for its search operations).
 *
 * \ref TraDaG::SceneAnalyzer "SceneAnalyzer" assigns an ID to all the scenes it manages, in alphabetical order of
 * the file name of the depth image belonging to the scene. When searching for planes, the result will always contain
 * this scene ID, which can then be used with the other functions. For example, when you have found a scene that you
 * want to use for a simulation, you can call \ref TraDaG::SceneAnalyzer::createSimulator(unsigned int) "createSimulator"
 * with the scene ID to comfortably construct a \ref TraDaG::Simulator "Simulator" for this scene. If you have the
 * plane available at this time, you can even pass it directly to
 * \ref TraDaG::SceneAnalyzer::createSimulator(unsigned int, const GroundPlane&) "createSimulator" and it will be
 * registered automatically (alternatively, you can use \ref TraDaG::Simulator::setGroundPlane "setGroundPlane" on
 * the \ref TraDaG::Simulator "Simulator" instance to specify the plane after construction).
 *
 * @subsubsection simulator_sec Simulator
 *
 * This class manages the simulation of dropping objects for a single scene. It facilitates the creation of one or
 * more \ref TraDaG::DroppableObject "DroppableObject"s to use for the simulation and allows setting parameters
 * like the \ref TraDaG::Simulator::setGravity "gravity vector" or the
 * \ref TraDaG::Simulator::setMaxAttempts "maximum attempts". Object-specific parameters like
 * \ref TraDaG::DroppableObject::setDesiredOcclusion "desired occlusion" can be set directly
 * on the objects returned by \ref TraDaG::Simulator::createObject "createObject". When you are ready to start the
 * simulation, call \ref TraDaG::Simulator::execute "execute". This function will run the simulation and repeat it
 * up to \ref TraDaG::Simulator::getMaxAttempts "max attempts" times, or until an optimal result (i.e. a result
 * where all restrictions are met) is found. In the case that no optimal result can be computed within the
 * \ref TraDaG::Simulator::getMaxAttempts "max attempts", the best one found will be returned. The result also
 * contains a \ref TraDaG::Simulator::DropStatus "status" to indicate success and failure, amongst others.
 * The parameters of the dropped objects after the simulation can be retrieved from the
 * \ref TraDaG::DroppableObject "DroppableObject"s themselves using
 * \ref TraDaG::DroppableObject::getFinalOcclusion "getFinalOcclusion",
 * \ref TraDaG::DroppableObject::getFinalRotation "getFinalRotation" etc. (see the
 * \ref TraDaG::DroppableObject "API documentation" for a complete list).
 *
 * @subsubsection droppableobject_sec DroppableObject
 *
 * @subsubsection groundplane_sec GroundPlane
 *
 * @subsection ogre_mesh_sec OGRE .mesh files
 *
 * @subsubsection new_object_sec Registering a new object
 *
 * @section attribution_sec Attribution
 *
 * %TraDaG was written in 2015/2016 by Julian Harttung for the <a href="http://cvlab-dresden.de/">Computer
 * Vision Lab Dresden</a> (CVLD). It is part of a research project on 6D pose estimation at the
 * <a href="https://www.inf.tu-dresden.de/portal.php?node_id=1&ln=en&group=13">Faculty of Computer
 * Science</a> of the <a href="http://tu-dresden.de/en">Dresden University of Technology</a>.
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
