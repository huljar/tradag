/************************************************************//**
 * @file
 *
 * @brief Header containing typedefs, constants, defaults and
 * small structs that are needed in many different places.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef UTIL_H
#define UTIL_H

#include <OGRE/OgreMath.h>
#include <OGRE/OgreVector3.h>

#include <opencv2/core/core.hpp>

#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

/// The %TraDaG namespace contains everything belonging to the framework.
namespace TraDaG {

    class DroppableObject; // forward declaration

    /// Vector of pointers to DroppableObject instances.
    typedef std::vector<DroppableObject*> ObjectVec;
    /// Vector of label values as they are defined in the label images.
    typedef std::vector<unsigned short> LabelVec;
    /// Mapping of string labels to label values.
    typedef std::map<std::string, LabelVec> LabelMap;

    /**
     * @brief Simple struct that is mostly used for parameters that can be calculated automatically.
     *
     * When this struct is used to set a parameter, it indicates if the value should be calculated
     * automatically, and if not, it contains the explicit value that will be used.
     */
    template<typename T>
    struct Auto {
        /**
         * @brief Simple constructor.
         * @tparam T Type of the represented parameter.
         * @param automate Flag indicating whether the represented parameter should be automatically calculated.
         * @param manualValue Explicit value of the parameter. Specify this only if @c automate is @c false.
         */
        Auto(bool automate, const T& manualValue = T())
            : automate(automate), manualValue(manualValue)
        {
        }

        /**
         * Flag indicating whether the represented parameter should be automatically calculated.
         */
        bool automate;
        /**
         * @brief Explicit value of the parameter, in the case that it is not automatically calculated, i.e.
         * @c automate is set to @c false.
         */
        T manualValue;
    };

    /// Namespace containing string constants used throughout the framework.
    namespace Strings {
        const std::string LogfilePath = "Ogre.log";
        const std::string ResourcesCfgPath = "../config/resources.cfg";
        const std::string PluginsCfgPath = "../config/plugins.cfg";

        const std::string ObjRigidBodyName = "objectBody";
        const std::string PlaneRigidBodyName = "planeBody";

        const std::string VertexRigidBodyName = "vertexBody";

        const std::string PreviewWindowName = "Training Data Generator Preview Window";
        const std::string RenderWindowName = "Training Data Generator Render Window";

        const std::string StandardMaterialName = "BaseWhiteNoLighting";
        const std::string DepthMapMaterialName = "TraDaG/DepthMap";
        const std::string InfoTrayMaterialName = "TraDaG/InfoTray";

        const std::string UserInputPromptOverlayName = "InputPromptOverlay";
        const std::string UserInputPromptPanelName = "InputPromptPanel";
        const std::string UserInputPromptTextAreaName = "InputPromptTextArea";
        const std::string UserInputPromptFontName = "TraDaG/Inputs";

        const std::string UserInputPromptText = "Select an action:\n\nEnter - Accept this result\nBackspace - Discard this result\nr - Restart with same parameters\nEsc - Abort";

        const std::string FileNamePatternPlaneInfo = "%s-%n";
        const std::string FileExtensionPlaneInfo = ".planeinfo";
        const std::string FileExtensionPlane = ".plane";

    }

    /// Namespace containing default values for many parameters.
    namespace Defaults {
        const std::pair<float, float> ObjectDesiredOcclusion(0.0, 0.95);
        const std::pair<unsigned short, unsigned short> ObjectDesiredDistance(0, std::numeric_limits<unsigned short>::max());
        const Auto<cv::Vec3f> ObjectInitialPosition(true);
        const Auto<cv::Matx33f> ObjectInitialRotation(true);
        const float ObjectInitialAzimuth = 0.0; // in degrees
        const cv::Vec3f ObjectInitialVelocity(0, 0, 0);
        const cv::Vec3f ObjectInitialTorque(0, 0, 0);
        const bool ObjectMustBeUpright = false;
        const bool ObjectCastShadows = true;
        const float ObjectRestitution = 0.4;
        const float ObjectFriction = 0.7;
        const float ObjectMass = 1.0;
        const cv::Vec3f ObjectScale(1000, 1000, 1000);
        const float ObjectScoreWeight = 1.0;

        const float PlaneRestitution = 0.1;
        const float PlaneFriction = 0.9;

        const unsigned int MaxAttempts = 100;
        const bool ShowPreviewWindow = false;
        const bool ShowPhysicsAnimation = false;
        const bool MarkInlierSet = false;
        const bool DrawBulletShapes = false;
        const Auto<cv::Vec3f> Gravity(true, cv::Vec3f(0, -9810, 0));

        const size_t MaxCacheScenes = 100;
    }

    /// Namespace containing constants used throughout the framework.
    namespace Constants {
        const float ObjectDropDistance = 500.0;
        const Ogre::Degree MaxPlaneNormalToGravityAngle(25);
        const float MaxObjectToPlaneDistanceSquared = 90.0 * 90.0; // in mm

        const unsigned int MinLabelPixelsToBeValid = 10;
        const unsigned int MinRegionPixelsToBeValid = 10;

        const float RansacConfidenceInterval = 18.0;

        const float IdleTimeThreshold = 1.0; // in seconds
        const float TimeoutTimeThreshold = 12.0; // in seconds

        const float ScoreOcclusionWeight = 150.0;
        const float ScoreDistanceWeight = 0.1;

        const unsigned int PreviewWindowWidth = 1024;
        const unsigned int PreviewWindowHeight = 768;

        const Ogre::Vector3 DefaultCameraPosition = Ogre::Vector3::ZERO;
        const Ogre::Vector3 DefaultCameraLookAt = Ogre::Vector3::NEGATIVE_UNIT_Z;
        const Ogre::Real CameraNearClipDistance = 5.0;

        const unsigned short WorkPlaneDepth = 50;

        const int KDTreeMaxDepth = 10;
        const int KDTreeMinSize = 10;
    }

    /// Namespace containing some predefined label map constants.
    namespace Labels {
        /**
         * Predefined label map for the NYU Depth V1 data set.
         * @remarks This map is not necessarily exhaustive, but should contain the most useful labels.
         */
        const LabelMap NYUDepthV1({
            {"floor", {
                423,  // 'floor'
                424,  // 'floor mat'
                425,  // 'floor tiles'
                426,  // 'floors'
                489,  // 'ground'
                1386  // 'wood flooring'
            }},
            {"mat", {
                424,  // 'floor mat'
                671,  // 'mat'
                1416, // 'yoga mat'
                1417  // 'yoga mats'
            }},
            {"table", {
                281,  // 'counter  table' [sic!]
                282,  // 'counter table'
                283,  // 'counter table with rows'
                304,  // 'cutting table'
                331,  // 'display desk'
                333,  // 'display table'
                361,  // 'dressing table'
                556,  // 'ironing table'
                830,  // 'pink pong table' [sic!]
                906,  // 'reception desk'
                1122, // 'table'
                1129  // 'table sheet'
            }}
        });

        /**
         * Predefined label map for the NYU Depth V2 data set.
         * @remarks This map is not necessarily exhaustive, but should contain the most useful labels.
         */
        const LabelMap NYUDepthV2({
            {"floor", {
                11,   // 'floor'
                143,  // 'floor mat'
                868   // 'floor trim'
            }},
            {"mat", {
                143,  // 'floor mat'
                205,  // 'yoga mat'
                473   // 'desk mat'
            }},
            {"mattress", {
                576   // 'mattress'
            }},
            {"table", {
                19,   // 'table'
                36,   // 'desk'
                292,  // 'tablecloth'
                356,  // 'coffee table'
                375,  // 'table runner'
                429,  // 'game table'
                473,  // 'desk mat'
                510,  // 'foosball table' [sic!]
                515,  // 'pool table'
                526,  // 'toy table'
                625   // 'ping pong table'
            }}
        });
    }

}

#endif // UTIL_H

