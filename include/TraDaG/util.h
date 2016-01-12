#ifndef UTIL_H
#define UTIL_H

#include <Ogre.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <map>
#include <vector>

namespace TraDaG {

    class DroppableObject; // forward declaration

    typedef enum {
        MM_MAPPED_RGB_TO_DEPTH,
        MM_MAPPED_DEPTH_TO_RGB,
        MM_UNMAPPED_RGB_TO_DEPTH,
        MM_UNMAPPED_DEPTH_TO_RGB
    } MapMode;

    typedef enum {
        LM_DEPTH_IMAGE,
        LM_RGB_IMAGE
    } LabelMode;

    typedef enum {
        PF_SUCCESS,
        PF_DIFFERENT_DIMENSIONS,
        PF_INVALID_LABEL,
        PF_LABEL_NOT_IN_IMAGE
    } PlaneFitStatus;

    typedef enum {
        SR_SUCCESS,
        SR_ABORTED,
        SR_TIMEOUT
    } SimulationResult;

    typedef enum {
        UA_KEEP,
        UA_RESTART,
        UA_ABORT
    } UserAction;

    typedef enum {
        OD_SUCCESS,
        OD_PLANE_TOO_STEEP,
        OD_MAX_ATTEMPTS_REACHED,
        OD_USER_ABORTED,
        OD_UNKNOWN_ERROR
    } ObjectDropStatus;

    typedef std::vector<DroppableObject*> ObjectVec;
    typedef std::vector<std::pair<cv::Point, bool>> PixelInfoVec;
    typedef std::vector<unsigned short> LabelVec;
    typedef std::map<std::string, LabelVec> LabelMap;

    template<typename T>
    struct Auto {
        Auto(bool automate, const T& manualValue = T())
            : automate(automate), manualValue(manualValue)
        {
        }

        bool automate;
        T manualValue;
    };

    struct ObjectDropResult {
        ObjectDropResult(ObjectDropStatus status, const cv::Mat& depthImage, const cv::Mat& rgbImage)
            : status(status), depthImage(depthImage), rgbImage(rgbImage)
        {
        }

        const ObjectDropStatus status;
        const cv::Mat depthImage;
        const cv::Mat rgbImage;
    };

    namespace Strings {
        const std::string ResourcesCfgPath = "../config/resources.cfg";
        const std::string PluginsCfgPath = "../config/plugins.cfg";

        const std::string RgbdSceneName = "rgbdSceneEntity";
        const std::string ObjRigidBodyName = "objectBody";
        const std::string PlaneRigidBodyName = "planeBody";

        const std::string VertexRigidBodyName = "vertexBody";

        const std::string PreviewWindowName = "Training Data Generator Preview Window";
        const std::string RenderWindowName = "Training Data Generator Render Window";

        const std::string DepthMapMaterialName = "TraDaG/DepthMap";
        const std::string InfoTrayMaterialName = "TraDaG/InfoTray";

        const std::string UserInputPromptOverlayName = "InputPromptOverlay";
        const std::string UserInputPromptPanelName = "InputPromptPanel";
        const std::string UserInputPromptTextAreaName = "InputPromptTextArea";
        const std::string UserInputPromptFontName = "TraDaG/Inputs";

        const std::string UserInputPromptText = "Select an action:\n\nEnter - Accept this result\nr - Restart\nEsc - Abort";
    }

    namespace Defaults {
        const bool ObjectMustBeUpright = false;
        const std::pair<float, float> ObjectDesiredOcclusion(0.0, 0.95);
        const bool ObjectCastShadows = true;
        const unsigned int MaxAttempts = 100;
        const bool ShowPreviewWindow = false;
        const bool ShowPhysicsAnimation = false;
        const bool MarkInlierSet = false;
        const bool DrawBulletShapes = false;
        const Auto<cv::Vec3f> ObjectInitialPosition(true);
        const Auto<cv::Matx33f> ObjectInitialRotation(true);
        const float ObjectInitialAzimuth = 0.0;
        const cv::Vec3f ObjectInitialVelocity(0, 0, 0);
        const cv::Vec3f ObjectInitialTorque(0, 0, 0);
        const Auto<cv::Vec3f> Gravity(true, cv::Vec3f(0, -9810, 0));
        const float ObjectRestitution = 0.4;
        const float ObjectFriction = 0.7;
        const float ObjectMass = 1.0;
        const float PlaneRestitution = 0.1;
        const float PlaneFriction = 0.9;
        const cv::Vec3f ObjectScale(1000, 1000, 1000);
    }

    namespace Constants {
        const float ObjectDropDistance = 500.0;
        const Ogre::Degree MaxPlaneNormalToGravityAngle(25);
        const float RansacConfidenceInterval = 18.0;
        const float IdleTimeThreshold = 1.0; // in seconds
        const float TimeoutTimeThreshold = 12.0; // in seconds

        const Ogre::Vector3 DefaultCameraPosition = Ogre::Vector3::ZERO;
        const Ogre::Vector3 DefaultCameraLookAt = Ogre::Vector3::NEGATIVE_UNIT_Z;

        const float WorkPlaneDepth = 50.0;
    }

    namespace Labels {
        const LabelMap NyuDepthV1({
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

        const LabelMap NyuDepthV2({
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

    Ogre::Matrix3 convertCvMatToOgreMat(const cv::Matx33f& mat);
    cv::Matx33f convertOgreMatToCvMat(const Ogre::Matrix3& mat);

}

#endif // UTIL_H

