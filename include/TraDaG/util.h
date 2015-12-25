#ifndef UTIL_H
#define UTIL_H

#include <Ogre.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <map>
#include <vector>

namespace TraDaG {

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
        UA_RETRY,
        UA_ABORT
    } UserAction;

    typedef enum {
        OD_SUCCESS,
        OD_PLANE_TOO_STEEP,
        OD_UNKNOWN_ERROR
    } ObjectDropStatus;

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

    struct PlaneFitResult {
        PlaneFitResult(PlaneFitStatus status, const Ogre::Plane& planeFound = Ogre::Plane(),
                           const std::vector<Ogre::Vector3>& planeVertices = std::vector<Ogre::Vector3>())
            : status(status), plane(planeFound), vertices(planeVertices)
        {
        }

        const PlaneFitStatus status;
        const Ogre::Plane plane;
        const std::vector<Ogre::Vector3> vertices;
    };

    struct ObjectDropResult {
        ObjectDropResult(ObjectDropStatus status, const cv::Mat& image, float covered, const cv::Matx33f& rot, const cv::Vec3f& trans)
            : status(status), renderedImage(image), fractionCovered(covered), rotation(rot), translation(trans)
        {
        }

        const ObjectDropStatus status;
        const cv::Mat renderedImage;
        const float fractionCovered;
        const cv::Matx33f rotation;
        const cv::Vec3f translation;
    };

    namespace Strings {
        const std::string ResourcesCfgPath = "../config/resources.cfg";
        const std::string PluginsCfgPath = "../config/plugins.cfg";

        const std::string ObjName = "droppingObject";
        const std::string RgbdSceneName = "sceneRgbdEntity";
        const std::string ObjRigidBodyName = "objectBody";
        const std::string PlaneRigidBodyName = "planeBody";

        const std::string PreviewWindowName = "Training Data Generator Preview Window";
        const std::string RenderWindowName = "Training Data Generator Render Window";
    }

    namespace Defaults {
        const bool ObjectMustBeUpright = false;
        const Auto<float> ObjectCoveredFraction(true);
        const bool ObjectCastShadows = true;
        const unsigned int MaxAttempts = 100;
        const bool ShowPreviewWindow = false;
        const bool ShowPhysicsAnimation = false;
        const bool MarkInlierSet = false;
        const bool DrawBulletShapes = false;
        const Auto<cv::Vec3f> Gravity(true, cv::Vec3f(0, -9810, 0));
        const float ObjectRestitution = 0.4;
        const float ObjectFriction = 0.7;
        const float PlaneRestitution = 0.1;
        const float PlaneFriction = 0.9;
        const float ObjectScale = 1000.0;
    }

    namespace Constants {
        const float ObjectDropDistance = 500.0;
        const Ogre::Degree MaxPlaneNormalToGravityAngle(25);
        const float RansacConfidenceInterval = 18.0;
        const float IdleTimeThreshold = 1.0; // in seconds
        const float TimeoutTimeThreshold = 12.0; // in seconds

        const Ogre::Vector3 DefaultCameraPosition = Ogre::Vector3::ZERO;
        const Ogre::Vector3 DefaultCameraLookAt = Ogre::Vector3::NEGATIVE_UNIT_Z;
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

}

#endif // UTIL_H

