#ifndef UTIL_H
#define UTIL_H

#include <Ogre.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <map>
#include <vector>

namespace TraDaG {

    typedef enum {
        MAPPED_RGB_TO_DEPTH,
        MAPPED_DEPTH_TO_RGB,
        UNMAPPED_RGB_TO_DEPTH,
        UNMAPPED_DEPTH_TO_RGB
    } MapMode;

    typedef enum {
        LABELS_ON_DEPTH_IMAGE,
        LABELS_ON_RGB_IMAGE
    } LabelMode;

    typedef enum {
        SUCCESS_FIT,
        DEPTH_LABEL_DIFFERENT_DIMENSIONS,
        INVALID_LABEL,
        LABEL_NOT_IN_IMAGE
    } PlaneFittingResultStatus;

    typedef enum {
        SUCCESS_DROP
    } ObjectDropResultStatus;

    typedef std::vector<unsigned short> LabelVec;
    typedef std::map<std::string, LabelVec> LabelMap;

    template<class T>
    struct Auto {
        Auto(bool automate, const T& manualValue = T()) : automate(automate), manualValue(manualValue) {}

        bool automate;
        T manualValue;
    };

    struct PlaneFittingResult {
        PlaneFittingResult(PlaneFittingResultStatus res, const Ogre::Plane& planeFound = Ogre::Plane())
            : result(res), plane(planeFound)
        {
        }

        const PlaneFittingResultStatus result;
        const Ogre::Plane plane;
    };

    struct ObjectDropResult {
        ObjectDropResult(ObjectDropResultStatus res, const cv::Mat& image, float covered, const cv::Matx33f& rot, const cv::Vec3f& trans)
            : result(res), renderedImage(image), fractionCovered(covered), rotation(rot), translation(trans)
        {
        }

        const ObjectDropResultStatus result;
        const cv::Mat renderedImage;
        const float fractionCovered;
        const cv::Matx33f rotation;
        const cv::Vec3f translation;
    };

    namespace Strings {
        const std::string ResourcesCfgPath = "../config/resources.cfg";
        const std::string PluginsCfgPath = "../config/plugins.cfg";

        const std::string RgbdSceneName = "sceneRgbdEntity";
        const std::string ObjRigidBodyName = "objectBody";
        const std::string PlaneRigidBodyName = "planeBody";

        const std::string RenderWindowName = "Training Data Generator Preview Window";
    }

    namespace Defaults {
        const bool ObjectMustBeUpright = false;
        const Auto<float> ObjectCoveredFraction = Auto<float>(true);
        const bool ObjectCastShadows = true;
        const unsigned int MaxAttempts = 20;
        const bool ShowPreviewWindow = false;
        const bool ShowPhysicsAnimation = false;
        const cv::Vec3f Gravity = cv::Vec3f(0, -9810, 0);
        const float ObjectRestitution = 0.4;
        const float ObjectFriction = 0.7;
        const float PlaneRestitution = 0.1;
        const float PlaneFriction = 0.9;
        const float ObjectScale = 1000.0;
    }

    namespace Constants {
        const float RansacConfidenceInterval = 400.0;
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

