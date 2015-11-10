#ifndef UTIL_H
#define UTIL_H

#include <opencv2/core/core.hpp>

#include <string>

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

    namespace Strings {
        const std::string ResourcesCfgPath = "../config/resources.cfg";
        const std::string PluginsCfgPath = "../config/plugins.cfg";

        const std::string RgbdSceneName = "sceneRgbdEntity";
        const std::string ObjRigidBodyName = "objectBody";
        const std::string PlaneRigidBodyName = "planeBody";

        const std::string RenderWindowName = "Training Data Generator Preview Window";
    }

    template<class T>
    struct Auto {
        Auto(bool automate, const T& specific = T())
            : automated(automate), manualValue(specific)
        {
        }

        const bool automated;
        const T& manualValue;
    };

    struct ObjectDropResult {
        ObjectDropResult(bool successful, const cv::Mat& image, float covered)
            : success(successful), renderedImage(image), fractionCovered(covered)
        {
        }

        const bool success;
        const cv::Mat renderedImage;
        const float fractionCovered;
    };

}

#endif // UTIL_H

