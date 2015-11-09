#ifndef UTIL_H
#define UTIL_H

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
    struct Automatable {
        Automatable(bool automate, const T& specific = T())
            : automated(automate), manualValue(specific)
        {
        }

        const bool automated;
        const T& manualValue;
    };

}

#endif // UTIL_H

