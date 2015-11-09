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
        const std::string RgbdObjName = "sceneRgbdEntity";
        const std::string RenderWindowName = "Training Data Generator Preview Window";
    }

}

#endif // UTIL_H

