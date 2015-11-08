#ifndef UTIL_H
#define UTIL_H

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

}

#endif // UTIL_H

