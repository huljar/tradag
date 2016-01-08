#include <TraDaG/util.h>

Ogre::Matrix3 TraDaG::convertCvMatToOgreMat(const cv::Matx33f& mat) {
    Ogre::Real conv[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            conv[i][j] = mat(i, j);
        }
    }
    return Ogre::Matrix3(conv);
}

cv::Matx33f TraDaG::convertOgreMatToCvMat(const Ogre::Matrix3& mat) {
    float conv[3][3];
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            conv[i][j] = mat[i][j];
        }
    }
    return cv::Matx33f(&conv[0][0]);
}
