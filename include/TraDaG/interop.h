#ifndef INTEROP_H
#define INTEROP_H

#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreVector2.h>
#include <OGRE/OgreVector3.h>

#include <opencv2/core/core.hpp>

namespace TraDaG {

    inline Ogre::Matrix3 cvToOgre(const cv::Matx33f& mat) {
        return Ogre::Matrix3(mat(0, 0), mat(0, 1), mat(0, 2),
                             mat(1, 0), mat(1, 1), mat(1, 2),
                             mat(2, 0), mat(2, 1), mat(2, 2));
    }

    inline Ogre::Vector2 cvToOgre(const cv::Vec2f& vec) {
        return Ogre::Vector2(vec[0], vec[1]);
    }

    inline Ogre::Vector3 cvToOgre(const cv::Vec3f& vec) {
        return Ogre::Vector3(vec[0], vec[1], vec[2]);
    }

    inline cv::Matx33f ogreToCv(const Ogre::Matrix3& mat) {
        return cv::Matx33f(mat[0][0], mat[0][1], mat[0][2],
                           mat[1][0], mat[1][1], mat[1][2],
                           mat[2][0], mat[2][1], mat[2][2]);
    }

    inline cv::Vec2f ogreToCv(const Ogre::Vector2& vec) {
        return cv::Vec2f(vec.x, vec.y);
    }

    inline cv::Vec3f ogreToCv(const Ogre::Vector3& vec) {
        return cv::Vec3f(vec.x, vec.y, vec.z);
    }
}

#endif // INTEROP_H

