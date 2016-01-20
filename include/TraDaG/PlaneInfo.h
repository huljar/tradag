#ifndef PLANEINFO_H
#define PLANEINFO_H

#include <TraDaG/GroundPlane.h>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreVector3.h>

#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

namespace TraDaG {
    class PlaneInfo;
}

class TraDaG::PlaneInfo
{
public:
    typedef std::tuple<std::vector<Ogre::Vector3>, unsigned short, unsigned short> Region;
    typedef std::vector<Region> RegionVec;

    enum class PickMode { LARGEST, UNIFORM_RANDOM, WEIGHTED_RANDOM };

    PlaneInfo();
    PlaneInfo(const Ogre::Plane& plane, const std::string& label);

    bool saveToFile(const std::string& filePath, bool overwrite = false) const;

    bool isPlaneDefined() const;

    GroundPlane createGroundPlane(unsigned short minDistance = 0, unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                  PickMode regionMode = PickMode::WEIGHTED_RANDOM);

    void addRegion(const Region& region);
    void addRegion(const std::vector<Ogre::Vector3>& vertices, unsigned short minDistance, unsigned short maxDistance);
    RegionVec& regions();

    // Retrieve PlaneInfo from file
    static PlaneInfo readFromFile(const std::string& filePath);

protected:
    template<typename T>
    inline bool intervalSubsumed(T minBigger, T maxBigger, T minSmaller, T maxSmaller) const {
        return minSmaller >= minBigger && maxSmaller <= maxBigger;
    }

    template<typename T>
    inline bool intervalsOverlap(T min1, T max1, T min2, T max2) const {
        return min1 <= max2 && min2 <= max1;
    }

    Ogre::Plane mPlane;
    RegionVec mRegions;
    std::string mLabel;

    std::default_random_engine mRandomEngine;
};

#endif // PLANEINFO_H
