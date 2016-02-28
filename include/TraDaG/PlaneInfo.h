/************************************************************//**
 * @file
 *
 * @brief PlaneInfo class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef PLANEINFO_H
#define PLANEINFO_H

#include <TraDaG/GroundPlane.h>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreVector3.h>

#include <boost/filesystem/fstream.hpp>

#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

namespace TraDaG {
    class PlaneInfo;
}

/**
 * @brief Class that stores information about planes.
 *
 * This class is similar to GroundPlane, but stores all vertices of the plane (i.e. not only a
 * single connected region). GroundPlane instances can be created from this class using
 * createGroundPlane.
 */
class TraDaG::PlaneInfo
{
public:
    /// Vector of vertices belonging to a region, together with minimum and maximum distance of the vertices.
    typedef std::tuple<std::vector<Ogre::Vector3>, unsigned short, unsigned short> Region;
    /// Vector of regions.
    typedef std::vector<Region> RegionVec;
    /// Tuple containing plane label, plane normal and the distance of the plane along this normal.
    typedef std::tuple<std::string, Ogre::Vector3, Ogre::Real> Headers;

    /**
     * @brief Enumerator describing how a region should be selected from the available ones.
     */
    enum class PickMode {
        LARGEST, /**< Pick the largest region, i.e. the region with the most vertices. */
        UNIFORM_RANDOM,  /**< Pick a randomly selected region. */
        WEIGHTED_RANDOM /**< Pick a randomly selected region, but weight each region with its number of vertices. */
    };

    /**
     * @brief Default constructor.
     *
     * Constructs an empty %PlaneInfo instance with an undefined plane.
     */
    PlaneInfo();
    /**
     * @brief Constructor.
     * @param plane OGRE plane containing normal and distance along the normal.
     * @param label The label that was used to fit the plane.
     *
     * Constructs a plane defined by plane normal, distance along the normal and
     * the label that was used to fit the plane. Regions can be added using the
     * appropriate functions.
     *
     * @sa addRegion
     * @sa regions
     */
    PlaneInfo(const Ogre::Plane& plane, const std::string& label);

    /**
     * @brief Store this plane information to a <em>.planeinfo</em> file.
     * @param filePath Path (including file name) to save the plane information to. The standard plane extension (<em>.planeinfo</em>)
     * will be appended to the path.
     * @param overwrite Whether to overwrite the file if it already exists.
     * @return Success or failure.
     *
     * This function stores this plane information definition to a file. The difference to GroundPlane::saveToFile is that
     * GroundPlane contains only a single region of connected vertices, and only this region will be saved when using its save function.
     * This class will save all regions without distance limitations, so it is better suited when e.g. precomputing a lot of planes.
     *
     * @sa createGroundPlane
     */
    bool saveToFile(const std::string& filePath, bool overwrite = false) const;

    /// Check if this plane is valid, i.e. has a valid normal.
    bool isPlaneDefined() const;

    /**
     * @brief Extract a GroundPlane instance from this plane info.
     * @param minDistance Minimum distance of vertices to the camera.
     * @param maxDistance Maximum distance of vertices to the camera.
     * @param regionMode This specifies how the region should be selected from those that remain after
     * the distance checks.
     * @return The created GroundPlane instance.
     *
     * @remarks The number of vertices considered for \ref PickMode::LARGEST "largest" and
     * \ref PickMode::WEIGHTED_RANDOM "weighted random" is the number after the distance checks,
     * i.e. after removing all vertices that are not within the distance interval from the regions.
     */
    GroundPlane createGroundPlane(unsigned short minDistance = 0, unsigned short maxDistance = std::numeric_limits<unsigned short>::max(),
                                  PickMode regionMode = PickMode::WEIGHTED_RANDOM);

    /// Add a region to this plane info definition.
    void addRegion(const Region& region);
    /// Add a region to this plane info definition.
    void addRegion(const std::vector<Ogre::Vector3>& vertices, unsigned short minDistance, unsigned short maxDistance);
    /// Get the region vector of this plane info definition. It can be modified freely.
    RegionVec& regions();

    /**
     * @brief Parse a plane info file and construct a plane info instance from it.
     * @param filePath Path of the file to read the plane info from. The standard plane info extension (<em>.planeinfo</em>)
     * will be appended to the path.
     * @return The constructed plane info instance.
     */
    static PlaneInfo readFromFile(const std::string& filePath);
    /**
     * @brief Parse only the headers of a plane info file.
     * @param filePath Path of the file to read the plane info headers from. The standard plane info extension (<em>.planeinfo</em>)
     * will be appended to the path.
     * @return The parsed headers.
     *
     * This method can be used to quickly check if the label and/or plane normal of a file matches what you
     * are looking for, without parsing all the regions and vertices of the file.
     */
    static Headers readHeadersFromFile(const std::string& filePath);

protected:
    static Headers parseHeaders(boost::filesystem::ifstream& ifs);

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
