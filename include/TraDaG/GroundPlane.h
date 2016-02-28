/************************************************************//**
 * @file
 *
 * @brief GroundPlane class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef GROUNDPLANE_H
#define GROUNDPLANE_H

#include <TraDaG/KDTree.h>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreVector3.h>

#include <string>
#include <vector>

namespace TraDaG {
    class GroundPlane;
}

/**
 * @brief Class representing the plane that \ref DroppableObject "DroppableObject"s can drop on.
 *
 * This class is used by Simulator as the ground when dropping objects. It contains a standard
 * plane definition (normal + distance along the normal) and the inliers of the plane in the
 * scene as determined by RANSAC. In addition, it provides some utility functions.
 */
class TraDaG::GroundPlane
{
public:
    /**
     * @brief Default constructor.
     *
     * Constructs an empty, undefined plane.
     */
    GroundPlane();
    /**
     * @brief Constructor.
     * @param plane OGRE plane containing normal and distance along the normal.
     * @param vertices The inlier vertices of the plane.
     * @param label The label that was used to fit the plane.
     * @param autoConstructKDTree If set to @c true, a KD tree of the inlier vertices will be created immediately.
     * Otherwise, it will be created on the first \ref findKNearestNeighbors "nearest neighbor query".
     *
     * Constructs a plane defined by plane normal, distance along the normal, inlier vertices and
     * the label that was used to fit the plane.
     */
    GroundPlane(const Ogre::Plane& plane, const std::vector<Ogre::Vector3>& vertices, const std::string& label, bool autoConstructKDTree = false);

    /// Copy constructor.
    GroundPlane(const GroundPlane& other);
    /// Copy assignment operator.
    GroundPlane& operator=(const GroundPlane& other);

    /**
     * @brief Store this plane to a <em>.plane</em> file.
     * @param filePath Path (including file name) to save the plane to. The standard plane extension (<em>.plane</em>)
     * will be appended to the path.
     * @param overwrite Whether to overwrite the file if it already exists.
     * @return Success or failure.
     *
     * This function stores this specific plane definition to a file. The difference to PlaneInfo::saveToFile is that
     * PlaneInfo contains several @a regions of plane vertices, which will all be saved when using its save function.
     * Instances of this class usually are created from selecting one of the regions from a PlaneInfo instance, hence only the selected
     * region will be saved when using this function.
     *
     * @sa PlaneInfo::createGroundPlane
     */
    bool saveToFile(const std::string& filePath, bool overwrite = false) const;

    /**
     * @brief Project a point along the plane normal onto the plane.
     * @param point The point to project onto the plane.
     * @return The projected point.
     *
     * @remarks If the plane is undefined, the original point will be returned unmodified.
     */
    Ogre::Vector3 projectPointOntoPlane(const Ogre::Vector3& point) const;

    /**
     * @brief Find the k nearest neighbors of a query point from the inlier set.
     * @param queryPoint The point for which the nearest neighbors will be searched.
     * @param k How many nearest neighbors shall be searched.
     * @return Vector of k points containing the nearest neighbors.
     *
     * @remarks If the KD tree is not yet constructed when calling this function, it will be constructed automatically
     * before querying the nearest neighbors.
     */
    std::vector<Ogre::Vector3> findKNearestNeighbors(const Ogre::Vector3& queryPoint, unsigned int k);

    /// Check if this plane is valid, i.e. has a valid normal.
    bool isPlaneDefined() const;

    /// Get the underlying OGRE plane (containing normal and distance along the normal).
    Ogre::Plane getOgrePlane() const;
    /// Get the underlying OGRE plane (containing normal and distance along the normal).
    const Ogre::Plane& ogrePlane() const;
    /// Set the underlying OGRE plane (containing normal and distance along the normal).
    void setOgrePlane(const Ogre::Plane& plane);

    /// Get the inlier vertices of this plane.
    std::vector<Ogre::Vector3> getVertices() const;
    /// Get the inlier vertices of this plane.
    const std::vector<Ogre::Vector3>& vertices() const;
    /// Set the inlier vertices of this plane.
    void setVertices(const std::vector<Ogre::Vector3>& vertices);

    /**
     * @brief Get the label that this plane represents.
     * @remarks This is usually the label that was used to fit this plane into the scene with RANSAC.
     */
    std::string getLabel() const;
    /**
     * @brief Set the label that this plane represents.
     * @remarks This is usually the label that was used to fit this plane into the scene with RANSAC.
     */
    void setLabel(const std::string& label);

    /// Get the resitution of the plane (physics engine parameter).
    float getRestitution() const;
    /// Set the resitution of the plane (physics engine parameter).
    void setRestitution(float restitution);

    /// Get the friction of the plane (physics engine parameter).
    float getFriction() const;
    /// Set the friction of the plane (physics engine parameter).
    void setFriction(float friction);

    /**
     * @brief Parse a plane file and construct a plane from it.
     * @param filePath Path of the file to read the plane from. The standard plane extension (<em>.plane</em>)
     * will be appended to the path.
     * @return The constructed plane.
     */
    static GroundPlane readFromFile(const std::string& filePath);

    /**
     * @brief Create a plane from 3 vertices.
     * @param points The vertices to construct the plane.
     * @return Created OGRE plane.
     *
     * This function is used as the model construction function for RANSAC.
     */
    static Ogre::Plane createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points);
    /**
     * @brief Evaluate if a point is an inlier or an outlier of a specific plane.
     * @param point The point to check.
     * @param plane The plane for which the point will be checked.
     * @return Score of the point (normally 0 for inliers and 1 for outliers).
     *
     * This function is used as the point evaluation function for RANSAC.
     */
    static float pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane);

protected:
    Ogre::Plane mPlane;
    std::vector<Ogre::Vector3> mVertices;
    KDTree mKDTree;
    bool mKDTreeUpdated;

    std::string mLabel;

    float mRestitution;
    float mFriction;
};

#endif // GROUNDPLANE_H
