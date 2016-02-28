#include <TraDaG/GroundPlane.h>
#include <TraDaG/debug.h>
#include <TraDaG/util.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;

GroundPlane::GroundPlane()
    : mKDTreeUpdated(false)
    , mRestitution(Defaults::PlaneRestitution)
    , mFriction(Defaults::PlaneFriction)
{

}

GroundPlane::GroundPlane(const Ogre::Plane& plane, const std::vector<Ogre::Vector3>& vertices, const std::string& label, bool autoConstructKDTree)
    : mPlane(plane)
    , mVertices(vertices)
    , mKDTreeUpdated(false)
    , mLabel(label)
    , mRestitution(Defaults::PlaneRestitution)
    , mFriction(Defaults::PlaneFriction)
{
    if(!mPlane.normal.isZeroLength())
        mPlane.normalise();

    if(autoConstructKDTree) {
        mKDTree.initialize(&mVertices, Constants::KDTreeMaxDepth, Constants::KDTreeMinSize);
        mKDTreeUpdated = true;
    }
}

GroundPlane::GroundPlane(const GroundPlane& other)
    : mPlane(other.mPlane)
    , mVertices(other.mVertices)
    , mKDTreeUpdated(false)
    , mLabel(other.mLabel)
    , mRestitution(other.mRestitution)
    , mFriction(other.mFriction)
{
}

GroundPlane& GroundPlane::operator=(const GroundPlane& other) {
    mPlane = other.mPlane;
    mVertices = other.mVertices;
    mKDTreeUpdated = false;
    mLabel = other.mLabel;
    mRestitution = other.mRestitution;
    mFriction = other.mFriction;

    return *this;
}

bool GroundPlane::saveToFile(const std::string& filePath, bool overwrite) const {
    DEBUG_OUT("Saving plane to file \"" << filePath + Strings::FileExtensionPlane << "\"");

    // Check if this plane is defined
    if(!isPlaneDefined()) {
        DEBUG_OUT("The plane is not defined and cannot be saved");
        return false;
    }

    // Convert file path to boost path
    fs::path path(filePath + Strings::FileExtensionPlane);

    // Check if the file exists (if we don't want to overwrite)
    if(!overwrite && fs::exists(path)) {
        DEBUG_OUT("File " << path << " already exists and overwrite is disabled");
        return false;
    }

    // Create output file stream and truncate file (if it already exists)
    fs::ofstream ofs(path, std::ios::trunc);
    if(!ofs.is_open()) {
        DEBUG_OUT("Failed to open file " << path);
        return false;
    }

    // Save plane label, normal, distance, restitution and friction
    ofs << "label " << mLabel << '\n'
        << "normal " << mPlane.normal.x << ' ' << mPlane.normal.y << ' ' << mPlane.normal.z << '\n'
        << "distance " << -mPlane.d << '\n'
        << "restitution " << mRestitution << '\n'
        << "friction " << mFriction << '\n';

    // Save plane vertices
    ofs << "\nvertices " << mVertices.size() << '\n';
    for(std::vector<Ogre::Vector3>::const_iterator it = mVertices.cbegin(); it != mVertices.cend(); ++it) {
        ofs << it->x << ' ' << it->y << ' ' << it->z << '\n';
    }

    // Close stream
    ofs.close();

    DEBUG_OUT("Plane successfully saved to file " << path);
    return true;
}

Ogre::Vector3 GroundPlane::projectPointOntoPlane(const Ogre::Vector3& point) const {
    if(isPlaneDefined())
        return mPlane.projectVector(point) - mPlane.d * mPlane.normal;

    return point;
}

std::vector<Ogre::Vector3> GroundPlane::findKNearestNeighbors(const Ogre::Vector3& queryPoint, unsigned int k) {
    // Construct KD tree if it is not yet constructed or is out of date
    if(!mKDTreeUpdated || !mKDTree.initialized()) {
        mKDTree.initialize(&mVertices, Constants::KDTreeMaxDepth, Constants::KDTreeMinSize);
        mKDTreeUpdated = true;
    }

    std::vector<KDTree::ResultEntry> res = mKDTree.findKNearestNeighbors(queryPoint, k);

    std::vector<Ogre::Vector3> ret;
    ret.reserve(res.size());
    for(std::vector<KDTree::ResultEntry>::iterator it = res.begin(); it != res.end(); ++it) {
        ret.push_back(*it->vertex);
    }

    return ret;
}

bool GroundPlane::isPlaneDefined() const {
    return !mPlane.normal.isZeroLength();
}

Ogre::Plane GroundPlane::getOgrePlane() const {
    return mPlane;
}

const Ogre::Plane& GroundPlane::ogrePlane() const {
    return mPlane;
}

void GroundPlane::setOgrePlane(const Ogre::Plane& plane) {
    mPlane = plane;
    mPlane.normalise();
}

std::vector<Ogre::Vector3> GroundPlane::getVertices() const {
    return mVertices;
}

const std::vector<Ogre::Vector3>& GroundPlane::vertices() const {
    return mVertices;
}

void GroundPlane::setVertices(const std::vector<Ogre::Vector3>& vertices) {
    mVertices = vertices;
    mKDTreeUpdated = false;
}

std::string GroundPlane::getLabel() const {
    return mLabel;
}

void GroundPlane::setLabel(const std::string& label) {
    mLabel = label;
}

float GroundPlane::getRestitution() const {
    return mRestitution;
}

void GroundPlane::setRestitution(float restitution) {
    mRestitution = restitution;
}

float GroundPlane::getFriction() const {
    return mFriction;
}

void GroundPlane::setFriction(float friction) {
    mFriction = friction;
}

GroundPlane GroundPlane::readFromFile(const std::string& filePath) {
    DEBUG_OUT("Reading plane from file \"" << filePath + Strings::FileExtensionPlane << "\"");

    // Convert file path to boost path
    fs::path path(filePath + Strings::FileExtensionPlane);

    // Check if the file exists
    if(!fs::exists(path)) {
        DEBUG_OUT("File " << path << " does not exist");
        return GroundPlane();
    }

    // Check if the file is a regular file
    if(!fs::is_regular_file(path)) {
        DEBUG_OUT("Cannot read from " << path << " since it is not a regular file");
        return GroundPlane();
    }

    // Create input file stream
    fs::ifstream ifs(path);
    if(!ifs.is_open()) {
        DEBUG_OUT("Failed to open file " << path);
        return GroundPlane();
    }

    // Read header (label, normal, distance, restitution and friction)
    std::string line, word;

    // Read label
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find label line");
        return GroundPlane();
    }

    std::istringstream labeliss(line);
    if(!(labeliss >> word)) {
        DEBUG_OUT("Couldn't read label line");
        return GroundPlane();
    }

    if(word != "label") {
        DEBUG_OUT("Label line didn't start with \"label\"");
        return GroundPlane();
    }

    std::string label;
    labeliss >> label; // It's ok if this fails, in this case no label was assigned to the plane and we leave it empty

    // Read normal
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find normal line");
        return GroundPlane();
    }

    std::istringstream normaliss(line);
    if(!(normaliss >> word)) {
        DEBUG_OUT("Couldn't read normal line");
        return GroundPlane();
    }

    if(word != "normal") {
        DEBUG_OUT("Normal line didn't start with \"normal\"");
        return GroundPlane();
    }

    Ogre::Vector3 normal;
    if(!(normaliss >> normal.x >> normal.y >> normal.z)) {
        DEBUG_OUT("Unable to parse plane normal");
        return GroundPlane();
    }

    // Read distance
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find distance line");
        return GroundPlane();
    }

    std::istringstream distanceiss(line);
    if(!(distanceiss >> word)) {
        DEBUG_OUT("Couldn't read distance line");
        return GroundPlane();
    }

    if(word != "distance") {
        DEBUG_OUT("Distance line didn't start with \"distance\"");
        return GroundPlane();
    }

    Ogre::Real distance;
    if(!(distanceiss >> distance)) {
        DEBUG_OUT("Unable to parse plane distance");
        return GroundPlane();
    }

    // Read restitution
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find restitution line");
        return GroundPlane();
    }

    std::istringstream restitutioniss(line);
    if(!(restitutioniss >> word)) {
        DEBUG_OUT("Couldn't read restitution line");
        return GroundPlane();
    }

    if(word != "restitution") {
        DEBUG_OUT("Restitution line didn't start with \"restitution\"");
        return GroundPlane();
    }

    float restitution;
    if(!(restitutioniss >> restitution)) {
        DEBUG_OUT("Unable to parse plane restitution");
        return GroundPlane();
    }

    // Read friction
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find friction line");
        return GroundPlane();
    }

    std::istringstream frictioniss(line);
    if(!(frictioniss >> word)) {
        DEBUG_OUT("Couldn't read friction line");
        return GroundPlane();
    }

    if(word != "friction") {
        DEBUG_OUT("Friction line didn't start with \"friction\"");
        return GroundPlane();
    }

    float friction;
    if(!(frictioniss >> friction)) {
        DEBUG_OUT("Unable to parse plane friction");
        return GroundPlane();
    }

    DEBUG_OUT("Headers parsed without errors");

    // Skip empty line(s)
    do {
        if(!std::getline(ifs, line)) {
            DEBUG_OUT("Couldn't find vertex definitions");
            return GroundPlane();
        }
    } while(line.empty());

    // Read number of vertices
    std::istringstream verticesiss(line);
    if(!(verticesiss >> word)) {
        DEBUG_OUT("Couldn't read vertices line");
        return GroundPlane();
    }

    if(word != "vertices") {
        DEBUG_OUT("Vertices line didn't start with \"vertices\"");
        return GroundPlane();
    }

    size_t numVertices;
    if(!(verticesiss >> numVertices)) {
        DEBUG_OUT("Unable to parse number of vertices");
        return GroundPlane();
    }

    // Read vertices
    std::vector<Ogre::Vector3> vertices;
    vertices.reserve(numVertices);

    while(std::getline(ifs, line)) {
        std::istringstream vertexiss(line);
        Ogre::Vector3 vertex;

        if(!(vertexiss >> vertex.x >> vertex.y >> vertex.z)) {
            DEBUG_OUT("Encountered invalid vertex definition");
            return GroundPlane();
        }

        vertices.push_back(vertex);
    }

    DEBUG_OUT("Body parsed without errors");

    // Reached end of file, close file stream
    ifs.close();

    // Construct and return GroundPlane
    DEBUG_OUT("Successfully parsed " << path << " with the following information:");
    DEBUG_OUT("    Label: " << label);
    DEBUG_OUT("    Normal: " << normal << ", distance: " << distance);
    DEBUG_OUT("    Number of vertices: " << vertices.size());
    DEBUG_OUT("    Restitution: " << restitution);
    DEBUG_OUT("    Friction: " << friction);

    GroundPlane ret(Ogre::Plane(normal, distance), vertices, label);
    ret.setRestitution(restitution);
    ret.setFriction(friction);

    return ret;
}

Ogre::Plane GroundPlane::createPlaneFromPoints(const std::array<Ogre::Vector3, 3>& points) {
    Ogre::Plane ret(points[0], points[1], points[2]);
    ret.normalise(); // required so the getDistance member function returns the correct distance
    return ret;
}

float GroundPlane::pointEvaluation(const Ogre::Vector3& point, const Ogre::Plane& plane) {
    // Check for invalid plane first (this can occur, probably when the RANSAC sampled points
    // lie on a line and no unique plane can be created, or when the same point was sampled twice)
    if(plane.normal == Ogre::Vector3::ZERO)
        return 1.0; // treat every point as outlier so this model will be discarded

    // Ogre::Plane::getDistance returns positive/negative values depending on which side of the plane the point lies
    if(std::abs(plane.getDistance(point)) < Constants::RansacConfidenceInterval)
        return 0.0;

    return 1.0;
}
