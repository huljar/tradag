#include <TraDaG/PlaneInfo.h>
#include <TraDaG/debug.h>
#include <TraDaG/util.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace TraDaG;
namespace fs = boost::filesystem;

PlaneInfo::PlaneInfo()
    : mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{
}

PlaneInfo::PlaneInfo(const Ogre::Plane& plane, const std::string& label)
    : mPlane(plane)
    , mLabel(label)
    , mRandomEngine(std::chrono::system_clock::now().time_since_epoch().count())
{

}

bool PlaneInfo::saveToFile(const std::string& filePath, bool overwrite) const {
    DEBUG_OUT("Saving plane info to file \"" << filePath + Strings::FileExtensionPlaneInfo << "\"");

    // Convert file path to boost path
    fs::path path(filePath + Strings::FileExtensionPlaneInfo);

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

    // Save plane label, normal and distance
    ofs << "label " << mLabel << '\n'
        << "normal " << mPlane.normal.x << ' ' << mPlane.normal.y << ' ' << mPlane.normal.z << '\n'
        << "distance " << -mPlane.d << '\n';

    // Save plane regions
    ofs << "\nregions " << mRegions.size() << '\n';

    for(RegionVec::const_iterator it = mRegions.cbegin(); it != mRegions.cend(); ++it) {
        const std::vector<Ogre::Vector3>& region = std::get<0>(*it);
        ofs << "start region " << std::get<1>(*it) << ' ' << std::get<2>(*it) << ' ' << region.size() << '\n';

        for(std::vector<Ogre::Vector3>::const_iterator jt = region.cbegin(); jt != region.cend(); ++jt) {
            ofs << jt->x << ' ' << jt->y << ' ' << jt->z << '\n';
        }

        ofs << "end region\n";
    }

    // Close stream
    ofs.close();

    DEBUG_OUT("Plane info successfully saved to file " << path);
    return true;
}

bool PlaneInfo::isPlaneDefined() const {
    return !mPlane.normal.isZeroLength();
}

GroundPlane PlaneInfo::createGroundPlane(unsigned short minDistance, unsigned short maxDistance, PlaneInfo::PickMode regionMode) {
    DEBUG_OUT("Computing Ground Plane for label \"" << mLabel << "\" within distance [" << minDistance << ", " << maxDistance << "]");

    // Check if we are defined
    if(!isPlaneDefined()) {
        DEBUG_OUT("Plane is undefined");
        return GroundPlane();
    }

    // Find all regions within the specified interval
    std::vector<std::vector<Ogre::Vector3>> validRegions;

    for(RegionVec::const_iterator it = mRegions.cbegin(); it != mRegions.cend(); ++it) {
        unsigned short regionMin = std::get<1>(*it);
        unsigned short regionMax = std::get<2>(*it);

        DEBUG_OUT("Checking region " << std::distance(mRegions.cbegin(), it) << ", distance: [" << regionMin << ", " << regionMax << "]");

        // Check if region lies completely within the bounds
        if(intervalSubsumed<unsigned short>(minDistance, maxDistance, regionMin, regionMax)) {
            DEBUG_OUT("Region lies within the bounds");

            // Set region as valid if it has enough vertices
            const std::vector<Ogre::Vector3>& tmpRegion = std::get<0>(*it);
            if(tmpRegion.size() >= Constants::MinRegionPixelsToBeValid) {
                DEBUG_OUT("Adding region with " << tmpRegion.size() << " vertices");
                validRegions.push_back(tmpRegion);
            }
            else {
                DEBUG_OUT("Not adding region because it contains less than " << Constants::MinRegionPixelsToBeValid << " vertices");
            }
        }
        // Else check if the region partially overlaps with the bounds
        else if(intervalsOverlap<unsigned short>(minDistance, maxDistance, regionMin, regionMax)) {
            DEBUG_OUT("Region lies partially within the bounds");
            DEBUG_OUT("The region contains " << std::get<0>(*it).size() << " vertices, removing invalid ones");

            // Remove all vertices that do not lie within the bounds
            std::vector<Ogre::Vector3> tmpRegion(std::get<0>(*it));

            std::vector<Ogre::Vector3>::iterator tmpEnd = std::remove_if(tmpRegion.begin(), tmpRegion.end(),
                [minDistance, maxDistance] (const Ogre::Vector3& value) -> bool {
                    unsigned short elemDist = static_cast<unsigned short>(-value.z);
                    return elemDist < minDistance || elemDist > maxDistance;
                }
            );

            DEBUG_OUT(std::distance(tmpRegion.begin(), tmpEnd) << " valid vertices left");

            // Check if enough vertices are left
            if(std::distance(tmpRegion.begin(), tmpEnd) >= Constants::MinRegionPixelsToBeValid) {
                DEBUG_OUT("Adding remaining vertices as region");

                // Insert remaining vertices as a valid region
                validRegions.push_back(std::vector<Ogre::Vector3>(tmpRegion.begin(), tmpEnd));
            }
        }
        else {
            DEBUG_OUT("Region does not lie within the bounds");
        }
    }

    DEBUG_OUT(validRegions.size() << " regions remain after distance check");

    // Pick a region according to the regionMode
    std::vector<std::vector<Ogre::Vector3>>::iterator pick;
    if(regionMode == PickMode::LARGEST) {
        DEBUG_OUT("Picking the largest region for plane construction");

        size_t largestNumVertices = 0;
        for(std::vector<std::vector<Ogre::Vector3>>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            if(it->size() > largestNumVertices) {
                DEBUG_OUT("Registering region " << std::distance(validRegions.begin(), it) << " with " << it->size() << " vertices as currently largest region");

                pick = it;
                largestNumVertices = it->size();
            }
        }
    }
    else if(regionMode == PickMode::UNIFORM_RANDOM) {
        DEBUG_OUT("Picking a random region for plane construction");

        std::uniform_int_distribution<size_t> distribution(0, validRegions.size() - 1);
        pick = validRegions.begin() + distribution(mRandomEngine);
    }
    else if(regionMode == PickMode::WEIGHTED_RANDOM) {
        DEBUG_OUT("Picking a weighted random region for plane construction");

        // Using the algorithm as described here: https://stackoverflow.com/questions/1761626/weighted-random-numbers
        // Weights are the number of vertices in each region

        // Calculate sum of weights
        unsigned long sumOfWeights = 0;
        for(std::vector<std::vector<Ogre::Vector3>>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            sumOfWeights += static_cast<unsigned long>(it->size());
        }

        // Pick a random number between 0 and sum of weights
        std::uniform_int_distribution<unsigned long> distribution(0, sumOfWeights - 1);
        unsigned long rnd = distribution(mRandomEngine);

        // Select the corresponding region
        bool pickFound = false;
        for(std::vector<std::vector<Ogre::Vector3>>::iterator it = validRegions.begin(); it != validRegions.end(); ++it) {
            unsigned long weight = static_cast<unsigned long>(it->size());
            if(rnd < weight) {
                pick = it;
                pickFound = true;
                break;
            }
            rnd -= weight;
        }
        if(!pickFound)
            throw std::logic_error("Unable to pick a weighted random region - the function should never reach this point");
    }
    else {
        throw std::invalid_argument("Invalid region mode specified");
    }

    DEBUG_OUT("Picked region " << std::distance(validRegions.begin(), pick) << " with " << pick->size() << " valid vertices");

    // Construct and return plane
    return GroundPlane(mPlane, *pick, mLabel);
}

void PlaneInfo::addRegion(const Region& region) {
    mRegions.push_back(region);
}

void PlaneInfo::addRegion(const std::vector<Ogre::Vector3>& vertices, unsigned short minDistance, unsigned short maxDistance) {
    mRegions.push_back(std::make_tuple(vertices, minDistance, maxDistance));
}

PlaneInfo::RegionVec& PlaneInfo::regions() {
    return mRegions;
}

TraDaG::PlaneInfo PlaneInfo::readFromFile(const std::string& filePath) {
    DEBUG_OUT("Reading plane info from file \"" << filePath + Strings::FileExtensionPlaneInfo << "\"");

    // Convert file path to boost path
    fs::path path(filePath + Strings::FileExtensionPlaneInfo);

    // Check if the file exists
    if(!fs::exists(path)) {
        DEBUG_OUT("File " << path << " does not exist");
        return PlaneInfo();
    }

    // Check if the file is a regular file
    if(!fs::is_regular_file(path)) {
        DEBUG_OUT("Cannot read from " << path << " since it is not a regular file");
        return PlaneInfo();
    }

    // Create input file stream
    fs::ifstream ifs(path);
    if(!ifs.is_open()) {
        DEBUG_OUT("Failed to open file " << path);
        return PlaneInfo();
    }

    // Read header (label, normal and distance)
    std::string line;
    std::string word, word2;

    // Read label
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find label line");
        return PlaneInfo();
    }

    std::istringstream labeliss(line);
    if(!(labeliss >> word)) {
        DEBUG_OUT("Couldn't read label line");
        return PlaneInfo();
    }

    if(word != "label") {
        DEBUG_OUT("Label line didn't start with \"label\"");
        return PlaneInfo();
    }

    std::string label;
    if(!(labeliss >> label)) {
        DEBUG_OUT("Unable to parse label");
        return PlaneInfo();
    }

    // Read normal
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find normal line");
        return PlaneInfo();
    }

    std::istringstream normaliss(line);
    if(!(normaliss >> word)) {
        DEBUG_OUT("Couldn't read normal line");
        return PlaneInfo();
    }

    if(word != "normal") {
        DEBUG_OUT("Normal line didn't start with \"normal\"");
        return PlaneInfo();
    }

    Ogre::Vector3 normal;
    if(!(normaliss >> normal.x >> normal.y >> normal.z)) {
        DEBUG_OUT("Unable to parse plane normal");
        return PlaneInfo();
    }

    // Read distance
    if(!std::getline(ifs, line)) {
        DEBUG_OUT("Couldn't find distance line");
        return PlaneInfo();
    }

    std::istringstream distanceiss(line);
    if(!(distanceiss >> word)) {
        DEBUG_OUT("Couldn't read distance line");
        return PlaneInfo();
    }

    if(word != "distance") {
        DEBUG_OUT("Distance line didn't start with \"distance\"");
        return PlaneInfo();
    }

    Ogre::Real distance;
    if(!(distanceiss >> distance)) {
        DEBUG_OUT("Unable to parse plane distance");
        return PlaneInfo();
    }

    DEBUG_OUT("Headers parsed without errors");

    // Skip empty line(s)
    do {
        if(!std::getline(ifs, line)) {
            DEBUG_OUT("Couldn't find region definitions");
            return PlaneInfo();
        }
    } while(line.empty());

    // Read number of regions
    std::istringstream regionsiss(line);
    if(!(regionsiss >> word)) {
        DEBUG_OUT("Couldn't read regions line");
        return PlaneInfo();
    }

    if(word != "regions") {
        DEBUG_OUT("Regions line didn't start with \"regions\"");
        return PlaneInfo();
    }

    size_t numRegions;
    if(!(regionsiss >> numRegions)) {
        DEBUG_OUT("Unable to parse number of regions");
        return PlaneInfo();
    }

    // Read regions
    RegionVec regions;
    regions.reserve(numRegions);

    while(std::getline(ifs, line)) {
        std::istringstream regionstartiss(line);
        if(!(regionstartiss >> word >> word2)) {
            DEBUG_OUT("Couldn't read region start line");
            return PlaneInfo();
        }

        if(word != "start" || word2 != "region") {
            DEBUG_OUT("Region start line didn't start with \"start region\"");
            return PlaneInfo();
        }

        unsigned short minDistance, maxDistance;
        size_t numVertices;
        if(!(regionstartiss >> minDistance >> maxDistance >> numVertices)) {
            DEBUG_OUT("Unable to parse region start");
            return PlaneInfo();
        }

        // Read vertices of region
        std::vector<Ogre::Vector3> vertices;
        vertices.reserve(numVertices);

        while(std::getline(ifs, line) && line != "end region") {
            std::istringstream vertexiss(line);
            Ogre::Vector3 vertex;

            if(!(vertexiss >> vertex.x >> vertex.y >> vertex.z)) {
                DEBUG_OUT("Encountered invalid vertex definition");
                return PlaneInfo();
            }

            vertices.push_back(vertex);
        }

        // Add region
        regions.push_back(std::make_tuple(std::move(vertices), minDistance, maxDistance));
    }

    DEBUG_OUT("Body parsed without errors");

    // Reached end of file, close file stream
    ifs.close();

    // Construct and return PlaneInfo
    DEBUG_OUT("Successfully parsed " << path << " with the following information:");
    DEBUG_OUT("    Label: " << label);
    DEBUG_OUT("    Normal: " << normal << ", distance: " << distance);
    DEBUG_OUT("    Number of regions: " << regions.size());

    PlaneInfo ret(Ogre::Plane(normal, distance), label);
    ret.regions() = std::move(regions);

    return ret;
}
