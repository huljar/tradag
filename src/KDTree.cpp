#include <TraDaG/KDTree.h>
#include <TraDaG/debug.h>

#include <algorithm>
#include <cmath>

using namespace TraDaG;

KDTree::KDNode::KDNode() {

}

KDTree::KDNode::~KDNode() {

}

bool KDTree::KDNode::isLeaf() const {
    return false;
}

KDTree::KDLeafNode::KDLeafNode(const vertex_iterator& vertices_begin, const vertex_iterator& vertices_end) :
    vertices_begin(vertices_begin),
    vertices_end(vertices_end)
{

}

bool KDTree::KDLeafNode::isLeaf() const {
    return true;
}

int KDTree::KDLeafNode::numVertices() const {
    return vertices_end - vertices_begin;
}

KDTree::const_vertex_iterator KDTree::KDLeafNode::begin() const {
    return vertices_begin;
}

KDTree::const_vertex_iterator KDTree::KDLeafNode::end() const {
    return vertices_end;
}

KDTree::KDSplitNode::KDSplitNode(KDNode* first_child, KDNode* second_child, const_vertex_iterator median, split_plane_normal planeNormal) :
    children(first_child, second_child),
    median(median),
    planeNormal(planeNormal)
{

}

KDTree::KDSplitNode::~KDSplitNode() {
    delete children.first;
    delete children.second;
}

bool KDTree::KDSplitNode::isLeaf() const {
    return false;
}

KDTree::KDNode* KDTree::KDSplitNode::first() const {
    return children.first;
}

KDTree::KDNode* KDTree::KDSplitNode::second() const {
    return children.second;
}

KDTree::const_vertex_iterator KDTree::KDSplitNode::getMedian() const {
    return median;
}

KDTree::split_plane_normal KDTree::KDSplitNode::getPlaneNormal() const {
    return planeNormal;
}

KDTree::ResultEntry::ResultEntry(float sqrDistance, const Ogre::Vector3* vertex) :
    sqrDistance(sqrDistance),
    vertex(vertex)
{

}

bool KDTree::ResultEntry::operator<(const ResultEntry& right) const {
    return sqrDistance < right.sqrDistance;
}

KDTree::SearchEntry::SearchEntry(float sqrDistance, const KDNode* node, const bounds& nodeBounds) :
    sqrDistance(sqrDistance),
    node(node),
    nodeBounds(nodeBounds)
{

}

bool KDTree::SearchEntry::operator<(const SearchEntry& right) const {
    return sqrDistance > right.sqrDistance;
}

KDTree::KDTree() :
    vertices(nullptr),
    rootNode(nullptr)
{

}

KDTree::~KDTree() {
    delete rootNode;
}

void KDTree::initialize(vertex_list* vertices, int maxDepth, int minSize) {
    DEBUG_OUT("Constructing KD tree for " << vertices->size() << " vertices with a max depth of " << maxDepth << " and a min leaf size of " << minSize);
    delete rootNode;

    this->vertices = vertices;
    this->maxDepth = maxDepth;
    this->minSize = minSize;
    treeBounds = getBounds(vertices->begin(), vertices->end());
    rootNode = construct(vertices->begin(), vertices->end(), 0);
    DEBUG_OUT("Finished construction of KD tree");
}

bool KDTree::initialized() const {
    return rootNode != nullptr;
}

KDTree::KDNode* KDTree::root() const {
    return rootNode;
}

std::vector<KDTree::ResultEntry> KDTree::findKNearestNeighbors(const Ogre::Vector3& origin, uint k) const {
    std::priority_queue<SearchEntry> searchQueue;
    std::priority_queue<ResultEntry> resultQueue;

    KDNode* currentNode = root();
    bounds currentBounds = treeBounds;

    while(!currentNode->isLeaf()) {
        // Traverse the k-d tree
        KDSplitNode* currentSplit = dynamic_cast<KDSplitNode*>(currentNode);
        split_plane_normal planeNormal = currentSplit->getPlaneNormal();
        float medianCoord = (*currentSplit->getMedian())[planeNormal];

        bounds firstBounds = currentBounds;
        bounds secondBounds = currentBounds;

        firstBounds.second[planeNormal] = medianCoord;
        secondBounds.first[planeNormal] = medianCoord;

        if(sqrDistance(origin, firstBounds) < sqrDistance(origin, secondBounds)) {
            currentNode = currentSplit->first();
            currentBounds = firstBounds;

            searchQueue.push(SearchEntry(sqrDistance(origin, secondBounds), currentSplit->second(), secondBounds));
        }
        else {
            currentNode = currentSplit->second();
            currentBounds = secondBounds;

            searchQueue.push(SearchEntry(sqrDistance(origin, firstBounds), currentSplit->first(), firstBounds));
        }

        insertVertexIfNecessary(currentSplit->getMedian(), resultQueue, origin, k);
    }

    // Get k best vertices from the leaf
    KDLeafNode* leaf = dynamic_cast<KDLeafNode*>(currentNode);
    getKBestFromLeaf(leaf, resultQueue, origin, k);

    // Iterate through all other nodes that are closer than the worst vertex in the queue (or as long as the queue does not contain k elements)
    while(!searchQueue.empty() && (searchQueue.top().sqrDistance < resultQueue.top().sqrDistance || resultQueue.size() < k)) {
        SearchEntry top = searchQueue.top();
        searchQueue.pop();

        if(top.node->isLeaf())
            getKBestFromLeaf(dynamic_cast<const KDLeafNode*>(top.node), resultQueue, origin, k);
        else {
            const KDSplitNode* topSplit = dynamic_cast<const KDSplitNode*>(top.node);
            split_plane_normal normal = topSplit->getPlaneNormal();
            float medianCoord = (*topSplit->getMedian())[normal];

            KDNode* first = topSplit->first();
            bounds firstBounds = top.nodeBounds;
            firstBounds.second[normal] = medianCoord;

            KDNode* second = topSplit->second();
            bounds secondBounds = top.nodeBounds;
            secondBounds.first[normal] = medianCoord;

            insertVertexIfNecessary(topSplit->getMedian(), resultQueue, origin, k);

            searchQueue.push(SearchEntry(sqrDistance(origin, firstBounds), first, firstBounds));
            searchQueue.push(SearchEntry(sqrDistance(origin, secondBounds), second, secondBounds));
        }
    }

    // Convert priority queue to vector and return
    std::vector<ResultEntry> ret;
    uint size = resultQueue.size();
    ret.reserve(size);
    for(uint i = 0; i < size; ++i) {
        ret.push_back(resultQueue.top());
        resultQueue.pop();
    }
    return ret;
}

void KDTree::getKBestFromLeaf(const KDLeafNode* leaf, std::priority_queue<ResultEntry>& queue, const Ogre::Vector3& origin, uint k) const {
    for(const_vertex_iterator it = leaf->begin(); it != leaf->end(); ++it) {
        float sqrDist = sqrDistance(origin, *it);
        if(queue.size() < k)
            queue.push(ResultEntry(sqrDist, &(*it)));
        else if(queue.top().sqrDistance > sqrDist) {
            queue.pop();
            queue.push(ResultEntry(sqrDist, &(*it)));
        }
    }
}

void KDTree::insertVertexIfNecessary(const_vertex_iterator vertex, std::priority_queue<ResultEntry>& queue, const Ogre::Vector3& origin, uint k) const {
    float sqrDist = sqrDistance(origin, *vertex);
    if(queue.size() < k)
        queue.push(ResultEntry(sqrDist, &(*vertex)));
    else if(queue.top().sqrDistance > sqrDist) {
        queue.pop();
        queue.push(ResultEntry(sqrDist, &(*vertex)));
    }
}

KDTree::KDNode* KDTree::construct(vertex_iterator begin, vertex_iterator end, int currentDepth) {
    if(currentDepth >= maxDepth || end - begin <= minSize || end - begin < 3) {
        return new KDLeafNode(begin, end);
    }

    bounds myBounds = getBounds(begin, end);
    float distX = myBounds.second.x - myBounds.first.x;
    float distY = myBounds.second.y - myBounds.first.y;
    float distZ = myBounds.second.z - myBounds.first.z;

    split_plane_normal myPlaneNormal;
    if(distY > distX) {
        if(distZ > distY)
            myPlaneNormal = Z;
        else
            myPlaneNormal = Y;
    }
    else {
        if(distZ > distX)
            myPlaneNormal = Z;
        else
            myPlaneNormal = X;
    }

    vertex_iterator mid = begin + (end - begin) / 2;
    std::nth_element(begin, mid, end, [&myPlaneNormal](const Ogre::Vector3& first, const Ogre::Vector3& second) {
        return first[myPlaneNormal] < second[myPlaneNormal];
    });

    return new KDSplitNode(construct(begin, mid, currentDepth + 1), construct(mid + 1, end, currentDepth + 1), mid, myPlaneNormal);
}

KDTree::bounds KDTree::getBounds(const_vertex_iterator begin, const_vertex_iterator end) const {
    float inf = std::numeric_limits<float>::infinity();
    bounds ret(Ogre::Vector3(inf, inf, inf), Ogre::Vector3(-inf, -inf, -inf));

    for(; begin != end; ++begin) {
        for(int i = 0; i < 3; ++i) {
            if((*begin)[i] < ret.first[i])
                ret.first[i] = (*begin)[i];
            if((*begin)[i] > ret.second[i])
                ret.second[i] = (*begin)[i];
        }
    }

    return ret;
}

float KDTree::sqrDistance(const Ogre::Vector3& first, const Ogre::Vector3& second) const {
    return (first - second).squaredLength();
}

float KDTree::sqrDistance(const Ogre::Vector3& origin, const bounds& box) const {
    Ogre::Vector3 closestPoint;

    for(int i = 0; i < 3; ++i)
        closestPoint[i] = std::max(std::min(origin[i], box.second[i]), box.first[i]);

    return sqrDistance(origin, closestPoint);
}

int KDTree::getMinSize() const {
    return minSize;
}

int KDTree::getMaxDepth() const {
    return maxDepth;
}
