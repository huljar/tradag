/************************************************************//**
 * @file
 *
 * @brief KDTree class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef KDTREE_H
#define KDTREE_H

#include <OGRE/OgreVector3.h>

#include <queue>
#include <utility>
#include <vector>

namespace TraDaG {
    class KDTree;
}

/**
 * @brief Implementation of a KD tree used for nearest neighbor queries.
 */
class TraDaG::KDTree
{
public:
    class KDNode {
    public:
        KDNode();
        virtual ~KDNode();
        virtual bool isLeaf() const;
    };

    typedef std::vector<Ogre::Vector3> vertex_list;
    typedef typename vertex_list::iterator vertex_iterator;
    typedef typename vertex_list::const_iterator const_vertex_iterator;
    typedef std::pair<KDNode*, KDNode*> child_pair;
    typedef std::pair<Ogre::Vector3, Ogre::Vector3> bounds;
    typedef enum { X = 0, Y = 1, Z = 2 } split_plane_normal;

    class KDLeafNode : public KDNode {
    public:
        KDLeafNode(const vertex_iterator& vertices_begin, const vertex_iterator& vertices_end);
        bool isLeaf() const override;
        int numVertices() const;
        const_vertex_iterator begin() const;
        const_vertex_iterator end() const;
    private:
        const_vertex_iterator vertices_begin, vertices_end;
    };

    class KDSplitNode : public KDNode {
    public:
        KDSplitNode(KDNode* first_child, KDNode* second_child, const_vertex_iterator median, split_plane_normal planeNormal);
        ~KDSplitNode();
        bool isLeaf() const override;
        KDNode* first() const;
        KDNode* second() const;
        const_vertex_iterator getMedian() const;
        split_plane_normal getPlaneNormal() const;
    private:
        child_pair children;
        const_vertex_iterator median;
        split_plane_normal planeNormal;
    };

    struct ResultEntry {
        ResultEntry(float sqrDistance, const Ogre::Vector3* vertex);
        float sqrDistance;
        const Ogre::Vector3* vertex;
        bool operator<(const ResultEntry& right) const;
    };

private:
    struct SearchEntry {
        SearchEntry(float sqrDistance, const KDNode* node, const bounds& nodeBounds);
        float sqrDistance;
        const KDNode* node;
        bounds nodeBounds;
        bool operator<(const SearchEntry& right) const;
    };

public:
    KDTree();
    ~KDTree();
    void initialize(vertex_list* vertices, int maxDepth, int minSize);
    bool initialized() const;
    KDNode* root() const;
    std::vector<ResultEntry> findKNearestNeighbors(const Ogre::Vector3& origin, uint k) const;

    int getMaxDepth() const;
    int getMinSize() const;

private:
    vertex_list* vertices;
    int maxDepth;
    int minSize;
    KDNode* rootNode;
    bounds treeBounds;

    KDNode* construct(vertex_iterator begin, vertex_iterator end, int currentDepth);
    bounds getBounds(const_vertex_iterator begin, const_vertex_iterator end) const;
    void getKBestFromLeaf(const KDLeafNode* leaf, std::priority_queue<ResultEntry>& queue, const Ogre::Vector3& origin, uint k) const;
    void insertVertexIfNecessary(const_vertex_iterator vertex, std::priority_queue<ResultEntry>& queue, const Ogre::Vector3& origin, uint k) const;
    float sqrDistance(const Ogre::Vector3& first, const Ogre::Vector3& second) const;
    float sqrDistance(const Ogre::Vector3& origin, const bounds& box) const;
};

#endif // KDTREE_H
