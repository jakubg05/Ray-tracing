#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <algorithm>
#include <queue>

#ifndef RTX_MATERIAL
#define RTX_MATERIAL
struct RaytracingMaterial
{
    // Thanks to https://learnopengl.com/Advanced-OpenGL/Advanced-GLSL

    glm::vec3 color;              // offset 0   // alignment 16 // size 12 // total 12 bytes
    float emissionStrength;       // offset 12  // alignment 4  // size 4  // total 16 bytes
    glm::vec3 emissionColor;      // offset 16  // alignment 16 // size 12 // total 28 bytes
    float std140padding;          // offset 28  // alignment 4  // size 4  // total 32 bytes
};
#endif

#ifndef TRIANGLE
#define TRIANGLE
struct Triangle
{
    // vertices
    glm::vec3 v1;
    float padding1; // Add padding for alignment
    glm::vec3 v2;
    float padding2; // Add padding for alignment
    glm::vec3 v3;
    float padding3; // Add padding for alignment

    // normals
    glm::vec3 NA;
    float padding4; // Add padding for alignment
    glm::vec3 NB;
    float padding5; // Add padding for alignment
    glm::vec3 NC;
    float padding6; // Add padding for alignment

    glm::vec3 centroid;
    float padding7;

    RaytracingMaterial material; // already padded correctly

    friend std::ostream& operator<<(std::ostream& os, const Triangle& triangle);
};
#endif // end of TRIANGLE

#ifndef OBJ_PARSER
#define OBJ_PARSER
void loadMesh(std::string filePath, std::vector<Triangle>& mesh, unsigned int& numTriangles);
#endif

#ifndef BVH_IMPLEMENTATION
#define BVH_IMPLEMENTATION
namespace BVH {
    enum class Heuristic {
        OBJECT_MEDIAN_SPLIT,
        SPATIAL_MIDDLE_SPLIT
    };

    const unsigned int AABB_primitives_limit = 4;

    class Node {
    public:
        Node(){}
        Node(glm::vec3 minVec, glm::vec3 maxVec);
        friend std::ostream& operator<<(std::ostream& os, const Node& node);

        unsigned int leaf_primitive_indices[AABB_primitives_limit];

        glm::vec3 minVec;
        int child1_idx;
        glm::vec3 maxVec;
        int child2_idx;
    };

    struct BVH_data {
        BVH::Node* BVH;
        unsigned int size;
    };

    glm::vec3 minCorner(const glm::vec3& current_min, const glm::vec3& vertex);

    glm::vec3 maxCorner(const glm::vec3& current_max, const glm::vec3& vertex);

    void computeAABB(const std::vector<unsigned int>& triangle_indices, const std::vector<Triangle>& triangle_mesh, glm::vec3& minVec, glm::vec3& maxVec);

    BVH::Node init(const std::vector<unsigned int>& triangle_indices, const std::vector<Triangle> triangle_mesh);

    struct Partition_output {
        std::vector<unsigned int> LTris;
        glm::vec3 LAABBmin;
        glm::vec3 LAABBmax;
        bool LIsLeaf = false;

        std::vector<unsigned int> RTris;
        glm::vec3 RAABBmin;
        glm::vec3 RAABBmax;
        bool RIsLeaf = false;
    };

    BVH::Partition_output PartitionNode(const BVH::Node parent_node, std::vector<unsigned int>& triangle_indices, const std::vector<Triangle>& triangles, const Heuristic& heuristic);

    BVH::BVH_data construct(const std::vector<Triangle>& triangles, const Heuristic heuristic);
}
#endif // end of BVH_IMPLEMENTATION

