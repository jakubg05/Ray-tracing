#include "ObjParser.h"


std::ostream& operator<<(std::ostream& os, const Triangle& triangle)
{
    os << "(" 
        << "[(" << triangle.v1.x << ", " << triangle.v1.y << ", " << triangle.v1.z << "), "
        << "(" << triangle.v2.x << ", " << triangle.v2.y << ", " << triangle.v2.z << "), "
        << "(" << triangle.v3.x << ", " << triangle.v3.y << ", " << triangle.v3.z << ")] "
        ", (" 
        << triangle.centroid.x << ", " << triangle.centroid.y << ", " << triangle.centroid.z << 
        ")),";
    return os;
}

void loadMesh(std::string filePath, std::vector<Triangle>& mesh, unsigned int& numTriangles)
{
    std::ifstream file(filePath);
    std::string line;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> vertexNormals;

    if (file.is_open()) {
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string identificator;

            iss >> identificator;
            if (identificator == "v")
            {
                glm::vec3 vertex;
                iss >> vertex.x >> vertex.y >> vertex.z;
                vertices.push_back(vertex);
            }
            else if (identificator == "vn")
            {
                glm::vec3 vertexNormal;
                iss >> vertexNormal.x >> vertexNormal.y >> vertexNormal.z;
                vertexNormals.push_back(vertexNormal);
            }
            else if (identificator == "f")
            {
                int v1, v2, v3;
                int vt1, vt2, vt3;
                int vn1, vn2, vn3;
                char slash;
                char space;
                //iss >>  v1 >> slash  >> slash >> vn1 >>
                //        v2 >> slash  >> slash >> vn2 >>
                //        v3 >> slash  >> slash >> vn3;

                iss >> v1 >> slash >> vt1 >> slash >> vn1 >>
                    v2 >> slash >> vt2 >> slash >> vn2 >>
                    v3 >> slash >> vt3 >> slash >> vn3;

                Triangle triangle;
                triangle.v1 = vertices[v1 - 1];
                triangle.v2 = vertices[v2 - 1];
                triangle.v3 = vertices[v3 - 1];

                triangle.NA = vertexNormals[vn1 - 1];
                triangle.NB = vertexNormals[vn2 - 1];
                triangle.NC = vertexNormals[vn3 - 1];

                triangle.centroid = (triangle.v1 + triangle.v2 + triangle.v3) / 3.0f; // average of all 3 vertices

                mesh.push_back(triangle);
                numTriangles++;
            }
        }
        file.close();
    }
    else {
        std::cout << "Unable to load file " + filePath << std::endl;
    }
}

// constructor
BVH::Node::Node(glm::vec3 minVec, glm::vec3 maxVec)
    : minVec(minVec), maxVec(maxVec), child1_idx(-1), child2_idx(-1)
{
    // populate the leaf_primitive_indices with -1 
    for (unsigned int i = 0; i < AABB_primitives_limit; i++)
    {
        leaf_primitive_indices[i] = -1;
    }
}

namespace BVH {
    std::ostream& operator<<(std::ostream& os, const Node& node) {
        os <<
            "{ \"minVec\": (" << node.minVec.x << ", " << node.minVec.y << ", " << node.minVec.z << ")," <<
            " \"maxVec\": (" << node.maxVec.x << ", " << node.maxVec.y << ", " << node.maxVec.z << ") },";
        return os;
    }
}

glm::vec3 BVH::minCorner(const glm::vec3& current_min, const glm::vec3& vertex) {
    return glm::vec3(std::min(current_min.x, vertex.x), std::min(current_min.y, vertex.y), std::min(current_min.z, vertex.z));
}

glm::vec3 BVH::maxCorner(const glm::vec3& current_max, const glm::vec3& vertex) {
    return glm::vec3(std::max(current_max.x, vertex.x), std::max(current_max.y, vertex.y), std::max(current_max.z, vertex.z));
}

void BVH::computeAABB(const std::vector<unsigned int>& triangle_indices, const std::vector<Triangle>& triangle_mesh, glm::vec3& minVec, glm::vec3& maxVec)
{
    minVec = glm::vec3(std::numeric_limits<float>::infinity());
    maxVec = glm::vec3(-std::numeric_limits<float>::infinity());

    for (const unsigned int& tri_index : triangle_indices)
    {
        const Triangle& triangle = triangle_mesh[tri_index]; // stands for current triangle

        minVec = minCorner(minVec, triangle.v1);
        maxVec = maxCorner(maxVec, triangle.v1);

        minVec = minCorner(minVec, triangle.v2);
        maxVec = maxCorner(maxVec, triangle.v2);

        minVec = minCorner(minVec, triangle.v3);
        maxVec = maxCorner(maxVec, triangle.v3);
    }
}

BVH::Node BVH::init(const std::vector<unsigned int>& triangle_indices, const std::vector<Triangle> triangle_mesh) {
    glm::vec3 minVec, maxVec;
    BVH::computeAABB(triangle_indices, triangle_mesh, minVec, maxVec);
    return BVH::Node(minVec, maxVec);
}

BVH::Partition_output BVH::PartitionNode(const BVH::Node parent_node, std::vector<unsigned int>& triangle_indices, const std::vector<Triangle>& triangles, const Heuristic& heuristic) {

    BVH::Partition_output output;

    float parent_AABB_Width = parent_node.maxVec.x - parent_node.minVec.x;
    float parent_AABB_Height = parent_node.maxVec.y - parent_node.minVec.y;
    float parent_AABB_Depth = parent_node.maxVec.z - parent_node.minVec.z;

    float* sorted_axis_sizes[3]{ &parent_AABB_Width, &parent_AABB_Height, &parent_AABB_Depth };

    // sorting the addresses by the values they point do
    std::sort(sorted_axis_sizes, sorted_axis_sizes + 3, [&](float* a, float* b)
        {
            return *a > *b;
        }
    );

    /*
        Lambda to split the box exactly in the middle
        returns an integer value - depending on the axis it is the given component of the vector
    */
    auto spatial_middle_split = [&](unsigned int axis) {
        switch (axis) {
        case 0:
            return parent_node.minVec.x + parent_AABB_Width / 2;
        case 1:
            return parent_node.minVec.y + parent_AABB_Height / 2;
        case 2:
            return parent_node.minVec.z + parent_AABB_Depth / 2;
        default:
            std::cout << "BVH spatial_middle_split() - axis can only be 0, 1, 2" << std::endl;
        }
    };

    /*
        Lambda to split the box based on the median
        calculates the median x or y or z based on the axis of all the centroids in the box
        returns the median, integer value - depending on the axis it is the given component of the vector
    */
    auto object_median_split = [&](unsigned int axis) {
        std::vector<float> centroid_axis_components;
        for (const unsigned int& triangle_index : triangle_indices)
        {
            centroid_axis_components.push_back(triangles[triangle_index].centroid[axis]);
        }

        std::sort(centroid_axis_components.begin(), centroid_axis_components.end());

        float median;
        unsigned int length = centroid_axis_components.size();
        if (centroid_axis_components.size() % 2 == 0) { // if even
            median = (centroid_axis_components[length / 2 - 1] + centroid_axis_components[length / 2]) / 2;
        }
        else { // if odd
            median = centroid_axis_components[length / 2];
        }
        return median;
    };

    /*
        Lambda to split the triangles into 2 std::vectors it chooses where to
        put them based on the division axis defined before
    */
    auto partition_triangles = [&](unsigned int division_axis, float axis_value)
        // Note - Division axis can be 0, 1 or 2 corresponding to the x, y, z vector components
    {
        for (unsigned int triangle_index : triangle_indices)
        {
            if (triangles[triangle_index].centroid[division_axis] < axis_value) {
                output.LTris.push_back(triangle_index);
            }
            else {
                output.RTris.push_back(triangle_index);
            }
        }
    };

    /*
        We assume that the a single axis might not be able to split the triangles
        We schedule the split by all 3 axes starting with the longest
        Once the triangles have been divided we check if any side is empty
        in which case we need to continue and split with a different axis.
        Once we do get a successful split meaning no side is empty
        we break out of the loop
    */
    unsigned int axis;
    for (float* current_axis_ptr : sorted_axis_sizes)
    {
        if (current_axis_ptr == &parent_AABB_Width) {
            axis = 0;
        }
        else if (current_axis_ptr == &parent_AABB_Height) {
            axis = 1;
        }
        else {
            axis = 2;
        }

        if (heuristic == BVH::Heuristic::OBJECT_MEDIAN_SPLIT) {
            partition_triangles(axis, object_median_split(axis));
        }
        else if (heuristic == BVH::Heuristic::SPATIAL_MIDDLE_SPLIT) {
            partition_triangles(axis, spatial_middle_split(axis));
        }

        if (!(output.LTris.empty() || output.RTris.empty())) {
            break;
        }
        output = BVH::Partition_output(); // reset the output
    }

    /*
        If we are very unlucky, not a single axis was able to split the triangles into 2 sides
        in this case we simply put half of them in one list and the second half to the other list
        - at this point we dont really care, that they might overlap because since this will very
            rarely happen and the bigger priority is to have a specific number of primitives in AABB
    */
    if ((output.LTris.empty() && output.RTris.size() > AABB_primitives_limit) || (output.RTris.empty() && output.LTris.size() > AABB_primitives_limit)) {
        output = BVH::Partition_output();
        for (unsigned int i = 0; i < triangle_indices.size() / 2; i++)
        {
            output.LTris.push_back(triangle_indices[i]);
        }
        for (unsigned int i = triangle_indices.size() / 2 + 1; i < triangle_indices.size(); i++)
        {
            output.RTris.push_back(triangle_indices[i]);
        }
    }

    /*
        Making the AABBs for the two sides
        based on the number on each side we set the isLeaf bool
    */
    BVH::computeAABB(output.LTris, triangles, output.LAABBmin, output.LAABBmax);
    if (output.LTris.size() <= AABB_primitives_limit) {
        output.LIsLeaf = true;
    }

    BVH::computeAABB(output.RTris, triangles, output.RAABBmin, output.RAABBmax);
    if (output.RTris.size() <= AABB_primitives_limit) {
        output.RIsLeaf = true;
    }

    return output;
}

BVH::BVH_data BVH::construct(const std::vector<Triangle>& triangles, const Heuristic heuristic) {
    std::vector<unsigned int> triangle_indices;
    for (unsigned int i = 0; i < triangles.size(); i++) {
        triangle_indices.push_back(i);
    }

    BVH::Node root_node = BVH::init(triangle_indices, triangles);
    std::vector<BVH::Node> BVH;
    BVH.push_back(root_node);

    std::queue<unsigned int> queue;
    std::queue<std::vector<unsigned int>> index_queue;

    queue.push(0);
    index_queue.push(triangle_indices);

    while (!queue.empty()) {
        unsigned int current_node_idx = queue.front();
        queue.pop();
        std::vector<unsigned int> current_tri_idxs = index_queue.front();
        index_queue.pop();
        BVH::Partition_output output = PartitionNode(BVH[current_node_idx], current_tri_idxs, triangles, heuristic);
        unsigned int BVH_len = BVH.size();

        Node Lnode(output.LAABBmin, output.LAABBmax);
        if (output.LIsLeaf) {
            for (unsigned int i = 0; i < output.LTris.size(); i++) {
                if (i <= BVH::AABB_primitives_limit - 1) { // protection if somehow there are more cell primitives than 4 in a leaf cell which is unlikely but still
                    Lnode.leaf_primitive_indices[i] = output.LTris[i];
                }
            }
        }
        else {
            queue.push(BVH_len);
            index_queue.push(output.LTris);
        }
        BVH[current_node_idx].child1_idx = BVH_len; // index if the first child in the BVH_index_array
        BVH.push_back(Lnode);



        Node Rnode(output.RAABBmin, output.RAABBmax);
        if (output.RIsLeaf) {
            for (unsigned int i = 0; i < output.RTris.size(); i++) {
                if (i <= BVH::AABB_primitives_limit - 1) { // protection if somehow there are more cell primitives than 4 in a leaf cell which is unlikely but still
                    Rnode.leaf_primitive_indices[i] = output.RTris[i];
                }
                else {
                    std::cout << "Was overflow" << std::endl;
                }
            }
        }
        else {
            queue.push(BVH_len + 1);
            index_queue.push(output.RTris);
        }
        BVH[current_node_idx].child2_idx = BVH_len + 1; // index if the first child in the BVH_index_array
        BVH.push_back(Rnode);

    }

    BVH::Node* BVH_array = new BVH::Node[BVH.size()];
    for (size_t i = 0; i < BVH.size(); i++) {
        BVH_array[i] = BVH[i];
    }

    BVH_data bvh_data;
    bvh_data.BVH = BVH_array;
    bvh_data.size = BVH.size();

    return  bvh_data;
}


