#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>

// dont modify

struct RaytracingMaterial
{
    glm::vec3 color;
    glm::vec3 emissionColor;
    float emissionStrength;
};

struct Sphere {
    glm::vec3 position;
    float radius;
    RaytracingMaterial material;
};

struct SceneData {
    const void* sceneObjects;
    size_t size;
};



SceneData getSceneData() {
    Sphere* spheres = new Sphere[4];

    // Here create any number of spheres
    Sphere sphere;
    sphere.position = glm::vec3(0.0f, 0.0f, 0.0f);
    sphere.radius = 1.0f;
    sphere.material.color = glm::vec3(0.0f, 0.0f, 0.0f); 
    sphere.material.emissionColor = glm::vec3(1.0f, 1.0f, 1.0f);
    sphere.material.emissionStrength = 10.0f;
    
    spheres[0] = sphere;


    SceneData sceneData;
    sceneData.sceneObjects = spheres;
    sceneData.size = 4 * sizeof(Sphere);

    return sceneData;
}