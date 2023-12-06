#version 460 core

/*
This is a compute shader responsible for the Ray Tracing
- in the first part there are defined struct used throughout the shader
- most of these struct MUST be exactly the same as in the c++ code when being sent by the CPU
*/


// CONSTANTS
const uint AABB_primitives_limit = 4;

// struct alignments: Thanks to https://learnopengl.com/Advanced-OpenGL/Advanced-GLSL

struct BVHNode
{
    uint leaf_primitive_indices[AABB_primitives_limit];     // offset 0 // alignment 4 // total 4 bytes

    vec3 minVec;        // offset 4  // alignment 16 // total 20 bytes
    int child1_idx;     // offset 20 // alignment 4  // total 24 bytes
    vec3 maxVec;        // offset 24 // alignment 16 // total 40 bytes
    int child2_idx;     // offset 40 // alignment 4  // total 44 bytes
};

struct RaytracingMaterial
{
    vec3 color;                 // offset 0   // alignment 16 // size 12 // total 12 bytes
    float emissionStrength;     // offset 12  // alignment 4  // size 4  // total 16 bytes
    vec3 emissionColor;         // offset 16  // alignment 16 // size 12 // total 28 bytes
    float std140padding;        // offset 28  // alignment 4  // size 4  // total 32 bytes
};

struct Sphere {
	RaytracingMaterial material;    // offset 0   // alignment 16 // size 32 // total 32 bytes
	vec3 position;                  // offset 32  // alignment 16 // size 12 // total 44 bytes
	float radius;                   // offset 44  // alignment 4  // size 4  // total 48 bytes
};

struct Triangle
{
    // vertices
    vec3 v1;                    // offset 0   // alignment 16 // size 12 // total 16 bytes 
    float std140padding1;       // offset 12  // alignment 4  // size 4  // total 16 bytes
    vec3 v2;                    // offset 16  // alignment 16 // size 12 // total 32 bytes 
    float std140padding2;       // offset 28  // alignment 4  // size 4  // total 32 bytes
    vec3 v3;                    // offset 32  // alignment 16 // size 12 // total 48 bytes 
    float std140padding3;       // offset 44  // alignment 4  // size 4  // total 48 bytes
    
    //normals
    vec3 NA;                    // offset 48  // alignment 16 // size 12 // total 64 bytes 
    float std140padding4;       // offset 60  // alignment 4  // size 4  // total 64 bytes
    vec3 NB;                    // offset 64  // alignment 16 // size 12 // total 80 bytes 
    float std140padding5;       // offset 76  // alignment 4  // size 4  // total 80 bytes
    vec3 NC;                    // offset 80  // alignment 16 // size 12 // total 96 bytes 
    float std140padding6;       // offset 92  // alignment 4  // size 4  // total 96 bytes
    
    vec3 centroid_vec;          // offset 96  // alignment 16 // size 12 // total 112 bytes
    float std140padding7;       // offset 108 // alignment 4  // size 4  // total 112 bytes

    RaytracingMaterial material;    // offset 112 // alignment 16 // size 32 bytes // total 148 bytes
};

// These actually don't need to be padded (not sent by the CPU)
struct Ray
{
    vec3 origin;
    vec3 dir;
};

struct HitInfo
{
    bool didCollide;
    float dst;
    vec3 hitPoint;
    vec3 normal;
    RaytracingMaterial material;
};

// work group sizes
layout (local_size_x = 8, local_size_y = 4, local_size_z = 1) in;

layout (rgba32f, binding = 0) uniform image2D rayTracingTexture;

//UBOs
layout (std140, binding = 0) uniform uniformParameters {
    uint u_numAccumulatedFrames;    // offset 0  // alignment 4 // total 4 bytes
    uint u_raysPerPixel;            // offset 4  // alignment 4 // total 8 bytes
    uint u_bouncesPerRay;           // offset 8  // alignment 4 // total 12 bytes
    float u_FocalLength;            // offset 12 // alignment 4 // total 16 bytes

    vec3 u_skyboxGroundColor;       // offset 16 // alignment 16 // total 32 bytes
    vec3 u_skyboxHorizonColor;      // offset 32 // alignment 16 // total 48 bytes
    vec3 u_skyboxZenithColor;       // offset 48 // alignment 16 // total 64 bytes                            
    vec3 u_CameraPos;               // offset 64 // alignment 16 // total 80 bytes

    mat4 u_ModelMatrix;             // offset 80 // alignment 16 // total 144 bytes

    bool u_WasInput;                // offset 144 // alignment 4 // total 148 bytes
};

layout (std140, binding = 1) uniform sceneBuffer
{
    Sphere u_Spheres[4];
};

// SSBOs
layout (std140, binding = 3) buffer MESH_buffer
{
    Triangle knight_mesh[];
};

layout (std140, binding = 4) buffer BVH_buffer
{
    BVHNode knight_BVH[];
};


uint getCurrentState(ivec2 texelCoords, int screenWidth)
{
    uint pixelIndex = (uint(texelCoords.y) * uint(screenWidth)) + uint(texelCoords.x);
    return pixelIndex + u_numAccumulatedFrames * 745621; // new state every frame
}

float RandomValue(inout uint state)
{
    state = state * 747796405u + 2891336453u;
    uint result = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    result = (result >> 22u) ^ result;
    return float(result) / 4294967295.0;
}

// mean=0 and sd=1
float RandomValueNormalDistribution(inout uint state)
{
    // Thanks to https://stackoverflow.com/a/6178290
    // And Sebastian Lague
    float theta = 2 * 3.1415926 * RandomValue(state);
    float rho = sqrt(-2 * log(RandomValue(state)));
    return rho * cos(theta);
}

// Calculate random vector on a sphere
vec3 RandomDirection(inout uint state)
{
    // thanks to https://math.stackexchange.com/questions/1585975 and Sebastian Lague
    float x = RandomValueNormalDistribution(state);
    float y = RandomValueNormalDistribution(state);
    float z = RandomValueNormalDistribution(state);
    return normalize(vec3(x, y, z));
}

vec3 RandomDirectionInHemisphere(vec3 normalVector, inout uint state)
{
    vec3 randomDirectionVector = RandomDirection(state);
    if (dot(normalVector, randomDirectionVector) < 0)
    {
        randomDirectionVector = -randomDirectionVector;
    }
    return randomDirectionVector;
}

vec3 GainSkyboxLight(Ray ray)
{
    // Environment Settings
    bool EnvironmentEnabled = true;   
    if (!EnvironmentEnabled)
    {
        return vec3(0.0);
    }
    float skyGradientT = pow(smoothstep(0.0, 0.4, ray.dir.y), 0.35);
    float groundToSkyT = smoothstep(-0.01, 0.0, ray.dir.y);
    vec3 skyGradient = mix(u_skyboxHorizonColor.rgb, u_skyboxZenithColor.rgb, skyGradientT);
    skyGradient = mix(u_skyboxGroundColor.rgb, skyGradient, groundToSkyT);
    return skyGradient;
}

bool RayAABBIntersection(vec3 rayOrigin, vec3 inv_rayDir, vec3 boxMin, vec3 boxMax)
{
    float tMin = 0.0; // Start at zero to ignore intersections behind the origin
    float tMax = 1.0e20; // Large positive value

    for (int i = 0; i < 3; ++i)
    {
        if (abs(inv_rayDir[i]) < 1.0e-8)
        {
            // Ray is parallel to the slab. No hit if origin not within slab
            if (rayOrigin[i] < boxMin[i] || rayOrigin[i] > boxMax[i]) 
                return false;
        }
        else
        {
            float t1 = (boxMin[i] - rayOrigin[i]) * inv_rayDir[i];
            float t2 = (boxMax[i] - rayOrigin[i]) * inv_rayDir[i];

            // Make t1 be intersection with near plane, t2 with far plane
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }

            // Compute the intersection of slab intersection intervals
            tMin = max(tMin, t1);
            tMax = min(tMax, t2);

            // Exit with no collision as soon as slab intersection becomes empty or if intersection is behind the ray's origin
            if (tMin > tMax || tMax < 0.0) 
                return false;
        }
    }
    return tMin <= tMax;
}

// Sebastian Lague - https://youtu.be/Qz0KTGYJtUk?t=321
HitInfo RaySphereIntersection(Ray ray, vec3 spherePosition, float sphereRadius)
{
    HitInfo hitInfo;
    hitInfo.didCollide = false;
    
    vec3 offsetRayOrigin = ray.origin - spherePosition;
    // from the equation: sqrt(length(rayOrigin + rayDirection * dst)) = radius^2
    float a = dot(ray.dir, ray.dir); //(a = 1)
    float b = 2 * dot(offsetRayOrigin, ray.dir);
    float c = dot(offsetRayOrigin, offsetRayOrigin) - sphereRadius * sphereRadius;
    // quadratic discriminant
    float discriminant = b * b - 4 * a * c; // b^2-4ac
    if (discriminant >= 0)
    {
        // nearest sphere intersect
        float dst = (-b - sqrt(discriminant)) / 2;
        
        // if the intersection did not happen behind the camera
        if (dst >= 0)
        {
            hitInfo.didCollide = true;
            hitInfo.dst = dst;
            hitInfo.hitPoint = ray.origin + (ray.dir * dst);
            hitInfo.normal = normalize(hitInfo.hitPoint - spherePosition);
        }
    }
    return hitInfo;
}

// https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d/42752998#42752998
//HitInfo RayTriangleIntersection(Ray ray, Triangle tri)
//{
//    vec3 E1 = tri.B - tri.A;
//    vec3 E2 = tri.C - tri.A;
//    vec3 triNormal = cross(E1, E2);
//
//    float determinant = -dot(ray.dir, triNormal);
//
//    // Early exit if the ray and triangle are nearly parallel
//    if (determinant < 1E-6)
//    {
//        HitInfo hitInfo;
//        hitInfo.didCollide = false;
//        return hitInfo;
//    }
//
//    float invdet = 1.0 / determinant;
//    vec3 AO = ray.origin - tri.A;
//    vec3 DAO = cross(AO, ray.dir);
//
//    float t = dot(AO, triNormal) * invdet;
//    float u = dot(E2, DAO) * invdet;
//    float v = -dot(E1, DAO) * invdet;
//    float w = 1 - u - v;
//
//    // Back-face culling (assuming triangles are consistently oriented)
//    if (t < 0 || u < 0 || v < 0 || w < 0)
//    {
//        HitInfo hitInfo;
//        hitInfo.didCollide = false;
//        return hitInfo;
//    }
//
//    HitInfo hitInfo;
//    hitInfo.didCollide = true;
//    hitInfo.hitPoint = ray.origin + ray.dir * t;
//    hitInfo.normal = normalize(tri.NA * w + tri.NB * u + tri.NC * v);
//    hitInfo.dst = t;
//    return hitInfo;
//}

HitInfo CheckRayCollision(Ray ray)
{
    HitInfo closestHit;
    closestHit.didCollide = false;
    closestHit.dst = 1.0 / 0.0; // infinity
    for (int i = 0; i < 4; i++)
    {
        Sphere sphere = u_Spheres[i];
        HitInfo hitInfo = RaySphereIntersection(ray, sphere.position, sphere.radius);
        if (hitInfo.didCollide && hitInfo.dst < closestHit.dst)
        {
            closestHit = hitInfo;
            closestHit.material = sphere.material;
        }
    }    
    
    //for (int i = 0; i < 456; i++)
    //{
    //    Triangle tri = u_gemMesh[i];
    //    tri.material.color = vec3(214.0f / 255.0f, 182.0f / 255.0f, 105.0f / 255.0f);
    //    tri.material.emissionColor = vec3(0.0f, 0.0f, 0.0f);
    //    tri.material.emissionStrength = 0.0f;
    //    
    //    HitInfo triHitInfo = RayTriangleIntersection(ray, tri);
    //      
    //    if (triHitInfo.didCollide && triHitInfo.dst < closestHit.dst)
    //    {
    //        closestHit = triHitInfo;
    //        closestHit.material = tri.material;
    //    }
    //}
    
    return closestHit;
}

vec3 TraceRay(Ray ray, inout uint state)
{
    vec3 rayColor = vec3(1.0);
    vec3 incomingLight = vec3(0.0);
    
    for (int i = 0; i <= u_bouncesPerRay; i++)
    {
        HitInfo hitInfo = CheckRayCollision(ray);
        if (hitInfo.didCollide)
        {
            ray.origin = hitInfo.hitPoint;
            ray.dir = normalize(hitInfo.normal + RandomDirection(state));
            
            RaytracingMaterial material = hitInfo.material;
            vec3 emittedLight = material.emissionColor * material.emissionStrength;
            
            // this is when the ray gets color
            incomingLight += emittedLight * rayColor;
            rayColor *= material.color;
        }
        else
        {
            incomingLight += GainSkyboxLight(ray) * rayColor;
            break;
        }
    }
    return incomingLight;
}

void main()
{
    ivec2 texelCoords = ivec2(gl_GlobalInvocationID.xy);
    ivec2 dims = imageSize(rayTracingTexture);
    float x = (float(texelCoords.x * 2 - dims.x) / dims.x); // transforms to [-1.0, 1.0]
    float y = (float(texelCoords.y * 2 - dims.y) / dims.x); // deviding by x to keep the ratio
    
    uint state = getCurrentState(texelCoords, dims.x);
    
    Ray ray;
    ray.dir = normalize(vec3(x, y, u_FocalLength));
    ray.dir = (u_ModelMatrix * vec4(ray.dir, 1.0f)).rgb; // apply the rotation transformation of the camera
    ray.origin = u_CameraPos.xyz;
    
    // The actual tracing of the ray
    vec3 tracingResult = vec3(0.0);
    for (int i = 0; i < u_raysPerPixel; i++)
    {
        tracingResult += TraceRay(ray, state);
    }

    // averaging the current frame accumulated pixel color
    tracingResult = tracingResult / u_raysPerPixel;
    vec4 accumulatedColor = imageLoad(rayTracingTexture, texelCoords);
    
    // averaging the previous & current frame pixel color
    float weight = 1.0f / (u_numAccumulatedFrames + 1);
    vec3 outputColor = accumulatedColor.rgb * (1 - weight) + tracingResult * weight;
    
    // setting the texel coords (pixel color texture coordinates)
    imageStore(rayTracingTexture, texelCoords, vec4(outputColor, 1.0f));
};