#shader vertex										
#version 460 core						
												
layout(location = 0) in vec2 position;
layout(location = 1) in vec3 rayDir;

out vec3 interpolatedRayDir;
out vec2 v_Pos;
out flat int v_numAccumulatedFrames;

uniform int u_numAccumulatedFrames;
uniform float u_GLPointSize;

// runs for every pixel
void main()
{
    gl_Position = vec4(position, 0.0f, 1.0f);
    gl_PointSize = u_GLPointSize;
    interpolatedRayDir = rayDir;
    v_numAccumulatedFrames = u_numAccumulatedFrames;
    v_Pos = position;
};



#shader fragment												
#version 460 core

// for UBO
struct RaytracingMaterial
{
    vec3 color;
    float emissionStrength;
    vec3 emissionColor;
    float std140padding;
    
};
struct Sphere
{
    vec3 position;
    float radius;
    RaytracingMaterial material;
};

struct Triangle
{
    // vertices
    vec3 A;
    float padding1; // Add padding for alignment
    vec3 B;
    float padding2; // Add padding for alignment
    vec3 C;
    float padding3; // Add padding for alignment

    // normals
    vec3 NA;
    float padding4; // Add padding for alignment
    vec3 NB;
    float padding5; // Add padding for alignment
    vec3 NC;
    float padding6; // Add padding for alignment

    RaytracingMaterial material; // already padded correctly
};

// UBO
layout(std140, binding = 0) uniform sceneBuffer
{
    Sphere u_Spheres[4];
};

uniform int u_numGemMeshTri;

layout(std140, binding = 1) uniform triangleMeshGem
{
    Triangle u_gemMesh[456];
};

in vec3 interpolatedRayDir;
in flat int v_numAccumulatedFrames;
in vec2 v_Pos;

layout(location = 0) out vec4 color;


// UNIFORMS
uniform mat3 u_ModelMatrix;
uniform vec3 u_OffsetVector;
uniform float u_sWidth;
uniform float u_sHeight;
uniform int u_numberOfSceneObjects;
uniform bool u_wasInput;
uniform int u_raysPerPixel;
uniform int u_bouncesPerRay;

//Skybox
uniform vec3 u_GroundColor;
uniform vec3 u_SkyColorHorizon;
uniform vec3 u_SkyColorZenith;

uniform sampler2D u_AccumulateTexture;


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






float RandomValue(inout uint state)
{
    state = state * 747796405u + 2891336453u;
    uint result = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    result = (result >> 22u) ^ result;
    return float(result) / 4294967295.0;
}

uint getCurrentState()
{
    vec2 pixelPos = gl_FragCoord.xy;
    uint pixelIndex = (uint(pixelPos.y) * uint(u_sWidth)) + uint(pixelPos.x);
    return pixelIndex + v_numAccumulatedFrames * 745621;
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
// Note: there are many alternative methods for computing this,
// with varying trade-offs between speed and accuracy
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
    vec3 skyGradient = mix(u_SkyColorHorizon, u_SkyColorZenith, skyGradientT);
    skyGradient = mix(u_GroundColor, skyGradient, groundToSkyT);
    
    return skyGradient;
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
HitInfo RayTriangleIntersection(Ray ray, Triangle tri)
{
    vec3 E1 = tri.B - tri.A;
    vec3 E2 = tri.C - tri.A;
    vec3 triNormal = cross(E1, E2);
    
    // for now - no phong shading the normals are the same at the vertex as the triangle plane
    tri.NA = triNormal;
    tri.NB = triNormal;
    tri.NC = triNormal;
    
    float determinant = -dot(ray.dir, triNormal);
    float invdet = 1.0 / determinant;
    vec3 AO = ray.origin - tri.A; // vector from ray origin to the point A
    vec3 DAO = cross(AO, ray.dir);
    
    float t = dot(AO, triNormal) * invdet;
    float u = dot(E2, DAO) * invdet;
    float v = -dot(E1, DAO) * invdet;
    float w = 1 - u - v;
    
    HitInfo hitInfo;
    hitInfo.didCollide = determinant >= 1E-6 && t >= 0 && u >= 0 && v >= 0 && w >= 0;
    hitInfo.hitPoint = ray.origin + ray.dir * t;
    hitInfo.normal = normalize(tri.NA * w + tri.NB * u + tri.NC * v);
    hitInfo.dst = t;
    return hitInfo;
}

HitInfo CheckRayCollision(Ray ray)
{
    HitInfo closestHit;
    closestHit.didCollide = false;
    closestHit.dst = 1.0 / 0.0; // infinity
    for (int i = 0; i < u_numberOfSceneObjects; i++)
    {
        Sphere sphere = u_Spheres[i];
        HitInfo hitInfo = RaySphereIntersection(ray, sphere.position, sphere.radius);
        if (hitInfo.didCollide && hitInfo.dst < closestHit.dst)
        {
            closestHit = hitInfo;
            closestHit.material = sphere.material;
        }
    }
    
    
    for (int i = 0; i < 456; i++)
    {
        Triangle tri = u_gemMesh[i];
        tri.material.color = vec3(214.0f / 255.0f, 182.0f / 255.0f, 105.0f / 255.0f);
        tri.material.emissionColor = vec3(0.0f, 0.0f, 0.0f);
        tri.material.emissionStrength = 0.0f;
        tri.A.x += 0;
        tri.B.x += 0;
        tri.C.x += 0;
        
        tri.A.y += -1.1;
        tri.B.y += -1.1;
        tri.C.y += -1.1;
        
        tri.A.z += 5;
        tri.B.z += 5;
        tri.C.z += 5;
        HitInfo triHitInfo = RayTriangleIntersection(ray, tri);
          
        if (triHitInfo.didCollide && triHitInfo.dst < closestHit.dst)
        {
            closestHit = triHitInfo;
            closestHit.material = tri.material;
        }
    }
    
    return closestHit;
}

vec3 TraceRay(Ray ray, inout uint state)
{
    vec3 incomingLight = vec3(0.0);
    vec3 rayColor = vec3(1.0);
    
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


// also runs for evey pixel
void main()
{
    uint state = getCurrentState();
    
    Ray ray;
    /*
    Basically the offset represents the current position of the
    camera so the initial rayOrigin is the camera position
    */
    ray.origin = u_OffsetVector;
    ray.dir = u_ModelMatrix * interpolatedRayDir;
	
    vec3 totalIncomingLight = vec3(0.0);
    for (int i = 0; i < u_raysPerPixel; i++)
    {
        totalIncomingLight += TraceRay(ray, state);
    }
    
    
    vec3 newRender = totalIncomingLight / u_raysPerPixel;
    
    
    // accumulate frames
    if (!u_wasInput)
    {
        vec3 accumulatedRender = texture(u_AccumulateTexture, vec2(gl_FragCoord.x / u_sWidth, gl_FragCoord.y / u_sHeight)).rgb;
        // v_Pos ranges from -1 to +1 in both x and y
        // we need to sample the coordinates in the range of 0 to +1 in x and y - so we //scale & translate//
        // gl_FragCoord seems to work better
        //float weight = 1.0 / (float(v_numAccumulatedFrames) + 1);
        //vec3 accumulatedAverage = accumulatedRender * (1 - weight) + newRender * weight;
        accumulatedRender = accumulatedRender + newRender;
        
        color = vec4(accumulatedRender, 1.0f);
    }
    // there was input
    // render just this frame
    else
    {
        color = vec4(newRender, 1.0f);
    }
};