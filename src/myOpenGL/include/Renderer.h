#pragma once
#include <GL/glew.h>

#include "UniformBuffer.h"

#include "ComputeShader.h"
#include "ComputeTexture.h"

#include "imgui/imgui.h"

#include "OpenGLdebugFuncs.h"
#include "camera/Camera.hpp"
#include "utility/ObjParser.h"



struct SceneData {
	const void* sceneObjects;
	size_t size;
	int numberOfObjects;
};


// all the uniforms passed to the compute shaders using the std140 standard (depending on the stage)
struct rtx_parameters_uniform_struct {
	unsigned int numAccumulatedFrames;      // offset 0  // alignment 4 // total 4 bytes
	unsigned int raysPerPixel;              // offset 4  // alignment 4 // total 8 bytes
	unsigned int bouncesPerRay;             // offset 8  // alignment 4 // total 12 bytes
	float FocalLength;						// offset 12 // alignment 4 // total 16 bytes

	glm::vec3 skyboxGroundColor;			// offset 16 // alignment 16 // total 32 bytes
	glm::vec3 skyboxHorizonColor;			// offset 32 // alignment 16 // total 48 bytes
	glm::vec3 skyboxZenithColor;			// offset 48 // alignment 16 // total 64 bytes                            
	glm::vec3 CameraPos;					// offset 64 // alignment 16 // total 80 bytes

	glm::mat4 ModelMatrix;					// offset 80 // alignment 16 // total 144 bytes

	bool WasInput;							// offset 144 // alignment 4 // total 148 bytes                        
};

struct postProcessing_parameters_uniform_struct {
	unsigned int numAccumulatedFrames;
};


class Renderer
{
private:
	SceneData m_Scene;

	glm::vec2 m_ViewportSize;

	void initComputeRtxStage();
	void initComputePostProcStage();

	// unifom buffer object setup and functions
	unsigned int rtx_parameters_UBO_ID, sphereBuffer_UBO_ID, postProcessing_parameters_UBO_ID, BVH_SSBO_ID;

	void configure_rtx_parameters_UBO_block();
	void update_rtx_parameters_UBO_block();

	void configure_sphereBuffer_UBO_block();
	void update_sphereBuffer_UBO_block();

	void configure_postProcessing_parameters_UBO_block();
	void update_postProcessing_parameters_UBO_block();

	void configure_BVHMesh_SSBO_block();
	void update_BVHMesh_SSBO_block();


public:
	Renderer(SceneData& scene, std::vector<Triangle>& mesh, BVH::Node* BVH_of_mesh);
	~Renderer();

	void setViewportSize(glm::vec2 viewportSize);

	void BeginComputeRtxStage();
	ComputeTexture* RenderComputeRtxStage();
	rtx_parameters_uniform_struct rtx_uniform_parameters;

	void BeginComputePostProcStage();
	ComputeTexture* RenderComputePostProcStage();
	postProcessing_parameters_uniform_struct postProcessing_uniform_parameters;

private:
	// compute rtx stage
	ComputeTexture* computeRtxTexture;
	ComputeShader* computeRtxShader;

	// compute post processing stage
	ComputeTexture* computePostProcTexture;
	ComputeShader* computePostProcShader;

	std::vector<Triangle> mesh;
	BVH::Node* BVH_of_mesh;
};