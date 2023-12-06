#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>

#include "Renderer.h"
#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "FrameBuffer.h"
#include "UniformBuffer.h"
#include "VertexBufferLayout.h"
#include "Shader.h"
#include "Texture.h"

#include "GUI/CameraComponentGUI.h"
#include "GUI/DockspaceGUI.h"
#include "GUI/PerformanceCounterGUI.h"
#include "GUI/InspectorGUI.h"
#include "GUI/SkyBoxGUI.h"

#include "OpenGLdebugFuncs.h"

#include "CameraHandler.hpp"
#include "utility/DeltaTime.h"

#include "scenes/Scene1.hpp"

const int SCREEN_WIDTH = 1920, SCREEN_HEIGHT = 1080;
int main() {
	//WINDOW SETUP
	GLFWwindow* window;
	if (!glfwInit()){
		glfwTerminate();
		ASSERT(false);
		return 1;}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	
	window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, " ", NULL, NULL);
	if (!window) {
		glfwTerminate();
		ASSERT(false);
		return 1;}

	glfwMakeContextCurrent(window);

	std::cout << glGetString(GL_VERSION) << std::endl;
	glfwSwapInterval(false);

	if (glewInit() != GLEW_OK){
		if (!window){
			glfwDestroyWindow(window);
			glfwTerminate();
			ASSERT(false);
			return 1;}}

	{	
		float ASPECT = 16.0/9.0;
		float fov = 90;

		DeltaTime deltaTime;

		Camera camera = Camera(fov, ASPECT, deltaTime);
		CameraHandler cameraHandler(camera);

		SceneData sceneData = getSceneData();
		
		BVH::BVH_data knight_BVH = BVH::construct("src/models/knight.obj", BVH::Heuristic::OBJECT_MEDIAN_SPLIT);

		std::cout << "----Traingle mesh-----" << std::endl;
		for (size_t i = 0; i < knight_BVH.TRIANGLES_size; i++) {
			std::cout << knight_BVH.TRIANGLES[i] << std::endl;
		}

		std::cout << "-------CUBE BVH-------" << std::endl;
		for (size_t i = 0; i < knight_BVH.BVH_size; i++) {
			std::cout << knight_BVH.BVH[i] << std::endl;
		}
		std::cout << "----------------------" << std::endl;
		
		Renderer renderer(sceneData, knight_BVH);
		
		// Setup Dear ImGui context
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();

		glfwSetWindowUserPointer(window, &cameraHandler);
		glfwSetKeyCallback(window, CameraHandler::GLFWKeyCallback);
		glfwSetCursorPosCallback(window, CameraHandler::GLFWMousePositionCallback);
		
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
		io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

		ImGui::StyleColorsDark();

		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init("#version 130");
		


		//////////////////////////
		// ImGui widgets variables
		bool was_ImGui_Input;
		bool shouldPostProcess = false;
		bool shouldAccumulate = true;
		int raysPerPixel = 1;
		int bouncesPerRay = 5;
		
		//skybox
		glm::vec3 SkyGroundColor  = glm::vec3(0.6392156862f, 0.5803921f, 0.6392156862f);
		glm::vec3 SkyColorHorizon = glm::vec3(1.0f, 1.0f, 1.0f);
		glm::vec3 SkyColorZenith  = glm::vec3(0.486274509f, 0.71372549f, 234.0f / 255.0f);
		
		bool show_demo_window = true;
		int totalFrames = 0;
		double prevCamAspect = 0;
		ImVec2 prevViewportWindowSize = ImVec2(0.0f, 0.0f);
		ImVec2 prevViewportWindowPos = ImVec2(0.0f, 0.0f);
		ImVec2 viewportSize, topLeftTextureCoords, bottomLeftTextureCoords;

		while (!glfwWindowShouldClose(window)) {
			deltaTime.update();
			totalFrames += 1;
			was_ImGui_Input = false;

			GLCall(glClear(GL_COLOR_BUFFER_BIT));
			GLCall(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));

			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();
			
			// Widget windows
			genDockspace();
			genInspector(cameraHandler.CameraControllMode);
			component_cameraGUI(camera, was_ImGui_Input, cameraHandler.CameraControllMode, shouldAccumulate, shouldPostProcess, raysPerPixel, bouncesPerRay);
			genSkyboxGUI(SkyGroundColor, SkyColorHorizon, SkyColorZenith, was_ImGui_Input, cameraHandler.CameraControllMode);
			ImGui::ShowDemoWindow(&show_demo_window);
			if (camera.cameraKeybinds.camera_keybind_window_active) { genCameraChangeKeybindSplashScreen(); }
			

			// viewport
			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{ 0, 0 });
			ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar| ImGuiWindowFlags_NoCollapse);
			ImGui::SetNextWindowDockID(ImGui::GetID("MainDockspace"));
			
			ImVec2 viewportWindowSize = ImGui::GetContentRegionAvail();
			// check if the viewport size or aspect ratio changed
			if (viewportWindowSize.x != prevViewportWindowSize.x || viewportWindowSize.y != prevViewportWindowSize.y || camera.getAspect() != prevCamAspect)
			{
				prevViewportWindowSize = viewportWindowSize;
				prevCamAspect = camera.getAspect();

				double parentAspect = double(viewportWindowSize.x) / double(viewportWindowSize.y);

				if (parentAspect <=  camera.getAspect())
				{
					viewportSize.x = ceil(viewportWindowSize.x);
					viewportSize.y = ceil(viewportWindowSize.x / camera.getAspect());
				} 
				else if (parentAspect > camera.getAspect())
				{
					viewportSize.y = ceil(viewportWindowSize.y);
					viewportSize.x = ceil(viewportWindowSize.y * camera.getAspect());
				}
				
				renderer.setViewportSize(glm::vec2(viewportSize.x, viewportSize.y)); // sets the textures

				camera.setScreenDimentions(int(viewportSize.x), int(viewportSize.y));
				was_ImGui_Input = true;
			
				topLeftTextureCoords.x = (viewportWindowSize.x - viewportSize.x) / 2.0f;
				topLeftTextureCoords.y = (viewportWindowSize.y - viewportSize.y) / 2.0f;
				// viewport offset
				topLeftTextureCoords.x += ImGui::GetWindowPos().x;
				topLeftTextureCoords.y += ImGui::GetWindowPos().y;
				
				bottomLeftTextureCoords.x = topLeftTextureCoords.x + viewportSize.x;
				bottomLeftTextureCoords.y = topLeftTextureCoords.y + viewportSize.y;
			}

			ImVec2 viewportWindowPos = ImGui::GetWindowPos();
			if (viewportWindowPos.x != prevViewportWindowPos.x || viewportWindowPos.y != prevViewportWindowPos.y)
			{
				prevViewportWindowPos = viewportWindowPos;

				topLeftTextureCoords.x = (viewportWindowSize.x - viewportSize.x) / 2.0f;
				topLeftTextureCoords.y = (viewportWindowSize.y - viewportSize.y) / 2.0f;
				// viewport offset
				topLeftTextureCoords.x += viewportWindowPos.x;
				topLeftTextureCoords.y += viewportWindowPos.y;

				bottomLeftTextureCoords.x = topLeftTextureCoords.x + viewportSize.x;
				bottomLeftTextureCoords.y = topLeftTextureCoords.y + viewportSize.y;
			}

			camera.Update();

			static bool wasInput;
			wasInput = (camera.wasCameraInput || was_ImGui_Input || !shouldAccumulate);
			static unsigned int m_NumAccumulatedFrames;
			if (wasInput) { m_NumAccumulatedFrames = 0; }
			else { m_NumAccumulatedFrames++; }

			renderer.BeginComputeRtxStage();
			renderer.rtx_uniform_parameters.numAccumulatedFrames = m_NumAccumulatedFrames;
			renderer.rtx_uniform_parameters.raysPerPixel = raysPerPixel;
			renderer.rtx_uniform_parameters.bouncesPerRay = bouncesPerRay;
			renderer.rtx_uniform_parameters.FocalLength = camera.focalLength;
			renderer.rtx_uniform_parameters.skyboxGroundColor = SkyGroundColor;
			renderer.rtx_uniform_parameters.skyboxHorizonColor = SkyColorHorizon;
			renderer.rtx_uniform_parameters.skyboxZenithColor = SkyColorZenith;
			renderer.rtx_uniform_parameters.CameraPos = camera.posVec;
			renderer.rtx_uniform_parameters.ModelMatrix = glm::mat4(camera.getModelMatrix());
			renderer.rtx_uniform_parameters.WasInput = wasInput;
			ComputeTexture* outputTexture = renderer.RenderComputeRtxStage();

			renderer.BeginComputePostProcStage();
			renderer.postProcessing_uniform_parameters.numAccumulatedFrames = m_NumAccumulatedFrames;
			ComputeTexture* postProcOutput = renderer.RenderComputePostProcStage();

			ImDrawList* drawList = ImGui::GetWindowDrawList();
			drawList->AddImage((ImTextureID)postProcOutput->ID(), topLeftTextureCoords, bottomLeftTextureCoords, {0, 1}, {1, 0});
			ImGui::PopStyleVar();
			ImGui::End();

			genPerformanceCounter();
			camera.ResetFlags();
			
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
			glfwSwapBuffers(window);
			glfwPollEvents();
		}
	}
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();
	return 0;
}