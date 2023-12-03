#pragma once
#include <imgui/imgui.h>
#include <glm/glm.hpp>

void genSkyboxGUI(glm::vec3& GroundColor, glm::vec3& SkyColorHorizon, glm::vec3& SkyColorZenith, bool& was_IMGUI_input, bool disabled)
{
	if (disabled) { ImGui::BeginDisabled(); }
	ImGui::Begin("Skybox Settings");
	if (ImGui::ColorEdit3("GroundColor", &GroundColor.x)) {
		was_IMGUI_input = true;
	}
	if (ImGui::ColorEdit3("SkyColorHorizon", &SkyColorHorizon.x)) {
		was_IMGUI_input = true;
	}
	if (ImGui::ColorEdit3("SkyColorZenith", &SkyColorZenith.x)) {
		was_IMGUI_input = true;
	}
	ImGui::End();
	if (disabled) { ImGui::EndDisabled(); }
}