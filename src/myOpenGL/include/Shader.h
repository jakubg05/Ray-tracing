#pragma once
#include <iostream>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

struct ShaderSourceCode
{
	std::string vertexSource;
	std::string fragmentSource;
};

class Shader
{
private:
	std::unordered_map<std::string, int> m_UniformLocationCache;
	std::string m_Filepath;
	unsigned int m_RendererID;

public:
	// CONSTRUCTOR AND DECONSTURCTOR
	Shader(){}
	Shader(const std::string& filepath);
	~Shader();

	void Bind();
	void Unbind();

	void SetUniform1f(const std::string& name, float value);
	void SetUniformMat3(const std::string& name, glm::mat3 matrix);
	void SetUniformVec3(const std::string& name, glm::vec3 vector);
	void SetUniform1i(const std::string& name, int value);
	void SetUniformBool(const std::string& name, bool value);
	void SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3);

private:
	unsigned int GetUniformLocation(const std::string& name);

	// reads the file and returns a ShaderSourceCode object which has .vertex and .fragment attributes
	ShaderSourceCode ParseShader(const std::string& filepath);
	// combines - compiles and links shaders and returns the shader id (uses Shader::CompileShader())
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	// compiles singular shader (vertex, fragment)
	unsigned int CompileShader(unsigned int type, const std::string& source);

};