#include "Texture.h"
#include <iostream>

Texture::Texture()
	:m_Width(0), m_Height(0), m_RendererID(0)
{
	GLCall(glGenTextures(1, &m_RendererID));
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));

	// texture parameters
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}

Texture::Texture(int width, int height)
: m_Width(width), m_Height(height), m_RendererID(0) {
	int boundFBO;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &boundFBO);
	if (boundFBO == 0)
	{
		std::cout << "[WARNING] Texture.cpp - can not copy texture from the default FBO" << std::endl;
	}

	//IMPORTANT: the source texture has to be attached to a bound frame buffer for this to work
	GLCall(glGenTextures(1, &m_RendererID));
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));

	// texture parameters
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

	GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_Width, m_Height, 0, GL_RGBA, GL_FLOAT, 0));
	GLCall(glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 0, 0, m_Width, m_Height, 0));

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}

Texture::~Texture()
{
	GLCall(glDeleteTextures(1, &m_RendererID));
}

void Texture::copyFBOtexture()
{
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));

	GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_Width, m_Height, 0, GL_RGBA, GL_FLOAT, 0));
	GLCall(glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 0, 0, m_Width, m_Height, 0));

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}

void Texture::setActiveTextureSlot(unsigned int slot)
{
	/*
	Texture slot is needed when we pass a texture to a sampler uniform
	Essentially, the sampler2D uniform always samples a specific slot (unsigned int passed to the uniform - 0, 1, 2)
	Each slot or texture unit maintains its own binding state. So, if you change the active slot and bind a different texture to it, 
	the previous texture bound to the first slot remains bound to that slot, and the new texture is bound to the newly specified slot
	 */
	GLCall(glActiveTexture(GL_TEXTURE0 + slot));
}

void Texture::setSize(int width, int height)
{
	m_Width = width;
	m_Height = height;
	Bind();
	// creates an emply buffer since we dont pass a texture - allocates space for a texture of the size of the viewport
	GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_Width, m_Height, 0, GL_RGBA, GL_FLOAT, 0));
	Unbind();
}

void Texture::Bind() const
{
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));
}

void Texture::Unbind() const
{
	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}