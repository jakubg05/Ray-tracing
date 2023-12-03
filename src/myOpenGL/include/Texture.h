#pragma once
#include "OpenGLdebugFuncs.h"

class Texture
{
protected:
	int m_Width, m_Height;
private:
	unsigned int m_RendererID;
public:
	Texture();
	Texture(int width, int height);
	~Texture();

	void copyFBOtexture();
	static void setActiveTextureSlot(unsigned int slot);
	void setSize(int width, int height);
	void Bind() const;
	void Unbind() const;

	inline int GetWidth() const { return m_Width; }
	inline int GetHeight() const { return m_Height; }
	inline int ID() { return m_RendererID; }
};