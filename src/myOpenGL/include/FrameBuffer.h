#pragma once
#include "Texture.h"

class FrameBuffer 
{
public:
	FrameBuffer();
	~FrameBuffer();

	void Bind();
	void Unbind();
	void attachTexture(unsigned int attachPoint, Texture& texture);
	void setActiveAttachmentPoint(unsigned int attachmentPoint);

private:
	unsigned int m_RendererID;
};