#include "FrameBuffer.h"
#include <iostream>

FrameBuffer::FrameBuffer()
{
	GLCall(glGenFramebuffers(1, &m_RendererID));
	GLCall(glBindFramebuffer(GL_FRAMEBUFFER, m_RendererID));

}

FrameBuffer::~FrameBuffer()
{
	GLCall(glDeleteFramebuffers(1, &m_RendererID));
}

void FrameBuffer::Bind()
{
	GLCall(glBindFramebuffer(GL_FRAMEBUFFER, m_RendererID));
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "[ERROR] Problem with FrameBuffer status" << std::endl;
		ASSERT(false);
	}
}

void FrameBuffer::Unbind()
{
	GLCall(glBindFramebuffer(GL_FRAMEBUFFER, 0));
}

void FrameBuffer::attachTexture(unsigned int attachPoint, Texture& texture)
{
	GLCall(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + attachPoint, GL_TEXTURE_2D, texture.ID(), 0));
}

void FrameBuffer::setActiveAttachmentPoint(unsigned int attachmentPoint)
{
	GLCall(glDrawBuffers(1, &attachmentPoint));
}
