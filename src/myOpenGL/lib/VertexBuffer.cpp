#include "VertexBuffer.h"
#include "OpenGLdebugFuncs.h"

VertexBuffer::VertexBuffer(const void* data, unsigned int vertexCount, unsigned int vertexSize)
: m_VertexCount(vertexCount), m_VertexSize(vertexSize){
	GLCall(glGenBuffers(1, &m_RendererID));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, m_VertexCount * m_VertexSize, data, GL_STATIC_DRAW));
}

VertexBuffer::~VertexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RendererID));
}

void VertexBuffer::SetNewData(const void* data, unsigned int size)
{
	// identical to the consturctor
	GLCall(glGenBuffers(1, &m_RendererID));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
}

void VertexBuffer::Bind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RendererID));
}

void VertexBuffer::Unbind() const 
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}