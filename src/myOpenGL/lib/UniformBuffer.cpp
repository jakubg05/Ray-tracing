#include "UniformBuffer.h"

#include "OpenGLdebugFuncs.h"

UniformBuffer::UniformBuffer(unsigned int size, unsigned int bindingPoint)
: m_RendererID(0){
	GLCall(glGenBuffers(1, &m_RendererID));
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, m_RendererID));
	GLCall(glBufferData(GL_UNIFORM_BUFFER, size, nullptr, GL_STATIC_DRAW));
	GLCall(glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, m_RendererID));
}

UniformBuffer::~UniformBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RendererID));
	delete[] m_DataPointer;
	m_DataPointer = nullptr;
}

void UniformBuffer::Bind() const
{
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, m_RendererID));
}

void UniformBuffer::Unbind() const
{
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, 0));
}

void UniformBuffer::UpdateData(unsigned int offset, const void* newData, unsigned int dataSize) {
	glBufferSubData(GL_UNIFORM_BUFFER, offset, dataSize, newData);
}
