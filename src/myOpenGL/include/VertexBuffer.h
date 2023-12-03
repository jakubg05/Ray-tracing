#pragma once

class VertexBuffer
{
private:
	unsigned int m_RendererID, m_VertexCount, m_VertexSize;


public:
	// CONSTRUCTOR
	VertexBuffer(){}
	VertexBuffer(const void* data, unsigned int vertexCount, unsigned int vertexSize);
	~VertexBuffer();

	void SetNewData(const void* data, unsigned int size);
	
	void Bind() const;
	void Unbind() const;

	inline unsigned int getVertexCount() const { return m_VertexCount; }
	inline unsigned int getVertexSize() const { return m_VertexSize; }
};