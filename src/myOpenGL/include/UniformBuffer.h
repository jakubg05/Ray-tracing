#pragma once

class UniformBuffer
{
private:
	unsigned int m_RendererID;
	const void* m_DataPointer;
	unsigned int bindingPoint;

public:
	// consturctor
	UniformBuffer(unsigned int size, unsigned int bindingPoint);
	~UniformBuffer();

	void Bind() const;
	void Unbind() const;
	void UpdateData(unsigned int offset, const void* newData, unsigned int dataSize);
};