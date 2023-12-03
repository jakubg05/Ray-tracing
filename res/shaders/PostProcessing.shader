#shader vertex

#version 460 core

layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTexCoords;

out vec2 TexCoords;

out flat int v_numAccumulatedFrames;

uniform int u_numAccumulatedFrames;

void main()
{
    gl_Position = vec4(aPos.x, aPos.y, 0.0f, 1.0f);
    TexCoords = aTexCoords;
    v_numAccumulatedFrames = u_numAccumulatedFrames;
}


#shader fragment

#version 460 core

out vec4 FragColor;
in vec2 TexCoords;
in flat int v_numAccumulatedFrames;

layout(rgba32f, binding = 0) uniform image2D screen;
uniform bool u_wasInput;



void main()
{ 
    // invert colors
    vec3 color = imageLoad(screen, TexCoords);
    //color.x = 1.0 - color.x;
    //color.y = 1.0 - color.y;
    //color.z = 1.0 - color.z;
    if (v_numAccumulatedFrames != 0)
    {
        FragColor = vec4(color / v_numAccumulatedFrames, 1.0f);
        
    }
    else
    {
        FragColor = vec4(color, 1.0f);
    }

    
    
    //FragColor = texture(u_RenderedTexture, vec2(TexCoords.x, TexCoords.y));
} 


