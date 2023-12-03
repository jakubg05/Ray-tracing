#version 460 core

layout(local_size_x = 8, local_size_y = 4, local_size_z = 1) in;
layout(rgba32f, binding = 0) uniform image2D rtxTexture;
layout(rgba32f, binding = 1) uniform image2D postprocTexture;

layout(std140, binding = 2) uniform uniforms {
int u_numAccumulatedFrames;
};

void main()
{
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);

    vec3 color = imageLoad(rtxTexture, pixel_coords).rgb;
    //if (u_numAccumulatedFrames != 0)
    //{
    //    imageStore(postprocTexture, pixel_coords, vec4(color / u_numAccumulatedFrames, 1.0f));
    //}
    //else
    //{
    //    imageStore(postprocTexture, pixel_coords, vec4(color, 1.0f));
    //}
    imageStore(postprocTexture, pixel_coords, vec4(color, 1.0f));
}