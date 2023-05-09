#version 330 core
layout (location = 0) in vec3 aPos1;
layout (location = 1) in vec3 aPos2;
uniform mat4 view;
uniform mat4 projection;
out vec3 fragPos;
void main()
{
    gl_Position = projection * view * vec4(aPos1.x, aPos1.y, aPos1.z, 1.0);
    Pos = aPos2;
}
