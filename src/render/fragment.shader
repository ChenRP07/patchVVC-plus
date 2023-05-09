#version 330 core
in vec3 fragPos;
out vec4 FragColor;
uniform vec3 myColor;
void main()
{
    FragColor = vec4(fragPos.x, fragPos.y, fragPos.z, 1.0f);
}
