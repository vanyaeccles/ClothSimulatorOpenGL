#version 330 core
in vec2 TexCoord;

out vec4 color;

uniform vec3 passedColour;

void main()
{
	
	
	color = vec4(passedColour, 1.0f);

    //color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}