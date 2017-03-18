#version 330 core

in vec3 Normal;
in vec3 Position;

out vec4 color;

uniform vec3 cameraPos;
uniform samplerCube skybox;

void main()
{             

    vec3 i = normalize(Position - cameraPos);
    vec3 reflect = reflect(i, normalize(Normal));
    
    
    color = texture(skybox, reflect);

}  