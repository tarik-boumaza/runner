#version 330


#ifdef VERTEX_SHADER
layout(location= 0) in vec3 position;
layout(location= 1) in vec2 texcoord;
layout(location= 2) in vec3 normal;

uniform mat4 mvpMatrix;
uniform mat4 modelMatrix;

out vec3 p;
out vec3 n;
out vec2 vertex_texcoord;

void main( )
{
    gl_Position= mvpMatrix * vec4(position, 1);

    p= vec3(modelMatrix * vec4(position, 1));
    n= mat3(modelMatrix) * normal;
    vertex_texcoord = texcoord;
}
#endif

#ifdef FRAGMENT_SHADER
in vec2 vertex_texcoord;
in vec3 p;
in vec3 n;

uniform mat4 viewInvMatrix;
uniform sampler2D texture0;

const vec3 emission= vec3(1);
const float k= 0.8;

const float PI= 3.14159265359;

out vec4 fragment_color;
void main( )
{
    vec3 camera= vec3(viewInvMatrix * vec4(0, 0, 0, 1));        // position de la camera dans le repere du monde
    vec3 source = camera;

    // directions
    vec3 o= normalize(camera - p);
    vec3 l= normalize(source - p);
    // cos
    float cos_theta= max(0, dot(normalize(n), l));

    // brdf
    float fr= k / PI;
    //vec3 color= emission * fr * cos_theta;

    //fragment_color= vec4(color, 1);

    //vec4 color= texture(texture0, vertex_texcoord);
    fragment_color= vec4(0, vertex_texcoord.y, 0,1);
}
#endif
