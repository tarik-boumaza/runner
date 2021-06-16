#version 330


#ifdef VERTEX_SHADER
layout(location= 0) in vec3 position;
layout(location= 2) in vec3 normal;

uniform mat4 mvpMatrix;
uniform mat4 modelMatrix;

out vec3 p;
out vec3 n;

void main( )
{
    gl_Position= mvpMatrix * vec4(position, 1);

    p= vec3(modelMatrix * vec4(position, 1));
    n= mat3(modelMatrix) * normal;
}
#endif

#ifdef FRAGMENT_SHADER
in vec3 p;
in vec3 n;

uniform mat4 viewInvMatrix;

const vec3 emission= vec3(1);
const float k= 0.8;

const float PI= 3.14159265359;



uniform vec2 u_resolution;
uniform float u_time;

vec2 brickTile(vec2 _st, float _zoom){
    _st *= _zoom;

    // Here is where the offset is happening
    _st.x += step(1., mod(_st.y,2.0)) * 0.5;

    return fract(_st);
}

float box(vec2 _st, vec2 _size){
    _size = vec2(0.5)-_size*0.5;
    vec2 uv = smoothstep(_size,_size+vec2(1e-4),_st);
    uv *= smoothstep(_size,_size+vec2(1e-4),vec2(1.0)-_st);
    return uv.x*uv.y;
}



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
    vec3 color= emission * fr * cos_theta;

    fragment_color= vec4(color, 1);
}
#endif
