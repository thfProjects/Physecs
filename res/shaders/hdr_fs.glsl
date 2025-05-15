#version 330 core
out vec4 fragColor;
  
in vec2 uv;

uniform sampler2D colorMap;

void main()
{             
    vec3 hdrColor = texture(colorMap, uv).rgb;

    float exposure = 1.5;
    
    // exposure tone mapping
    vec3 mapped = vec3(1.0) - exp(-hdrColor * exposure);
    // gamma correction 
    mapped = pow(mapped, vec3(1.0 / 2.2));

    fragColor = vec4(mapped, 1.0);
}  