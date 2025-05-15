#version 330 core

out float FragColor;
  
in vec2 uv;
  
uniform sampler2D ssaoOutput;
uniform sampler2D gDepth;
uniform vec2 projInfo;
uniform uint direction;

float reconstructViewDepth(vec2 screenPos) {
    return -projInfo.x / (2 * texture(gDepth, screenPos).r + projInfo.y - 1);
}

void main() {
    const float depthThreshold = 0.02;
    const int M = 2;
    const float coeffs[M + 1] = float[M + 1](1.0, 0.8661265944287591, 0.44070256305264244);

    float texelSize = 1.0 / vec2(textureSize(ssaoOutput, 0))[direction];
    float fragDepth = reconstructViewDepth(uv);
    vec2 sampleOffset = vec2(0);
    sampleOffset[direction] = texelSize;

    float result = 0.0;
    float totalWeight = 0.0;
    
    for (int i = -M; i <= M; ++i)
    {
        vec2 samplePos = uv + sampleOffset * i;
        float sampleDepth = reconstructViewDepth(samplePos);

        float edgeCheck = float(abs(sampleDepth - fragDepth) < depthThreshold);

        float weight = edgeCheck * coeffs[abs(i)];

        result += texture(ssaoOutput, samplePos).r * weight;
        totalWeight += weight;
    }
    FragColor = result / totalWeight;
}  