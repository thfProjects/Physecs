#version 330 core
out float FragColor;
  
in vec2 uv;

uniform sampler2D gDepth;
uniform sampler2D gNormal;
uniform sampler2D noise;

uniform mat4 projection;

uniform vec2 screenSize;

vec3 reconstructViewPos(vec2 screenPos) {
    float viewZ = -projection[3][2] / (2 * texture(gDepth, screenPos).r + projection[2][2] - 1);
    float viewX = -((screenPos.x * 2.0 - 1.0) / projection[0][0]) * viewZ;
    float viewY = -((screenPos.y * 2.0 - 1.0) / projection[1][1]) * viewZ;
    return vec3(viewX, viewY, viewZ);
}

void main()
{
    vec2 noiseScale = screenSize / textureSize(noise, 0);

    vec3 fragPos = reconstructViewPos(uv);
    vec3 normal = texture(gNormal, uv).xyz;
    vec3 randomVec = vec3(texture(noise, uv * noiseScale).xy, 0.0);

    vec3 tangent   = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);

    mat3 TBN = mat3(tangent, bitangent, normal);

    vec3 halfSphereSamps[] = vec3[](
        vec3(-0.001911766579313939, -0.0002525762674416723, 0.0018314090748514683),
        vec3(-0.03779268231488415, 0.28384226301535215, 0.2009696850561953),
        vec3(-0.17901027149480006, -0.005063922643533762, 0.10413077684599852),
        vec3(-2.01611717808468e-05, -1.7686531795719263e-05, 2.0144520239452702e-05),
        vec3(-0.2822497285300654, 0.06847467570314258, 0.09171291052133033),
        vec3(0.03949606609588392, 0.022669744950509736, 0.042522411261985076),
        vec3(1.1187278857487193e-06, -0.08237904822753926, 0.0035151091989160895),
        vec3(0.0014160060517956065, 1.0722930815118928e-06, 0.0010256668665174812),
        vec3(0.005494212569980125, 0.028569831695486596, 0.004844402367968765),
        vec3(7.388107526128888e-10, 2.1341026796102302e-05, 3.135450282409507e-06),
        vec3(-1.678179625710595e-08, 6.737812294230184e-07, 1.9571208260288367e-07),
        vec3(0.00019433369571921052, 0.00032024999432561706, 0.0004021525527862663),
        vec3(0.16668010929566507, 4.955548553245325e-06, 0.07486954329122605),
        vec3(0.32249872728370715, -0.1635919873567441, 0.1024319341939423),
        vec3(7.026166139949474e-05, -0.04808151277831542, 0.06435644760277064),
        vec3(8.990528484158456e-05, 0.00012699723528791722, 7.499983095842244e-05),
        vec3(0.00028372598061221813, 0.4117992660868147, 0.027590916958187387),
        vec3(-0.00013555799052732703, -6.0622303830051135e-06, 7.504259098772241e-05),
        vec3(0.37316394019098664, -0.0028155777668865366, 0.15930717258480942),
        vec3(0.12125447055925988, -0.015829298058125787, 0.043435460106775785),
        vec3(0.0007873005718911795, -0.023413575514458, 0.40607845145733584),
        vec3(0.13254208246817337, 0.20638110814963254, 0.05735094387731922),
        vec3(-0.008935704699416024, -0.0007651911121378962, 0.009340006345524529),
        vec3(0.2960645007424801, 0.021322443516878137, 0.14347558683123446),
        vec3(0.00012639205129548367, 5.647515253549893e-05, 6.834955486374388e-05),
        vec3(-0.29371931790948735, 0.049023478164795756, 0.008052146566365912),
        vec3(-0.00047028846472196487, -2.161941475921405e-07, 0.00043611229634861725),
        vec3(0.022996836028838615, -0.08652792689430519, 0.08232538199106706),
        vec3(-0.00035869099655152495, 0.06557273111610885, 0.1064632039965965),
        vec3(8.691998393390051e-06, 1.3779652616572802e-05, 1.220241037053872e-05),
        vec3(0.08427301149840834, 0.17673513939244356, 0.1788245647568495),
        vec3(-0.00212704765352897, -0.002171114431044537, 0.000685826560736034)
    );

    float occlusion = 0.0;
    float radius = 0.1;

    for (int i = 0; i < 32; i++) {
        vec3 samplePos = fragPos + (TBN * halfSphereSamps[i]) * radius;

        vec4 offset = projection * vec4(samplePos, 1.0);    // from view to clip-space
        offset.xyz /= offset.w;               // perspective divide
        offset.xyz = offset.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0  

        vec3 sampleGPos = reconstructViewPos(offset.xy); 

        float rangeCheck = smoothstep(0.0, 1.0, radius / length(fragPos - sampleGPos));
        occlusion += float(sampleGPos.z >= samplePos.z + 0.025) * rangeCheck; 
    }

    occlusion = 1.0 - occlusion / 32.0;

    FragColor = occlusion;
}