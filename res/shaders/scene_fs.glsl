#version 330 core

in vec3 NormalWorldSpace;
in vec3 NormalViewSpace;
in vec3 FragPosWorldSpace;
in vec3 FragPosViewSpace;
in vec2 texCoords;

out vec4 FragColor;

uniform vec3 lightColor;
uniform vec3 lightDir;

const int NB_SHADOW_CASCADES = 3;
uniform mat4 lightProjectionViewMatrices[NB_SHADOW_CASCADES];
uniform float shadowCascadeDepths[NB_SHADOW_CASCADES];
uniform sampler2DArrayShadow shadowMaps;

uniform sampler2D aoMap;

uniform sampler2D texMap;

uniform vec2 screenSize;

float shadowCalculation()
{
	int shadowCascade = NB_SHADOW_CASCADES - 1;
	for (int i = 0; i < NB_SHADOW_CASCADES; i++) {
		if (-FragPosViewSpace.z < shadowCascadeDepths[i]) {
			shadowCascade = i;
			break;
		}
	}

	vec4 fragPosLightSpace = lightProjectionViewMatrices[shadowCascade] * vec4(FragPosWorldSpace + normalize(NormalWorldSpace) * 0.052 * (pow(2.5, shadowCascade)), 1.0);

	//vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w; //perspective division in case we are using a perspective projection
    vec3 projCoords = fragPosLightSpace.xyz * 0.5 + 0.5; //transform coordinates from [-1, 1] to [0, 1] for shadowMap access

	vec2 poissonDisk[16] = vec2[](
		vec2(0.97484398, 0.75648379),
		vec2(0.94558609, -0.76890725),
		vec2(-0.81544232, -0.87912464),
		vec2(-0.81409955, 0.91437590),
		vec2(-0.94201624, -0.39906216),
		vec2(-0.094184101, -0.92938870),
		vec2(0.34495938, 0.29387760),
		vec2(-0.91588581, 0.45771432),
		vec2(-0.38277543, 0.27676845),
		vec2(0.44323325, -0.97511554),
		vec2(0.53742981, -0.47373420),
		vec2(-0.26496911, -0.41893023),
		vec2(0.79197514, 0.19090188),
		vec2(-0.24188840, 0.99706507),
		vec2(0.19984126, 0.78641367),
		vec2(0.14383161, -0.14100790)
	);
	
	float shadow = 0;
	
	for(int i = 0; i < 16; i++) {
		vec4 shadowTexCoords;
		shadowTexCoords.xyw = (projCoords + vec3(poissonDisk[i]/1000, 0)).xyz;
		shadowTexCoords.z = shadowCascade;
		shadow += texture(shadowMaps, shadowTexCoords);
	}
	
	shadow /= 16;

	return shadow;
}

void main()
{
	vec3 nNormal = normalize(NormalViewSpace);
	vec3 nLightDir = normalize(lightDir);

	vec3 reflectDir = reflect(nLightDir, nNormal);
	vec3 viewDir = normalize(-FragPosViewSpace);

	float specular = 0.5 * pow(max(dot(reflectDir, viewDir), 0.0), 32);
	float diffuse = max(dot(nNormal,-nLightDir), 0.0);
	float ambient = 0.3;

	float shadow = shadowCalculation();

	float ao = texture(aoMap, gl_FragCoord.xy / screenSize).r;

	FragColor = vec4(ao * (ambient + (1.0 - shadow) * (diffuse + specular)) * lightColor * pow(texture(texMap, texCoords).rgb, vec3(2.2)), 1.0);
}