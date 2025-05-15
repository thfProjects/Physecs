#include <glad/glad.h>
#include <SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Mesh.h"
#include "Renderer.h"
#include <string>
#include "Components.h"
#include "Shader.h"

#define getUniformLocation(shader, uniform) (shader.uniform##Location = glGetUniformLocation(shader.id, #uniform))
#define setUniformMat4f(location, mat) (glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(mat)))
#define setUniformMat3f(location, mat) (glUniformMatrix3fv(location, 1, GL_FALSE, glm::value_ptr(mat)))
#define setUniformVec3f(location, vec) (glUniform3f(location, vec.x, vec.y, vec.z))
#define setUniformVec4f(location, vec) (glUniform4f(location, vec.x, vec.y, vec.z, vec.w))
#define setUniformI(location, i) (glUniform1i(location, i))
#define setUniformUI(location, ui) (glUniform1ui(location, ui))
#define setUniformF(location, f) (glUniform1f(location, f))
#define setUniformF2(location, f0, f1) (glUniform2f(location, f0, f1))
#define setUniformI2(location, i0, i1) (glUniform2i(location, i0, i1))
#define setUniformI4(location, i0, i1, i2, i3) (glUniform4i(location, i0, i1, i2, i3))
#define setUniformMat4fv(location, mat, count) (glUniformMatrix4fv(location, count, GL_FALSE, glm::value_ptr(mat)))
#define setUniformFv(location, f, count) (glUniform1fv(location, count, f))
#define setUniformVec3fv(location, vec, count) (glUniform3fv(location, count, glm::value_ptr(vec)))
#define setUniformIv(location, i, count) (glUniform1iv(location, count, i))

namespace {
    SDL_Window* window;
    SDL_GLContext context;
    int width;
    int height;
    glm::vec3 bgColor = glm::pow(glm::vec3(0.122f, 0.42f, 0.655f), glm::vec3(2.2f));

    struct {
        int id = 0;
        int colorMapLocation = 0;
    } hdrShader;

    struct {
        int id = 0;
        int modelLocation = 0;
        int viewLocation = 0;
        int projectionLocation = 0;
        int normalWorldLocation = 0;
        int normalViewLocation = 0;
        int lightProjectionViewMatricesLocation = 0;
        int shadowCascadeDepthsLocation = 0;
        int lightColorLocation = 0;
        int lightDirLocation = 0;
        int shadowMapLocation = 0;
        int aoMapLocation = 0;
        int texMapLocation = 0;
        int screenSizeLocation = 0;
    } sceneShader;

    struct {
        int id = 0;
        int lightProjectionViewLocation = 0;
        int modelLocation = 0;
    } shadowMapShader;

    struct {
        int id = 0;
        int modelLocation = 0;
        int viewLocation = 0;
        int projectionLocation = 0;
        int normalLocation = 0;
    } gBufferShader;

    struct {
        int id = 0;
        int gDepthLocation = 0;
        int gNormalLocation = 0;
        int noiseLocation = 0;
        int projectionLocation = 0;
        int screenSizeLocation = 0;
    } ssaoShader;

    struct {
        int id = 0;
        int ssaoOutputLocation = 0;
        int gDepthLocation = 0;
        int projInfoLocation = 0;
        int directionLocation = 0;
    } ssaoBlurShader;

    unsigned int colorFBO = 0;
    unsigned int colorMultisampleFBO = 0;
    unsigned int colorMap = 0;

    const int NB_SHADOW_CASCADES = 3;
    const int SHADOW_WIDTH = 4096;
    const int SHADOW_HEIGHT = 4096;
    unsigned int shadowMapFBOs[NB_SHADOW_CASCADES];
    unsigned int shadowMaps = 0;

    unsigned int gBufferFBO = 0;
    unsigned int normalMap = 0;
    unsigned int depthMap = 0;

    unsigned int ssaoFBO = 0;
    unsigned int ssaoNoiseMap = 0;
    unsigned int ssaoMap = 0;

    unsigned int ssaoBlurFBO = 0;
    unsigned int ssaoBlurredMap = 0;

    int screenQuadVAO = 0;
}

static void configureHDR() {
    hdrShader.id = createShader("screen_quad_vs", "hdr_fs");
    getUniformLocation(hdrShader, colorMap);
}

static void configureScene() {
    glGenFramebuffers(1, &colorFBO);
    glGenFramebuffers(1, &colorMultisampleFBO);

    unsigned int colorMultiSampleRBO;
    glGenRenderbuffers(1, &colorMultiSampleRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, colorMultiSampleRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_RGBA16F, width, height);

    unsigned int depthRBO;
    glGenRenderbuffers(1, &depthRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRBO);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, width, height);

    glGenTextures(1, &colorMap);
    glBindTexture(GL_TEXTURE_2D, colorMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, colorMultisampleFBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorMultiSampleRBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRBO);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("color multisample framebuffer incomplete\n");

    glBindFramebuffer(GL_FRAMEBUFFER, colorFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, colorMap, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("color framebuffer incomplete\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    sceneShader.id = createShader("scene_vs", "scene_fs");
    getUniformLocation(sceneShader, model);
    getUniformLocation(sceneShader, view);
    getUniformLocation(sceneShader, projection);
    getUniformLocation(sceneShader, normalWorld);
    getUniformLocation(sceneShader, normalView);
    getUniformLocation(sceneShader, lightProjectionViewMatrices);
    getUniformLocation(sceneShader, shadowCascadeDepths);
    getUniformLocation(sceneShader, lightColor);
    getUniformLocation(sceneShader, lightDir);
    getUniformLocation(sceneShader, shadowMap);
    getUniformLocation(sceneShader, aoMap);
    getUniformLocation(sceneShader, texMap);
    getUniformLocation(sceneShader, screenSize);
}

static void configureShadowMap() {
    glGenFramebuffers(NB_SHADOW_CASCADES, shadowMapFBOs);

    glGenTextures(1, &shadowMaps);
    glBindTexture(GL_TEXTURE_2D_ARRAY, shadowMaps);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, NB_SHADOW_CASCADES, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_FUNC, GL_GREATER);
    float borderColor[] = {1.f};
    glTexParameterfv(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BORDER_COLOR, borderColor);

    for (int i = 0; i < NB_SHADOW_CASCADES; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBOs[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, shadowMaps, 0, i);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("Shadow map framebuffer incomplete\n");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    shadowMapShader.id = createShader("shadow_map_vs", "shadow_map_fs");
    getUniformLocation(shadowMapShader, lightProjectionView);
    getUniformLocation(shadowMapShader, model);
}

static void configureGBuffer() {
    glGenFramebuffers(1, &gBufferFBO);

    glGenTextures(1, &normalMap);
    glBindTexture(GL_TEXTURE_2D, normalMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, gBufferFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, normalMap, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthMap, 0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("G buffer framebuffer incomplete\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    gBufferShader.id = createShader("g_buffer_vs", "g_buffer_fs");
    getUniformLocation(gBufferShader, model);
    getUniformLocation(gBufferShader, view);
    getUniformLocation(gBufferShader, projection);
    getUniformLocation(gBufferShader, normal);
}

static void configureSSAO() {
    glGenFramebuffers(1, &ssaoFBO);

    glGenTextures(1, &ssaoMap);
    glBindTexture(GL_TEXTURE_2D, ssaoMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ssaoMap, 0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("SSAO framebuffer incomplete\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenTextures(1, &ssaoNoiseMap);
    glBindTexture(GL_TEXTURE_2D, ssaoNoiseMap);

    std::vector<glm::vec2> ssaoNoise = {
        glm::vec2(-0.2473617842849769, 0.8700776009512741),
        glm::vec2(-0.9657687452696304, 0.9116913600525844),
        glm::vec2(-0.07262069701107454, -0.21612773700485666),
        glm::vec2(0.2565408654379977, -0.6747910240063866),
        glm::vec2(-0.8420798351184655, 0.11726231190273295),
        glm::vec2(0.34772003429726484, -0.5044640404084837),
        glm::vec2(0.12280321764273983, 0.5686962723426037),
        glm::vec2(-0.042618392564717666, 0.28167797463362576),
        glm::vec2(0.075907568132767, 0.9181713373225369),
        glm::vec2(-0.25276172067457603, -0.03803966013110793),
        glm::vec2(-0.058709036354897215, 0.880339415745008),
        glm::vec2(0.8394033931214164, -0.06234665700630315),
        glm::vec2(0.9141693948341552, 0.7133988687129862),
        glm::vec2(-0.33392091992013295, 0.270287523533405),
        glm::vec2(0.19048981567866474, -0.6965539173400959),
        glm::vec2(-0.012337370348203036, -0.9557649100793781)
    };

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, 4, 4, 0, GL_RG, GL_FLOAT, &ssaoNoise[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    ssaoShader.id = createShader("screen_quad_vs", "ssao_fs");
    getUniformLocation(ssaoShader, gDepth);
    getUniformLocation(ssaoShader, gNormal);
    getUniformLocation(ssaoShader, noise);
    getUniformLocation(ssaoShader, projection);
    getUniformLocation(ssaoShader, screenSize);
}

static void configureSSAOBlur() {
    glGenFramebuffers(1, &ssaoBlurFBO);

    glGenTextures(1, &ssaoBlurredMap);
    glBindTexture(GL_TEXTURE_2D, ssaoBlurredMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, ssaoBlurFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ssaoBlurredMap, 0);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) printf("SSAO blur framebuffer incomplete\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ssaoBlurShader.id = createShader("screen_quad_vs", "ssao_blur_fs");
    getUniformLocation(ssaoBlurShader, ssaoOutput);
    getUniformLocation(ssaoBlurShader, gDepth);
    getUniformLocation(ssaoBlurShader, projInfo);
    getUniformLocation(ssaoBlurShader, direction);
}

static void createScreenQuad() {

    std::vector<ScreenVertex> vertices = {
        { glm::vec3( 1,  1, 0), glm::vec2(1, 1) }, // top right
        { glm::vec3( 1, -1, 0), glm::vec2(1, 0) }, // bottom right
        { glm::vec3(-1, -1, 0), glm::vec2(0, 0) }, // bottom left
        { glm::vec3(-1,  1, 0), glm::vec2(0, 1) } // top left 
    };

    std::vector<unsigned int> indices = {
        0, 3, 1,   // first triangle
        1, 3, 2    // second triangle
    };

    screenQuadVAO = createMesh(vertices, indices);
}

void Renderer::init(int width, int height) {

    ::width = width;
    ::height = height;

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    window = SDL_CreateWindow("PhysecsDemo", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL);

    context = SDL_GL_CreateContext(window);

    SDL_GL_SetSwapInterval(1);

    gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress);

    glViewport(0, 0, width, height);

    glClearColor(bgColor.x, bgColor.y, bgColor.z, 1.0f);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    configureHDR();
    configureScene();
    configureShadowMap();
    configureGBuffer();
    configureSSAO();
    configureSSAOBlur();
    createScreenQuad();
}

static void renderShadowMaps(entt::registry& registry, glm::mat4* lightProjectionViews) {
    for (int i = 0; i < NB_SHADOW_CASCADES; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBOs[i]);

        glClear(GL_DEPTH_BUFFER_BIT);
        
        glUseProgram(shadowMapShader.id);

        setUniformMat4f(shadowMapShader.lightProjectionViewLocation, lightProjectionViews[i]);

        for (auto [entity, transform, mesh] : registry.view<TransformComponent, RenderComponent>().each()) {
            setUniformMat4f(shadowMapShader.modelLocation, mesh.model);

            glBindVertexArray(mesh.VAO);

            glDrawElements(GL_TRIANGLES, mesh.elements, GL_UNSIGNED_INT, 0);
        }
    }
}

static void createGBuffer(entt::registry& registry, glm::mat4 projection, glm::mat4 view) {
    glBindFramebuffer(GL_FRAMEBUFFER, gBufferFBO);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(gBufferShader.id);

    setUniformMat4f(gBufferShader.projectionLocation, projection);
    setUniformMat4f(gBufferShader.viewLocation, view);

    for (auto [entity, transform, mesh] : registry.view<TransformComponent, RenderComponent>().each()) {
        setUniformMat4f(gBufferShader.modelLocation, mesh.model);

        glm::mat3 normal(glm::transpose(glm::inverse(view * mesh.model)));
        setUniformMat3f(gBufferShader.normalLocation, normal);

        glBindVertexArray(mesh.VAO);

        glDrawElements(GL_TRIANGLES, mesh.elements, GL_UNSIGNED_INT, 0);
    }
}

static void renderSSAOTexture(glm::mat4 projection) {
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFBO);

    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(ssaoShader.id);

    glBindTexture(GL_TEXTURE_2D, depthMap);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalMap);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, ssaoNoiseMap);
    glActiveTexture(GL_TEXTURE0);

    setUniformMat4f(ssaoShader.projectionLocation, projection);

    setUniformI(ssaoShader.gDepthLocation, 0);
    setUniformI(ssaoShader.gNormalLocation, 1);
    setUniformI(ssaoShader.noiseLocation, 2);

    setUniformF2(ssaoShader.screenSizeLocation, width, height);

    glBindVertexArray(screenQuadVAO);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

static void blurSSAOTexture(glm::mat4 projection) {
    glUseProgram(ssaoBlurShader.id);

    setUniformI(ssaoBlurShader.ssaoOutputLocation, 0);
    setUniformI(ssaoBlurShader.gDepthLocation, 1);
    setUniformF2(ssaoBlurShader.projInfoLocation, projection[3][2], projection[2][2]);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glActiveTexture(GL_TEXTURE0);

    glBindVertexArray(screenQuadVAO);

    //Horizontal
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoBlurFBO);

    glClear(GL_COLOR_BUFFER_BIT);

    glBindTexture(GL_TEXTURE_2D, ssaoMap);

    setUniformUI(ssaoBlurShader.directionLocation, 0);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    //Vertical
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFBO);

    glClear(GL_COLOR_BUFFER_BIT);

    glBindTexture(GL_TEXTURE_2D, ssaoBlurredMap);

    setUniformUI(ssaoBlurShader.directionLocation, 1);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

static void copyPrecomputedDepthBuffer() {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gBufferFBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, colorMultisampleFBO);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

static void renderScene(entt::registry& registry, Light light, glm::mat4 projection, glm::mat4 view, glm::mat4* lightProjectionViews, float* shadowCascadeDepths) {
    glBindFramebuffer(GL_FRAMEBUFFER, colorMultisampleFBO);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBindTexture(GL_TEXTURE_2D_ARRAY, shadowMaps);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, ssaoMap);
    glActiveTexture(GL_TEXTURE2);

    glUseProgram(sceneShader.id);

    setUniformMat4f(sceneShader.projectionLocation, projection);
    setUniformMat4f(sceneShader.viewLocation, view);

    setUniformVec3f(sceneShader.lightColorLocation, light.color);
    setUniformVec3f(sceneShader.lightDirLocation, glm::vec3(view * glm::vec4(light.direction, 0.0f)));
    setUniformMat4fv(sceneShader.lightProjectionViewMatricesLocation, lightProjectionViews[0], NB_SHADOW_CASCADES);
    setUniformFv(sceneShader.shadowCascadeDepthsLocation, shadowCascadeDepths, NB_SHADOW_CASCADES);

    setUniformI(sceneShader.shadowMapLocation, 0);
    setUniformI(sceneShader.aoMapLocation, 1);
    setUniformI(sceneShader.texMapLocation, 2);

    setUniformF2(sceneShader.screenSizeLocation, width, height);

    for (auto [entity, transform, mesh] : registry.view<TransformComponent, RenderComponent>().each()) {
        setUniformMat4f(sceneShader.modelLocation, mesh.model);

        glm::mat3 normalWorld(glm::transpose(glm::inverse(mesh.model)));
        setUniformMat3f(sceneShader.normalWorldLocation, normalWorld);

        glm::mat3 normalView(glm::transpose(glm::inverse(view * mesh.model)));
        setUniformMat3f(sceneShader.normalViewLocation, normalView);

        glBindTexture(GL_TEXTURE_2D, mesh.texture);

        glBindVertexArray(mesh.VAO);

        glDrawElements(GL_TRIANGLES, mesh.elements, GL_UNSIGNED_INT, 0);
    }

    glActiveTexture(GL_TEXTURE0);
}

static void copyMultisampleColorBuffer() {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, colorMultisampleFBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, colorFBO);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
}

static void renderHDR() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(hdrShader.id);

    glBindTexture(GL_TEXTURE_2D, colorMap);

    glBindVertexArray(screenQuadVAO);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

static void initCSM(glm::mat4 view, float fov, glm::vec3 cameraRight, float aspect, float zNear, float zFar, glm::vec3 lightDir, glm::mat4* lightProjectionViewMatrices, float* shadowCascadeDepths) {

    for (int i = 0; i < NB_SHADOW_CASCADES; ++i) {
        float ratio = i / static_cast<float>(NB_SHADOW_CASCADES);
        float log = zNear * glm::pow(zFar / zNear, ratio);
        float linear = zNear + (zFar - zNear) * ratio;
        shadowCascadeDepths[i] = glm::mix(log, linear, 0.5f);
    }

    shadowCascadeDepths[NB_SHADOW_CASCADES] = zFar;

    for (int i = 0; i < NB_SHADOW_CASCADES; ++i) {
        glm::mat4 subFrustumProjection = glm::perspective(
            fov,
            aspect,
            shadowCascadeDepths[i],
            shadowCascadeDepths[i + 1]
            );

        glm::mat4 invProjectionView = glm::inverse(subFrustumProjection * view);

        glm::vec4 frustumCorners[8];
        glm::vec3 frustumCenter = glm::vec3(0);

        for (unsigned int x = 0; x < 2; ++x)
        {
            for (unsigned int y = 0; y < 2; ++y)
            {
                for (unsigned int z = 0; z < 2; ++z)
                {
                    const glm::vec4 pt = invProjectionView * glm::vec4(
                            2.0f * x - 1.0f,
                            2.0f * y - 1.0f,
                            2.0f * z - 1.0f,
                            1.0f);
                    frustumCorners[x * 4 + y * 2 + z] = pt / pt.w;
                    frustumCenter += glm::vec3(pt / pt.w);
                }
            }
        }

        frustumCenter /= 8;

        glm::mat4 lightView = glm::lookAt(
            frustumCenter - lightDir * 5.0f,
            frustumCenter,
            glm::cross(-lightDir, cameraRight)
        );

        float minX = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::lowest();
        float minZ = std::numeric_limits<float>::max();
        float maxZ = std::numeric_limits<float>::lowest();
        for (const auto& v : frustumCorners)
        {
            const auto trf = lightView * v;
            minX = std::min(minX, trf.x);
            maxX = std::max(maxX, trf.x);
            minY = std::min(minY, trf.y);
            maxY = std::max(maxY, trf.y);
            minZ = std::min(minZ, trf.z);
            maxZ = std::max(maxZ, trf.z);
        }

        glm::mat4 lightProjection = glm::ortho(minX, maxX, minY, maxY, -maxZ - 10, -minZ);

        lightProjectionViewMatrices[i] = lightProjection * lightView;
    }
}

void Renderer::draw(const Camera& camera, const Light& light, entt::registry& registry)
{
    float zNear = 0.1f;
    float zFar = 200.0f;
    float fov = glm::radians(camera.fov);
    float aspect = static_cast<float>(width) / static_cast<float>(height);

    glm::mat4 projection = glm::perspective(fov, aspect, zNear, zFar);

    glm::mat4 view = glm::toMat4(glm::conjugate(camera.orientation)) * glm::translate(glm::mat4(1.0f), -camera.position);

    glm::mat4 lightProjectionViews[NB_SHADOW_CASCADES];
    float shadowCascadeDepths[NB_SHADOW_CASCADES + 1];

    initCSM(view, fov, glm::angleAxis(glm::yaw(camera.orientation), glm::vec3(0, 1, 0)) * glm::vec3(1, 0, 0), aspect, zNear, zFar, light.direction, lightProjectionViews, shadowCascadeDepths);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);

    renderShadowMaps(registry, lightProjectionViews);

    glViewport(0, 0, width, height);

    createGBuffer(registry, projection, view);
    renderSSAOTexture(projection);
    blurSSAOTexture(projection);

    copyPrecomputedDepthBuffer();

    glClearColor(bgColor.x, bgColor.y, bgColor.z, 1.0f);

    renderScene(registry, light, projection, view, lightProjectionViews, &shadowCascadeDepths[1]);
    copyMultisampleColorBuffer();
    renderHDR();

    SDL_GL_SwapWindow(window);
}

void Renderer::cleanupRenderer() {
    SDL_DestroyWindow(window);
}
