#include "Demo.h"
#include <Assets.h>
#include <Components.h>
#include <Renderer.h>

void Demo::onCreate() {
    Assets::loadMesh("box");
    Assets::loadTexture("white.png");

    //create platform
    auto platform = registry.create();
    auto [VAO, elements] = Assets::getMesh("box");
    registry.emplace<TransformComponent>(platform, glm::vec3(0), glm::quat(1, 0, 0, 0), glm::vec3(20, 1, 20));
    registry.emplace<RenderComponent>(platform, VAO, elements, Assets::getTexture("white.png"), glm::mat4(1.0f));
}

void Demo::onUpdate(float deltaTime) {
    cameraMovementSystem.onUpdate(deltaTime);
}

void Demo::draw() {
    for (auto [entity, transform, renderItem] : registry.view<TransformComponent, RenderComponent>().each()) {
        glm::mat4 model(1.0f);
        model = glm::translate(model, transform.position) * glm::toMat4(transform.orientation) * glm::scale(model, transform.scale);
        renderItem.model = model;
    }
    Renderer::draw(camera, light, registry);
}
