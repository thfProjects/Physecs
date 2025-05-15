#pragma once

struct TransformComponent {
    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 scale;
};

struct RenderComponent {
    unsigned int VAO;
    unsigned int elements;
    unsigned int texture;
    glm::mat4 model;
};
