#pragma once

#include <string>

namespace Assets {
    void loadMesh(const std::string& name);
    void loadTexture(const std::string& name);
    std::pair<unsigned int, unsigned int> getMesh(const std::string& name);
    unsigned int getTexture(const std::string& name);
}
