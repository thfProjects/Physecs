#pragma once

#include <entt.hpp>
#include "Camera.h"
#include "Light.h"
#include "Physecs.h"

namespace Renderer
{
	void init(int width, int height);
	void draw(const Camera& camera, const Light& light, entt::registry& registry);
	void cleanupRenderer();
};
