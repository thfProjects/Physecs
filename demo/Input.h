#pragma once

#include <SDL.h>
#include <functional>
#include <vector>
#include <glm/glm.hpp>
#include <string>

namespace Input {
	void init();
	void poll();
	bool getQuit();
	bool getMouseDown(Uint8 button);
	bool getMouseUp(Uint8 button);
	glm::vec2 getMouseDeltaPos();
	bool getKeyDown(SDL_Scancode scancode);
	bool getKey(SDL_Scancode scancode);
};

