#include "Input.h"
#include <algorithm>

namespace {
    bool quit = false;
    std::vector<Uint8> mouseButtonsDown = {};
    std::vector<Uint8> mouseButtonsUp = {};
    std::vector<SDL_Scancode> keysDown = {};
    std::pair<int, int> mousePos = { 0, 0 };
    std::pair<int, int> mouseDeltaPos = { 0, 0 };
}

void Input::init() {
    SDL_GetMouseState(&mousePos.first, &mousePos.second);
}

void Input::poll()
{
    mouseButtonsDown.clear();
    mouseButtonsUp.clear();
    keysDown.clear();
    mouseDeltaPos.first = 0;
    mouseDeltaPos.second = 0;

	SDL_Event event;

    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                quit = true;
                break;
            case SDL_MOUSEMOTION:
                {
                    std::pair<int, int> mousePrevPos = mousePos;
                    mousePos.first = event.motion.x;
                    mousePos.second = event.motion.y;
                    mouseDeltaPos.first = mousePos.first - mousePrevPos.first;
                    mouseDeltaPos.second = mousePos.second - mousePrevPos.second;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                mouseButtonsDown.push_back(event.button.button);
                break;
            case SDL_MOUSEBUTTONUP:
                mouseButtonsUp.push_back(event.button.button);
                break;
            case SDL_KEYDOWN:
                keysDown.push_back(event.key.keysym.scancode);
                break;
        }
    }
}

bool Input::getQuit()
{
    return quit;
}

bool Input::getMouseDown(Uint8 button)
{
    return std::find(mouseButtonsDown.begin(), mouseButtonsDown.end(), button) != mouseButtonsDown.end();
}

bool Input::getMouseUp(Uint8 button)
{
    return std::find(mouseButtonsUp.begin(), mouseButtonsUp.end(), button) != mouseButtonsUp.end();
}

glm::vec2 Input::getMouseDeltaPos() {
    return glm::vec2(mouseDeltaPos.first, mouseDeltaPos.second);
}

bool Input::getKeyDown(SDL_Scancode scancode)
{
    return std::find(keysDown.begin(), keysDown.end(), scancode) != keysDown.end();
}

bool Input::getKey(SDL_Scancode scancode){
    return SDL_GetKeyboardState(NULL)[scancode];
}
