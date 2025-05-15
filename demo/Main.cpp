#include <Demo.h>
#include <SDL.h>
#include <Renderer.h>
#include <Input.h>

int main(int argc, char *args[]) {

    SDL_Init(SDL_INIT_EVERYTHING);

    Renderer::init(1920, 1080);
    Input::init();

    Demo demo;
    demo.onCreate();

    float deltaTime = 0.0f;
    unsigned long long prevTime = 0;

    while (true) {
        Input::poll();

        if (Input::getQuit()) break;

        const unsigned long long time = SDL_GetTicks64();
        deltaTime = (time - prevTime) / 1000.0f;
        prevTime = time;

        demo.onUpdate(deltaTime);
        demo.draw();
    }

    Renderer::cleanupRenderer();
    SDL_Quit();
    return 0;
}
