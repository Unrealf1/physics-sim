#include "window.hpp"

#include <SDL.h>
#include <spdlog/spdlog.h>

#include <cstdlib>


WindowParams::WindowParams()
: title("new window")
, x(SDL_WINDOWPOS_UNDEFINED)
, y(SDL_WINDOWPOS_UNDEFINED)
, w(200)
, h(200)
, flags(0) 
{}

void exit_cleanup() {
    SDL_Quit();
}

void destroySdlWindow(SDL_Window* window) {
   SDL_DestroyWindow(window);
}

void destroySdlRenderer(SDL_Renderer* renderer) {
   SDL_DestroyRenderer(renderer);
}

SDL_Window* create_raw_window(const WindowParams& params) {
    static bool sdl_ready = false;
    if (!sdl_ready) {
        if (SDL_Init(SDL_INIT_VIDEO)) {
            spdlog::error("could not initialize SDL: {}", SDL_GetError());
        }
        std::atexit(exit_cleanup);
    }

    auto raw_window = SDL_CreateWindow(
        params.title,
        params.x,
        params.y,
        params.w,
        params.h,
        params.flags
    );
    if (raw_window == nullptr) {
        spdlog::error("could not create window: {}", SDL_GetError());
    }
    return raw_window;
}

SDL_Renderer* create_raw_renderer(SDL_Window* raw_window) {
    auto raw_renderer = SDL_CreateRenderer(
        raw_window, 
        -1, 
        SDL_RENDERER_ACCELERATED
    );
    if (raw_renderer == nullptr) {
        spdlog::error("could not create renderer: {}", SDL_GetError());
    }
    return raw_renderer;
}

Window::Window(const WindowParams& params)
: m_window(create_raw_window(params), &destroySdlWindow)
, m_renderer(create_raw_renderer(m_window.get()), &destroySdlRenderer) {
    SDL_RenderClear(m_renderer.get());
    SDL_RenderPresent(m_renderer.get());
}

void Window::SetClearColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {
    SDL_SetRenderDrawColor(m_renderer.get(), red, green, blue, alpha);
}

glm::ivec2 Window::getDimentions() const {
    int width;
    int height;
    
    SDL_GetWindowSize(m_window.get(), &width, &height);

    return { width, height };
}

