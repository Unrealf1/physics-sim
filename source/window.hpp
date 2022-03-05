#pragma once

#include <glm/glm.hpp>

#include <memory>
#include <cstdint>


struct SDL_Window;
struct SDL_Renderer;
namespace engine {
    struct WindowParams {
        WindowParams();

        const char* title;
        int x;
        int y;
        int w;
        int h;
        uint32_t flags;
    };

    struct Window {
        using sdl_window_t = std::unique_ptr<SDL_Window, void(*)(SDL_Window*)>;
        using sdl_renderer_t = std::unique_ptr<SDL_Renderer, void(*)(SDL_Renderer*)>;
        
        Window(const WindowParams&);
        
        void SetClearColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha = 255);
        glm::ivec2 getDimentions() const;
        
        sdl_window_t m_window;
        sdl_renderer_t m_renderer;
    };
}

