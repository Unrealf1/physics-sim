#pragma once

#include <glm/glm.hpp>
#include <SDL.h>
#include "window.hpp"


class Visualizer {
public:
    Visualizer(engine::Window& window) : m_window(window) {
        auto surface = SDL_GetWindowSurface(window.m_window.get());
        m_texture = SDL_CreateTextureFromSurface(window.m_renderer.get(), surface);
        m_renderer = window.m_renderer.get();
        SDL_FreeSurface(surface);
    }

    void draw_circle(glm::vec2 position, float radius, glm::ivec4 color) {
        uint8_t r, g, b, a;
        SDL_GetRenderDrawColor(m_renderer, &r, &g, &b, &a);
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
        SDL_FRect rect { position.x, position.y, radius, radius };
        SDL_RenderDrawRectF(m_renderer, &rect);   
        SDL_SetRenderDrawColor(m_renderer, r, g, b, a);
    }

    void finish_frame() {
        SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);
        SDL_RenderPresent(m_renderer);
        SDL_RenderClear(m_renderer);
    }

private:
    SDL_Texture* m_texture;
    SDL_Renderer* m_renderer;
    engine::Window& m_window;
};

