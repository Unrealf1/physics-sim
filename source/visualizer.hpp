#pragma once

#include <glm/glm.hpp>
#include <SDL.h>
#include "simulation.hpp"
#include "window.hpp"


class Visualizer {
public:
    Visualizer(engine::Window& window) : m_window(window) {
        auto surface = SDL_GetWindowSurface(window.m_window.get());
        m_texture = SDL_CreateTextureFromSurface(window.m_renderer.get(), surface);
        m_renderer = window.m_renderer.get();
        SDL_FreeSurface(surface);
    }

    void set_simulation_rectangle(glm::vec2 simulation_rectangle) {
        m_simulation_rectangle = simulation_rectangle;
    }

    void draw_rectangle(glm::vec2 position, glm::vec2 dimentions, glm::ivec4 color) {
        position *= to_screen_k();
        dimentions *= to_screen_k();
        uint8_t r, g, b, a;
        SDL_GetRenderDrawColor(m_renderer, &r, &g, &b, &a);
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
        SDL_FRect rect { position.x, position.y, dimentions.x, dimentions.y };
        SDL_RenderDrawRectF(m_renderer, &rect);   
        SDL_SetRenderDrawColor(m_renderer, r, g, b, a);

    }

    void draw_circle(glm::vec2 position, float radius, glm::ivec4 color) {
        uint8_t r, g, b, a;
        SDL_GetRenderDrawColor(m_renderer, &r, &g, &b, &a);
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);

        auto old = position;
        auto old_r = radius;
        position *= to_screen_k();
        radius *= to_screen_k();
        spdlog::warn("circle: {},{} ({}) -> {},{} ({})",
                old.x, old.y, old_r,
                position.x, position.y, radius);
        for (float w = -radius; w < radius; w++) {
            for (float h = -radius; h < radius; h++) {
                if ((w*w + h*h) <= (radius * radius)) {
                    SDL_RenderDrawPointF(m_renderer, position.x + w, position.y + h);
                }
            }
        }


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
    glm::vec2 m_simulation_rectangle;


    float to_screen_k() {
        auto window_dims = m_window.getDimentions();
        float sim_win_height = window_dims.y;
        return (window_dims.y / m_simulation_rectangle.y);

    }

    glm::vec2 to_screen_coords(glm::vec2 coords) {
        // For now assume width > height

        return coords * to_screen_k();

    }

};

