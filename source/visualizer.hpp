#pragma once

#include <glm/glm.hpp>
#include <SDL.h>
#include <ios>
#include "simulation.hpp"
#include "window.hpp"


using color_t = glm::vec< 4, uint8_t, glm::defaultp >;

class Visualizer {
    struct ColorRestorer {
        ColorRestorer(SDL_Renderer* renderer) : m_renderer(renderer) {
            SDL_GetRenderDrawColor(m_renderer, &r, &g, &b, &a);
        }

        ~ColorRestorer() {
            SDL_SetRenderDrawColor(m_renderer, r, g, b, a);
        }

        SDL_Renderer* m_renderer;
        uint8_t r, g, b, a;
    };

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

    void draw_rectangle(glm::vec2 position, glm::vec2 dimentions, color_t color) {
        ColorRestorer cr(m_renderer);
        position *= to_screen_k();
        dimentions *= to_screen_k();
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
        SDL_FRect rect { position.x, position.y, dimentions.x, dimentions.y };
        SDL_RenderDrawRectF(m_renderer, &rect);   
    }

    void draw_line(glm::vec2 from, glm::vec2 to, color_t color) {
        ColorRestorer cr(m_renderer);
        from *= to_screen_k();
        to *= to_screen_k();
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
        SDL_RenderDrawLineF(m_renderer, from.x, from.y, to.x, to.y);
    }

    void draw_circle(glm::vec2 position, float radius, color_t color) {
        ColorRestorer cr(m_renderer);
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);

        position *= to_screen_k();
        radius *= to_screen_k();
        for (float w = -radius; w < radius; w++) {
            for (float h = -radius; h < radius; h++) {
                if ((w*w + h*h) <= (radius * radius)) {
                    SDL_RenderDrawPointF(m_renderer, position.x + w, position.y + h);
                }
            }
        }
    }

    void finish_frame() {
        SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);
        SDL_RenderPresent(m_renderer);
        SDL_RenderClear(m_renderer);
    }

    static glm::ivec4 some_color(uint64_t idx) {
        auto fidx = float(idx);
        auto s = sinf(fidx);
        auto c = cosf(fidx);
        auto cs = sinf(fidx * fidx) + cosf(fidx * fidx);
        auto r = (s + 1.0f) / 2.0f * 255.0f;
        auto g = (c + 1.0f) / 2.0f * 255.0f;
        auto b = (cs + 1.5f) / 3.0f * 255.0f;
        return glm::vec4(r, g, b, 255);
    }

private:
    SDL_Texture* m_texture;
    SDL_Renderer* m_renderer;
    engine::Window& m_window;
    glm::vec2 m_simulation_rectangle;


    float to_screen_k() {
        auto window_dims = m_window.getDimentions();
        return (float(window_dims.y) / m_simulation_rectangle.y);

    }

    glm::vec2 to_screen_coords(glm::vec2 coords) {
        // For now assume width > height

        return coords * to_screen_k();

    }

};

