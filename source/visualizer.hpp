#pragma once

#include <glm/glm.hpp>
#include <SDL.h>
#include <ios>
#include "window.hpp"


using color_t = glm::vec< 4, uint8_t, glm::defaultp >;

class Visualizer {
    struct ColorRestorer {
        ColorRestorer(SDL_Renderer* renderer);
        ~ColorRestorer();

        SDL_Renderer* m_renderer;
        uint8_t r, g, b, a;
    };

public:
    

    Visualizer(Window& window);

    void set_simulation_rectangle(glm::vec2 simulation_rectangle);

    void draw_rectangle(glm::vec2 position, glm::vec2 dimentions, color_t color);

    void draw_line(glm::vec2 from, glm::vec2 to, color_t color);

    void draw_circle(glm::vec2 position, float radius, color_t color);

    void finish_frame();
    static color_t some_color(uint64_t idx);

private:
    SDL_Texture* m_texture;
    SDL_Renderer* m_renderer;
    Window& m_window;
    glm::vec2 m_simulation_rectangle;

    float to_screen_k();

    glm::vec2 to_screen_coords(glm::vec2 coords);
};

