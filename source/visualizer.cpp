#include "visualizer.hpp"


Visualizer::ColorRestorer::ColorRestorer(SDL_Renderer* renderer) : m_renderer(renderer) {
    SDL_GetRenderDrawColor(m_renderer, &r, &g, &b, &a);
}

Visualizer::ColorRestorer::~ColorRestorer() {
    SDL_SetRenderDrawColor(m_renderer, r, g, b, a);
}

Visualizer::Visualizer(Window& window) : m_window(window) {
    auto surface = SDL_GetWindowSurface(window.m_window.get());
    m_texture = SDL_CreateTextureFromSurface(window.m_renderer.get(), surface);
    m_renderer = window.m_renderer.get();
    SDL_FreeSurface(surface);
}

void Visualizer::set_simulation_rectangle(glm::vec2 simulation_rectangle) {
    m_simulation_rectangle = simulation_rectangle;
}

void Visualizer::draw_rectangle(glm::vec2 position, glm::vec2 dimentions, color_t color) {
    ColorRestorer cr(m_renderer);
    position *= to_screen_k();
    dimentions *= to_screen_k();
    SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
    SDL_FRect rect { position.x, position.y, dimentions.x, dimentions.y };
    SDL_RenderDrawRectF(m_renderer, &rect);   
}
void Visualizer::draw_line(glm::vec2 from, glm::vec2 to, color_t color) {
    ColorRestorer cr(m_renderer);
    from *= to_screen_k();
    to *= to_screen_k();
    SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLineF(m_renderer, from.x, from.y, to.x, to.y);
}

void Visualizer::draw_circle(glm::vec2 position, float radius, color_t color) {
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
void Visualizer::finish_frame() {
    //SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);
    SDL_RenderPresent(m_renderer);
    SDL_RenderClear(m_renderer);
}
float Visualizer::to_screen_k() {
    auto window_dims = m_window.getDimentions();
    return std::min((float(window_dims.y) / m_simulation_rectangle.y), (float(window_dims.x) / m_simulation_rectangle.x));

}
glm::vec2 Visualizer:: to_screen_coords(glm::vec2 coords) {
    return coords * to_screen_k();
}

color_t Visualizer::some_color(uint64_t idx) {
    auto fidx = float(idx);
    auto s = sinf(fidx);
    auto c = cosf(fidx);
    auto cs = sinf(fidx * fidx) + cosf(fidx * fidx);
    auto r = (s + 1.0f) / 2.0f * 255.0f;
    auto g = (c + 1.0f) / 2.0f * 255.0f;
    auto b = (cs + 1.5f) / 3.0f * 255.0f;
    return {r, g, b, 255};
}
