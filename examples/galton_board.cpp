#include <thread>
#include <chrono>
#include <memory>
#include <stop_token>
#include <thread>
#include <functional>
#include <limits>

#include <spdlog/spdlog.h>
#include <SDL.h>

#include "colliders.hpp"
#include "collision_detectors.hpp"
#include "forces.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"
#include "util.hpp"

using namespace std::chrono_literals;


int main() {
    engine::WindowParams p {};
    p.w = 1000;
    p.h = 800;
    p.title = "Galton board";
    p.flags = SDL_WINDOW_RESIZABLE;

    engine::Window w(p);
    
    uint32_t sim_width = 1000;
    uint32_t sim_height = 900;
    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim({sim_width, sim_height});
    sim.add_force(Physics::earth_gravitation());
    sim.add_force(Physics::damping(0.17f));

    float top_offset = 300.0f;
    float bottom_offset = 200.0f;
    float sides_offset = 50.0f;

    float field_width = float(sim_width) - 2.0f * sides_offset;
    float field_height = float(sim_height) - top_offset - bottom_offset;

    uint32_t num_sections = 10;
    float section_len = field_width / float(num_sections);
    uint32_t num_layers = uint32_t(std::ceil(field_height / section_len));
    uint32_t num_knobs = num_sections;
    float knob_radius = 10.0f; // section_len * 2.0f / 3.0f;

    // Add separators
    for (uint32_t section = 0; section < num_sections + 1; ++section) {
        float x = sides_offset + float(section) * section_len;
        float y_top = float(sim_height) - bottom_offset;
        float y_bot = float(sim_height);
        float dx = section_len / 20.0f;
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x - dx, y_bot}, glm::vec2{x - dx, y_top}));
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x + dx, y_bot}, glm::vec2{x + dx, y_top}));
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x - dx, y_top}, glm::vec2{x + dx, y_top}));
    }

    // Add immovable circles
    for (uint32_t layer = 0; layer < num_layers; ++layer) {
        bool even_layer = layer % 2 == 0;
        uint32_t current_nobs = even_layer ? num_knobs - 1 : num_knobs;
        float top_start = top_offset + float(layer) * section_len;
        float left_start = sides_offset + section_len / 2.0f + even_layer * section_len / 2.0f;

        for (float knob = 0; knob < float(current_nobs); knob += 1.0f) {
            sim.add_static_collider(std::make_unique<Physics::StaticCircleCollider>(
                glm::vec2{left_start + knob * section_len, top_start}, knob_radius)
            );
        }
    }

    // Add falling items
    float circle_radius = knob_radius / 3.0f;
    glm::vec2 box_start = { float(sim_width) / 2.0f - section_len / 2.0f, circle_radius }; // left top
    glm::vec2 box_end = { box_start.x + section_len, top_offset - 2 * knob_radius - circle_radius }; // right bot
    float circle_area = circle_radius * 2.0f * 1.1f;
    float circles_w = std::floor((box_end.x - box_start.x) / (circle_area));
    float circles_h = std::floor((box_end.y - box_start.y) / (circle_area));
    spdlog::info("w = {}; h = {}", circles_w, circles_h);
    spdlog::info("start: {},{}; end: {},{}", box_start.x, box_start.y, box_end.x, box_end.y);
    uint64_t num_circles = 0;
    for (float j = 0.0f; j < circles_h; j += 1.0f) {
        for (float i = - circles_h + j; i < circles_w + circles_h - j; i += 1.0f) {
            ++num_circles;
            auto item = create_sim_object(
                {box_start + glm::vec2{ i * circle_area, j * circle_area }},
                {},
                circle_radius,
                1.0f
            );
            sim.add_circle(item);
        }
    }
    spdlog::info("launching simulaiton with {} circles", num_circles);
    // Add slides
    sim.add_static_collider(
        std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{0.0f, 0.0f}, glm::vec2{box_start.x, box_end.y})
    );
    sim.add_static_collider(
        std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{sim.get_simulation_rectangle().x, 0.0f}, box_end)
    );

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 0.01f, .fps_limit = 0.0f } ));

    bool quit = false;
    SDL_Event event;
    while(!quit) {
        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto frame_objects = sim.get_objects();
        for (const auto& item : frame_objects) {
            auto color = item.m_phys_item.is_static ? color_t{255, 255, 255, 255} : v.some_color(item.m_phys_item.id);
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius, color);
            //v.draw_line(item.m_phys_item.position, item.m_phys_item.position+item.m_phys_item.speed * 0.1f * item.m_phys_item.mass, {200, 200, 0, 255});
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});
        const auto& statics = sim.get_static_colliders();
        for (const auto& st_p : statics) {
            auto& st = *st_p;
            st.visualize(v); 
        }

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
    }
    phys_thread.request_stop();
}
