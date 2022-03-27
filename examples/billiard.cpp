#include <thread>
#include <chrono>
#include <memory>
#include <stop_token>
#include <thread>
#include <functional>

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


int main(int, char *[]) {
    WindowParams p {};
    p.w = 450;
    p.h = 900;
    p.title = "billiard";

    Window w(p);
    
    Physics::Simulation<Physics::ForwardEuler, Physics::BucketCollisionDetector<3>> sim({500, 1000});
    sim.add_force(Physics::Forces::damping(0.3f));
    
    uint32_t num_circles_in_side = 5;
    float circle_radius = 15.0f;
    float triangle_side = circle_radius * float(num_circles_in_side) * 2.0f;
    float triangle_top = 80.0f;
    float triangle_bot = triangle_top + triangle_side * sqrtf(3.0f) / 2.0f;
    float triangle_left = sim.get_simulation_rectangle().x / 2.0f - triangle_side / 2.0f;

    for (size_t layer = 0; layer < num_circles_in_side; ++layer) {
        for (size_t circle = 0; circle < num_circles_in_side - layer; ++circle) {
            float x_offset = float(layer) * circle_radius;
            float y_offset = float(layer) * 2.0f * circle_radius;
            Physics::CircleCollider collider = {
                {{
                     x_offset + triangle_left + circle_radius + 2.0f * circle_radius * float(circle), 
                     y_offset + triangle_top
                }}, 
                circle_radius
            };
            Physics::PhysicsItem item = {1.0f, collider.m_position, {0.0f, 0.0f}};
            sim.add_circle({collider, item});
        }
    }

    glm::vec2 last_circle_position = {
        sim.get_simulation_rectangle().x / 2.0f,
        sim.get_simulation_rectangle().y - (sim.get_simulation_rectangle().y - triangle_bot) / 3.0f
    };
    Physics::CircleCollider collider = {{last_circle_position}, circle_radius};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {0.0f, -1000.0f}};
    auto last_circle_id = item.id;
    sim.add_circle({collider, item});

    Visualizer v(w);
    w.SetClearColor(128, 100, 50, 255);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .fps_limit = 1000.0f, .measure_period = 500 } ));    

    bool quit = false;
    SDL_Event event;
    while(!quit){
        while(SDL_PollEvent(&event)){
            switch(event.type){
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto frame_objects = sim.get_objects();
        for (const auto& item : frame_objects) {
            auto color = item.m_phys_item.id != last_circle_id ? color_t(255, 255, 255, 255) : color_t(0, 0, 0, 255);
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius, color);
            v.draw_line(item.m_phys_item.position, item.m_phys_item.position+item.m_phys_item.speed * 0.1f * item.m_phys_item.mass, {255, 0, 100, 255});
        }
        const auto& statics = sim.get_static_colliders();
        for (const auto& st_p : statics) {
            auto& st = *st_p;
            st.visualize(v); 
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
    }
    phys_thread.request_stop();
    return 0;
}
