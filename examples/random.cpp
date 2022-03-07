#include <thread>
#include <chrono>
#include <stop_token>
#include <thread>
#include <functional>
#include <limits>

#include <spdlog/spdlog.h>
#include <SDL.h>

#include "colliders.hpp"
#include "collision_detectors.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"
#include "util.hpp"

using namespace std::chrono_literals;


int main() {
    engine::WindowParams p {};
    p.w = 1820;
    p.h = 880;

    engine::Window w(p);
    
    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim({920, 680});
    sim.add_force(Physics::earth_gravitation());
    
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 30; ++j) {

            Physics::CircleCollider collider = {{{30.0f + 10.0f * i, 30.0f+10.0f*j}}, 3.0f};
            Physics::PhysicsItem item = {1.0f, collider.m_position, {60.0f, 0.0f}};
            //Physics::PhysicsItem item = { 1.0f, collider.m_position, {cosf(i + j*j) * 100.0f, sinf(i + j*j) * 100.0f} };
            sim.add_circle({ 
                    collider,
                    item
            });
            
        }
    }

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 0.001f } ));

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
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius, v.some_color(item.m_phys_item.id));
            v.draw_line(item.m_phys_item.position, item.m_phys_item.position+item.m_phys_item.speed * 0.1f * item.m_phys_item.mass, {200, 200, 0, 255});
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
    }
    phys_thread.request_stop();
}
