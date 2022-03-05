#include <iostream>
#include <thread>
#include <chrono>

#include <spdlog/spdlog.h>

#include "colliders.hpp"
#include "collision_detectors.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"


int main(int argc, char** argv) {
    using namespace std::chrono_literals;

    spdlog::info("Hello, world!");

    engine::WindowParams p {};
    p.w = 1920;
    p.h = 1080;

    engine::Window w(p);
    Visualizer v(w);
    
    w.SetClearColor(1, 0, 0, 0);

    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim;
 

    Physics::CircleCollider collider = {{}, 3.0f};
    Physics::PhysicsItem item = {1.0f, {10.0f, 10.0f}, {}};
    sim.add_circle({ 
            collider,
            item
    });
    sim.add_force(Physics::earth_gravitation());

    for (int i = 0; i < 100; ++i) {
        sim.step(0.1f);
        for (const auto& item : sim.get_objects()) {
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius * 4, {255, 0, 0, 255});
        }

        v.finish_frame();
        std::this_thread::sleep_for(100ms);
    }

    /*SDL_Event event;    // Event variable
    while(!(event.type == SDL_QUIT)){

        // Circle will go down!
        for(int i=0; i<height; i++){
            SDL_Delay(10);  // setting some Delay
            SDL_PollEvent(&event);  // Catching the poll event.
            if(event.type == SDL_QUIT) return;
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
            SDL_RenderClear(renderer);
            draw_circle(width/2, i, 25);
        }

        // Circle will go up!
        for(int i=height; i>0; i--){
            SDL_Delay(10);  // setting some Delay
            SDL_PollEvent(&event);  // Catching the poll event.
            if(event.type == SDL_QUIT) return;
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
            SDL_RenderClear(renderer);
            draw_circle(width/2, i, 25);
        }
    }*/
}
