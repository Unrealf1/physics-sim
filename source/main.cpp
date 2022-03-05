#include <iostream>
#include <thread>
#include <chrono>

#include <spdlog/spdlog.h>
#include <SDL.h>

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
    
    w.SetClearColor(1, 0, 0, 0);

    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim({1000, 1000});
    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
 

    Physics::CircleCollider collider = {{{100.0f, 100.0f}}, 90.0f};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {50.0f, 0.0f}};
    sim.add_circle({ 
            collider,
            item
    });

    sim.add_circle({
        {{{260.0f, 100.0f}}, 30.0f},
        {1.0f, {260.0f, 100.0f}, {-50.0f, 0.0f}}
    });

    sim.add_force(Physics::earth_gravitation());
    bool quit = false;
    SDL_Event event;
    auto last_frame = std::chrono::steady_clock::now() - 10ms;
    auto frame_duration = 1ms;
    while(!quit){
        while(SDL_PollEvent(&event)){
            switch(event.type){
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto now = std::chrono::steady_clock::now();
        auto expected_end_time = now + frame_duration;
        auto since_last_frame = now - last_frame;
        last_frame = now;
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(since_last_frame).count() / 1000.0f;
        sim.step(dt);
        for (const auto& item : sim.get_objects()) {
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius, {255, 0, 0, 255});
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_until(expected_end_time);
        

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
