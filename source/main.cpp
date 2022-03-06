#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <stop_token>
#include <thread>
#include <functional>

#include <spdlog/spdlog.h>
#include <SDL.h>

#include "colliders.hpp"
#include "collision_detectors.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"

using namespace std::chrono_literals;


template<typename Sim>
void physics_f(std::stop_token stoken, Sim* simulation) {
    auto last_frame = std::chrono::steady_clock::now() - 10ms;
    auto frame_duration = 1ms;
    float fps = 0.0f;
    auto& sim = *simulation;
    while (!stoken.stop_requested()) {
        auto now = std::chrono::steady_clock::now();
        auto expected_end_time = now + frame_duration;
        auto since_last_frame = now - last_frame;
        last_frame = now;
        auto dt = float(std::chrono::duration_cast<std::chrono::milliseconds>(since_last_frame).count()) / 1000.0f;
        sim.step(0.005f);
        //sim.step(dt);   
        fps = fps * 0.9f + 1.0f/dt;
        spdlog::info("fps: {}", fps);
        std::this_thread::sleep_until(expected_end_time);
    }
}

int main() {
    engine::WindowParams p {};
    p.w = 1820;
    p.h = 880;

    engine::Window w(p);
    
    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim({1820, 880});
//#define SMALL
#ifdef SMALL
    Physics::CircleCollider collider = {{{100.0f + 10, 10.0f+10}}, 15.0f};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {80.0f, 0.0f}};
    sim.add_circle({collider, item});

    collider = {{{100.0f + 300.0f, 10.0f+10}}, 15.0f};
    item = {1.0f, collider.m_position, {-80.0f, 0.0f}};
    sim.add_circle({collider, item});
#else
    for (int i = 0; i < 18; ++i) {
        for (int j = 0; j < 10; ++j) {
            Physics::CircleCollider collider = {{{10.0f + 38.0f * i, 10.0f+38.0f*j}}, 16.0f};
            Physics::PhysicsItem item = {1.0f, collider.m_position, {80.0f, 0.0f}};
            //Physics::PhysicsItem item = {1.0f, collider.m_position, {cosf(i + j*j) * 100.0f, sinf(i + j*j) * 100.0f}};
            sim.add_circle({ 
                    collider,
                    item
            });
            
        }
    }
#endif
    // 1. друг на друга. 2. близко, произвольные скорости
    sim.add_force(Physics::earth_gravitation());
    //std::this_thread::sleep_for(10s);
    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(physics_f<decltype(sim)>, &sim);

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
            //v.draw_line(item.m_phys_item.position, item.m_phys_item.position+item.m_phys_item.speed, {200, 200, 0, 255});
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.finish_frame();
    }
    phys_thread.request_stop();

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
