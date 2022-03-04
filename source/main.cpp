#include <iostream>
#include <thread>
#include <chrono>

#include <spdlog/spdlog.h>

#include "collision_detectors.hpp"
#include "integrators.hpp"
#include "window.hpp"
#include "simulation.hpp"


int main(int argc, char** argv) {
    spdlog::info("Hello, world!");

    engine::Window w({});
    w.SetClearColor(1, 0, 0, 0);

    Physics::Simulation<Physics::ForwardEuler, Physics::SimpleCollisionDetector> sim;

    sim.add_circle({});
    sim.add_force(Physics::earth_gravitation());

    for (int i = 0; i < 100; ++i) {
        sim.step(0.1f);
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
