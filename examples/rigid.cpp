#include <thread>
#include <chrono>
#include <memory>
#include <stop_token>
#include <thread>
#include <functional>
#include <ranges>

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
using sim_obj_t = Physics::SimulationObject<Physics::PolygonCollider>;
using std::views::drop;


int main(int, char *[]) {
    WindowParams p {};
    p.w = 820;
    p.h = 380;

    Window w(p);
    
    Physics::Simulation<sim_obj_t, Physics::ForwardEuler, Physics::SimpleCollisionDetector<sim_obj_t>> sim({500, 500});
    sim.add_force(Physics::Forces::earth_gravitation());
    Physics::PolygonCollider collider = {{{100.0f + 10, 10.0f+10}}, {{0.0f, 0.0f}, {30.0f, 10.0f}, {0.0f, 30.0f}}};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {80.0f, 0.0f}};
    sim.add_circle({collider, item});

    collider = {{{100.0f + 10, 10.0f+10}}, {{0.0f, 40.0f}, {10.0f, 40.0f}, {10.0f, 50.0f}}};
    item = {1.0f, collider.m_position, {-80.0f, 0.0f}};
    sim.add_circle({collider, item});

    sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{500.0f, 500.0f}, glm::vec2{700.0f, 700.0f}));
    
    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .fps_limit = 500.0f } ));

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
            auto offset = item.m_collider.m_position;
            auto last_point = *(item.m_collider.m_vertices.begin());
            auto first_point = last_point;
            auto color = v.some_color(item.m_phys_item.id);
            auto points = item.m_collider.get_world_points();
            for (const auto& point : points | drop(1)) {
                v.draw_line(last_point + offset, point + offset, color);
                last_point = point;  
            }
            v.draw_line(last_point + offset, first_point + offset, color);
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
