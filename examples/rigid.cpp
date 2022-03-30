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
    Physics::PolygonCollider collider = {{{100.0f + 10, 10.0f + 10}}, {{0.0f, 0.0f}, {60.0f, 10.0f}, {0.0f, 30.0f}}};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {-30.0f, 0.0f}, 1.0f, 0.0f, 0.1f};
    sim.add_circle({collider, item});

    collider = {{{50.0f + 10, 10.0f+10}}, {{0.0f, 40.0f}, {50.0f, 40.0f}, {10.0f, 70.0f}}};
    item = {1.0f, collider.m_position, {80.0f, 0.0f}};
    sim.add_circle({collider, item});

    sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{500.0f, 500.0f}, glm::vec2{700.0f, 700.0f}));
    
    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 1.0f/300.0f, .fps_limit = 500.0f  } ));

    bool quit = false;
    SDL_Event event;
    auto collision_detector = Physics::SimpleCollisionDetector<sim_obj_t>(sim.get_simulation_rectangle());
    while(!quit){
        while(SDL_PollEvent(&event)){
            switch(event.type){
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto frame_objects = sim.get_objects();
        const auto collisions = collision_detector.detect_collisions(frame_objects, {});
        bool sleep = false;
        for (size_t i = 0; i < frame_objects.size(); ++i) {
            const auto& obj = frame_objects[i];
            const auto& col = collisions[i];
            for (const auto& collided : col.objects) {
                //sleep = true;
                auto [collision_point1, collision_point2, collision_normal] = obj.m_collider.get_collision_points_and_normal(collided.get().m_collider);
                v.draw_circle(collision_point1, 7.0f, {255, 0, 0, 255});
                v.draw_circle(collision_point2, 7.0f, {255, 0, 0, 255});
                v.draw_line(collision_point1, collision_point1 + collision_normal, {0, 0, 255, 255});
                spdlog::info("point1: {},{}; point2: {},{}; normal: {},{}",
                        collision_point1.x, 
                        collision_point1.y, 
                        collision_point2.x, 
                        collision_point2.y, 
                        collision_normal.x,
                        collision_normal.y
                );
            }
        }

        auto& o = frame_objects[0];
        for (const auto& item : frame_objects) {
            auto points = item.m_collider.get_world_points();
            auto last_point = *(points.begin());
            auto first_point = last_point;
            auto color = v.some_color(item.m_phys_item.id);
            for (const auto& point : points | drop(1)) {
                v.draw_line(last_point, point, color);
                last_point = point;  
            }
            v.draw_line(last_point, first_point, color);
        }

        const auto& statics = sim.get_static_colliders();
        for (const auto& st_p : statics) {
            auto& st = *st_p;
            st.visualize(v); 
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
        if (sleep) {
            std::this_thread::sleep_for(8s);
        }
    }
    phys_thread.request_stop();
    return 0;
}
