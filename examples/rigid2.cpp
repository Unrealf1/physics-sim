
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
#include "forces.hpp"
#include "glm/geometric.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "simulation_object.hpp"
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
    p.title = "Rigid bodies";
    p.flags = SDL_WINDOW_RESIZABLE;

    Window w(p);
    
    Physics::Simulation<sim_obj_t, Physics::SemiImplicitEuler, Physics::SimpleCollisionDetector<sim_obj_t>> sim({400, 280});
    sim.add_force(Physics::Forces::earth_gravitation());
    //sim.add_force(Physics::Forces::damping(0.15f));

    float r1 = 10.0f;
    auto obj1 = create_regular_polygon(3, r1, {205, 140}, 2.0f, {0.0f, 0.0f}, 0.0f);
    float r2 = 10.0f;
    auto obj2 = create_regular_polygon(3, r2, {300, 150}, 1.0f, {-100.0f, 0.0f}, 2.0f);
    auto ref2 = sim.add_circle(obj1);
    auto ref1 = sim.add_circle(obj2);

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 1.0f/2000.0f, .fps_limit = 1000.0f, .start_delay=5s }));

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
                auto [collision_point1, collision_point2, collision_normal, is_penetrator] = obj.m_collider.get_collision_points_and_normal(collided.get().m_collider);
                v.draw_circle(collision_point1, 3.0f, {255, 0, 0, 255});
                v.draw_circle(collision_point2, 3.0f, {255, 0, 0, 255});
                v.draw_line(collision_point1, collision_point1 + collision_normal * 20.0f, {0, 0, 255, 255});
                /*spdlog::info("point1: {},{}; point2: {},{}; normal: {},{}",
                        collision_point1.x, 
                        collision_point1.y, 
                        collision_point2.x, 
                        collision_point2.y, 
                        collision_normal.x,
                        collision_normal.y
                );*/
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
