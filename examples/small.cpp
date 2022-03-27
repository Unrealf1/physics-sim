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
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"
#include "util.hpp"

using namespace std::chrono_literals;


int main(int, char *[]) {
    WindowParams p {};
    p.w = 820;
    p.h = 380;

    Window w(p);
    
    Physics::Simulation<Physics::ForwardEuler, Physics::BucketCollisionDetector<2>> sim({920, 680});
    sim.add_force(Physics::Forces::earth_gravitation());
    Physics::CircleCollider collider = {{{100.0f + 10, 10.0f+10}}, 15.0f};
    Physics::PhysicsItem item = {1.0f, collider.m_position, {80.0f, 0.0f}};
    auto second_ref = sim.add_circle({collider, item});

    collider = {{{100.0f + 300.0f, 12.0f+10}}, 15.0f};
    item = {1.0f, collider.m_position, {-80.0f, 0.0f}};
    auto first_ref = sim.add_circle({collider, item});

    collider = {{{100.0f + 300.0f, 12.0f+50}}, 7.5f};
    item = {0.5f, collider.m_position, {50.0f, -10.0f}};
    sim.add_circle({collider, item});
    
    collider = {{{100.0f + 700.0f, 20.0f+10}}, 30.0f};
    item = {5.0f, collider.m_position, {-200.0f, 0.0f}};
    sim.add_circle({collider, item});

    sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{500.0f, 500.0f}, glm::vec2{700.0f, 700.0f}));

    auto ref1 = sim.get_object_ref(0);
    auto ref2 = sim.get_object_ref(1);

    auto& first_point = sim.get_point_ref(ref1);
    auto& second_point = sim.get_point_ref(ref2);
    auto spring = Physics::Forces::spring(3.5, 500.0, 
            ref1.get().m_phys_item.id, ref2.get().m_phys_item.id,
            first_point, second_point        
    );
    sim.add_force(spring);
    
    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .fps_limit = 100.0f } ));

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
        const auto& statics = sim.get_static_colliders();
        for (const auto& st_p : statics) {
            auto& st = *st_p;
            st.visualize(v); 
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});

        v.draw_line(first_point.get(), second_point.get(), {255, 0, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
    }
    phys_thread.request_stop();
    return 0;
}
