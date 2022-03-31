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
    
    auto regular_polygon = [](size_t num_vertices, float radius, glm::vec2 position, glm::vec2 velocity = {0.0f, 0.0f}, float rot_velocity = 0.0f) -> sim_obj_t {
        float angle_step = std::numbers::pi_v<float> * 2.0f / float(num_vertices);
        std::vector<glm::vec2> vertices;
        vertices.reserve(num_vertices);
        for (size_t i = 0; i < num_vertices; ++i) {
            float angle = angle_step * float(i);
            vertices.emplace_back(
                    std::cos(angle) * radius,
                    std::sin(angle) * radius
            );
        }
        Physics::PolygonCollider collider = {{position}, vertices};
        Physics::PhysicsItem item = {1.0f, collider.m_position, velocity, 1.0f, 0.0f, rot_velocity};
        return {collider, item};
    };

    Physics::Simulation<sim_obj_t, Physics::ForwardEuler, Physics::SimpleCollisionDetector<sim_obj_t>> sim({400, 280});
    sim.add_force(Physics::Forces::earth_gravitation());
    sim.add_force(Physics::Forces::damping(0.15f));

    auto obj1 = regular_polygon(4, 20, {40, 22}, {0.0f, 0.0f}, 0.0f);
    obj1.m_phys_item.inertia *=2.0f;
    auto ref1 = sim.add_circle(obj1);
    auto ref2 = sim.add_circle(regular_polygon(4, 20, {77, 35}, {-15.0f, 0.0f}, 1.0f));
    auto ref3 = sim.add_circle(regular_polygon(5, 22, {60, 60}, {0.0f, 0.0f}, 1.0f));
    auto ref4 = sim.add_circle(regular_polygon(7, 25, {115, 85}, {10.0f, 0.0f}, -1.5f));

    auto& p0 = sim.get_point_ref({150.0f, 0.0f});
    auto& p1 = sim.get_point_ref(ref1);
    auto& p2 = sim.get_point_ref(ref2);
    auto& p3 = sim.get_point_ref(ref3);
    auto& p4 = sim.get_point_ref(ref4);
    auto& p5 = sim.get_point_ref({170.0f, 0.0f});

    struct spring_info {
        float initial_length;
        const Physics::PointReference* ref1;
        const Physics::PointReference* ref2;
        Physics::item_id_t id1;
        Physics::item_id_t id2;
        float resistance = 2.0f;
    };

    auto create_spring_from_info = [&](const auto& info) {
        auto spring = Physics::Forces::spring(
                info.resistance, 
                info.initial_length,
                info.id1, info.id2,
                *info.ref1, *info.ref2
        );
        sim.add_force(spring);
    };

    spring_info info4{ glm::distance(p4.get(), p5.get()), &p5, &p4, Physics::item_id_t(-1), ref4.get().m_phys_item.id, 4.0f};
    spring_info info1{ glm::distance(p0.get(), p1.get()), &p0, &p1, Physics::item_id_t(-1), ref1.get().m_phys_item.id};
    spring_info info2{ glm::distance(p1.get(), p2.get()), &p1, &p2, ref1.get().m_phys_item.id, ref2.get().m_phys_item.id};
    spring_info info3{ glm::distance(p2.get(), p3.get()), &p2, &p3, ref2.get().m_phys_item.id, ref3.get().m_phys_item.id};
    const auto infos = {info1, info2, info3, info4};
    for (const auto& info : infos) {
        create_spring_from_info(info);
    }

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 1.0f/4000.0f, .fps_limit = 2000.0f, .start_delay=5s }));

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

        auto draw_spring = [&v](glm::vec2 from, glm::vec2 to, float initial_length) {
            float num_iters = initial_length / 15.0f;
            v.draw_circle(from, 5.0f, {0, 128, 128, 255});
            v.draw_circle(to, 5.0f, {0, 128, 128, 255});
            float length = glm::distance(from, to);
            float step = length / (num_iters + 2.0f);
            float offset = 0.0f;
            glm::vec2 direction = glm::normalize(to - from);
            glm::vec2 offset_dir = {-direction.y, direction.x};
            float iter = 1.0f;
            glm::vec2 last_point = from;
            float offset_mult = initial_length / 20.0f;
            while (iter < num_iters) {
                iter += 1.0f;
                offset_mult *= -1.0f;

                glm::vec2 point = from + step * iter * direction + offset_dir * offset_mult;
                v.draw_line(last_point, point, {255, 0, 0, 255});
                last_point = point;
            }
            v.draw_line(last_point, to, {255, 0, 0, 255});
        };
        for (const auto& info : infos) {
            draw_spring(info.ref1->get(), info.ref2->get(), info.initial_length);
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
