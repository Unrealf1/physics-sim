#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <algorithm>
#include <atomic>
#include <chrono>

#include <spdlog/spdlog.h>

#include "simulation_object.hpp"
#include "integrators.hpp"
#include "collision_detectors.hpp"


namespace Physics {
    template<Integrator integrator_t, CollisionDetector collision_detector_t>
    class Simulation {
    public:
        Simulation(glm::vec2 simulation_rectangle) : m_simulation_rectangle(simulation_rectangle) {}

        using objects_t = std::vector<SimulationObject>;

        objects_t get_objects() const {
            return get_last_buffer();    
        }

        const links_t& get_links() const {
            return m_links;
        }

        void add_circle(const SimulationObject& obj) {
            get_last_buffer().push_back(obj);
        }

        void add_force(const Force& force) {
            m_integrator.add_force(force);
        }

        void add_link(const item_id_t& first, const item_id_t& second) {
            const auto& buf = get_last_buffer();

            m_links.emplace_back(
                  std::ranges::find_if(buf, [&](const auto& item) { return item.m_phys_item.id == first; }) - std::begin(buf),
                  std::ranges::find_if(buf, [&](const auto& item) { return item.m_phys_item.id == second; }) - std::begin(buf)
            );
        }

        void step(float dt) {
            const auto& last_buffer = get_last_buffer();
            auto& active_buffer = get_active_buffer();
            active_buffer.clear();
            active_buffer.resize(last_buffer.size());
            
            const auto collisions = collision_detector_t::detect_collisions(last_buffer, m_links);

            std::transform(
                std::execution::par_unseq,
                std::begin(last_buffer), std::end(last_buffer),
                std::begin(collisions),
                std::begin(active_buffer),
                [dt, this](const SimulationObject& obj, const collision_t& collision) -> SimulationObject {
                    auto copy = obj;
                    std::vector<item_id_t> linked;
                    for (const auto& link : m_links) {
                        if (link.first_idx == copy.m_phys_item.id) {
                            linked.push_back(link.second_idx);
                        } else if (link.second_idx == copy.m_phys_item.id) {
                            linked.push_back(link.first_idx);
                        }
                    }

                    auto rays_collide = [](glm::vec2 p0, glm::vec2 p1, glm::vec2 q0, glm::vec2 q1) -> bool {
                        auto dp = p1 - p0;
                        auto dq = q1 - q0;
                        float dx = q0.x - p0.x;
                        float dy = q0.y - p0.y;
                        float det = dq.x * dp.y - dq.y * dp.x;
                        if (det == 0.0f) {
                            return false;
                        }
                        float u = (dy * dq.x - dx * dq.y) / det;
                        float v = (dy * dp.x - dx * dp.y) / det;
                        return (u >= 0) & (v >= 0);

                        float tq = (q0.x * dp.y - dp.x * q0.y) / (dp.x * dq.y * p0.y - dq.x * p0.x * dp.y);
                        float tp = (tq * dq.x + q0.x - p0.x) / dp.x;
                        return tq >= 0.0f && tp >= 0.0f;

                    };
                    
                    // process collisions
                    for (const SimulationObject& collided_obj : collision) {
                        /*std::pair<glm::vec2, glm::vec2> my_ray = {
                            copy.m_phys_item.position, 
                            copy.m_phys_item.position + copy.m_phys_item.speed, 
                        };

                        std::pair<glm::vec2, glm::vec2> other_ray = {
                            collided_obj.m_phys_item.position, 
                            collided_obj.m_phys_item.position + copy.m_phys_item.speed, 
                        };*/
                        //TODO: calc integrator step once
                        //for every item beforehand
                        auto is_unnecessary = [&]{
                            auto it = std::ranges::find(linked, collided_obj.m_phys_item.id);
                            if (it != linked.end()) {
                                return false;
                            }
                            auto new_item = m_integrator.update(dt, copy.m_phys_item);
                            auto new_item2 = m_integrator.update(dt, collided_obj.m_phys_item);
                            auto cur_dist = glm::distance(copy.m_phys_item.position, collided_obj.m_phys_item.position);
                            auto new_dist = glm::distance(new_item.position, new_item2.position);
                            return new_dist > cur_dist;
                        };
                        if (is_unnecessary()) {
                            continue;
                        }            
                        auto& v1 = copy.m_phys_item.speed;
                        auto& v2 = collided_obj.m_phys_item.speed;
                        auto m1 = copy.m_phys_item.mass;
                        auto m2 = collided_obj.m_phys_item.mass;
                        float dm = m1 - m2;
                        float sm = m1 + m2;
                        copy.m_phys_item.speed = (2 * m2 * v2 + v1 * dm) / sm;
                        /*using namespace std::chrono_literals;
                        spdlog::warn("pause");
                        std::this_thread::sleep_for(500ms);*/
                    }

                    if ((copy.m_collider.is_colliding_x(m_simulation_rectangle.x) & (copy.m_phys_item.speed.x > 0.0f))
                            | (copy.m_collider.is_colliding_x(0.0f) & (copy.m_phys_item.speed.x < 0.0f))) {
                        copy.m_phys_item.speed.x *= -1;   
                    }
                    if ((copy.m_collider.is_colliding_y(m_simulation_rectangle.y) & (copy.m_phys_item.speed.y > 0.0f))
                            | (copy.m_collider.is_colliding_y(0.0f) & (copy.m_phys_item.speed.y < 0.0f))) {
                        copy.m_phys_item.speed.y *= -1;   
                    }

                    copy.m_phys_item = m_integrator.update(dt, copy.m_phys_item);
                    copy.m_collider.m_position = copy.m_phys_item.position;
                    return copy;                  
                }
                
            );
            switch_buffers(); 
        }

        const glm::vec2& get_simulation_rectangle() const {
            return m_simulation_rectangle;
        }

    private:
        std::mutex m_objects_lock;
        objects_t m_objects_arrays[2];
        integrator_t m_integrator;
        std::atomic<uint8_t> m_active_index = 0; 
        glm::vec2 m_simulation_rectangle;
        links_t m_links;

        objects_t& get_active_buffer() {
            return m_objects_arrays[m_active_index];
        }

        const objects_t& get_active_buffer() const {
            return m_objects_arrays[m_active_index];
        }

        objects_t& get_last_buffer() {
            return m_objects_arrays[1 - m_active_index];
        }

        const objects_t& get_last_buffer() const {
            return m_objects_arrays[1 - m_active_index];
        }

        void switch_buffers() {
            m_active_index = 1 - m_active_index;
        }
    };

}

