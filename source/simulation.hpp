#pragma once

#include <memory>
#include <vector>
#include <algorithm>

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

        const objects_t& get_objects() const {
            return get_last_buffer();    
        }

        void add_circle(const SimulationObject& obj) {
            get_last_buffer().push_back(obj);
        }

        void add_force(const Force& force) {
            m_integrator.add_force(force);
        }

        void step(float dt) {
            auto& last_buffer = get_last_buffer();
            auto& active_buffer = get_active_buffer();
            active_buffer.clear();
            
            spdlog::info(
                    "simulation has {} elemetns. First coordinates are: {} - {}",
                    last_buffer.size(),
                    last_buffer[0].m_phys_item.position.x,                    
                    last_buffer[0].m_phys_item.position.y                    
            );

            auto collisions = collision_detector_t::detect_collisions(last_buffer);

            std::transform(
                    std::begin(last_buffer), std::end(last_buffer),
                    std::begin(collisions),
                    std::back_inserter(active_buffer),
                    [dt, this](const SimulationObject& obj, const collision_t& collision) -> SimulationObject {
                        auto copy = obj;
                        copy.m_phys_item = m_integrator.update(dt, obj.m_phys_item);
                        auto rays_collide = [](glm::vec2 p0, glm::vec2 p1, glm::vec2 q0, glm::vec2 q1) -> bool {
                            auto diff = p0 - q0;
                            glm::vec2 rotated = {-diff.y, diff.x};
                            float first = glm::dot(q1 - q0, rotated);
                            float second = glm::dot(p1 - q0, rotated);
                            bool first_condition = ((first >= 0) & (second >= 0)) | ((first < 0) & (second < 0));
                            
                            diff = p1 - p0;
                            rotated = {-diff.y, diff.x};
                            first = glm::dot(q1 - q0, rotated);
                            second = glm::dot(p1 - q0, rotated);
                            bool second_condition = ((first >= 0) & (second >= 0)) | ((first < 0) & (second < 0));
                            return first_condition & second_condition;
                        };
                        
                        // process collisions
                        for (SimulationObject& collided_obj : collision) {
                            if (!rays_collide(
                                    copy.m_phys_item.position, copy.m_phys_item.position + copy.m_phys_item.speed,
                                    collided_obj.m_phys_item.position, collided_obj.m_phys_item.position + collided_obj.m_phys_item.speed)) 
                            {
                                spdlog::info("collision between {} and {} is not important", copy.m_phys_item.id, collided_obj.m_phys_item.id);
                                continue;
                            }
                            spdlog::info("collision betweeb {} and {} is important! Rays:\n{},{} -> {},{}\n{},{} -> {},{}",
                                copy.m_phys_item.id, collided_obj.m_phys_item.id,
                                copy.m_phys_item.position.x, copy.m_phys_item.position.y, copy.m_phys_item.speed.x, copy.m_phys_item.speed.y,
                                collided_obj.m_phys_item.position.x, collided_obj.m_phys_item.position.y, collided_obj.m_phys_item.speed.x, collided_obj.m_phys_item.speed.y
                            );

                            glm::vec2 velocity = copy.m_phys_item.speed - collided_obj.m_phys_item.speed;
                            float dm = copy.m_phys_item.mass - collided_obj.m_phys_item.mass;
                            float sm = copy.m_phys_item.mass + collided_obj.m_phys_item.mass;
                            float k1 = dm / sm;
                            float k2 = 2 * copy.m_phys_item.mass / sm;
                            glm::vec2 new_velocity1 = k1 * velocity;
                            glm::vec2 new_velocity2 = k2 * velocity;
                            copy.m_phys_item.speed = new_velocity1 + collided_obj.m_phys_item.speed;
                            collided_obj.m_phys_item.speed = new_velocity2 + collided_obj.m_phys_item.speed;
                        }

                        if ((copy.m_collider.is_colliding_x(m_simulation_rectangle.x) & (copy.m_phys_item.speed.x > 0.0f))
                                | (copy.m_collider.is_colliding_x(0.0f) & (copy.m_phys_item.speed.x < 0.0f))) {
                            copy.m_phys_item.speed.x *= -1;   
                        }
                        if ((copy.m_collider.is_colliding_y(m_simulation_rectangle.y) & (copy.m_phys_item.speed.y > 0.0f))
                                | (copy.m_collider.is_colliding_y(0.0f) & (copy.m_phys_item.speed.y < 0.0f))) {
                            copy.m_phys_item.speed.y *= -1;   
                        }

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
        objects_t m_objects_arrays[2];
        integrator_t m_integrator;
        uint8_t m_active_index = 0; 
        glm::vec2 m_simulation_rectangle;

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

