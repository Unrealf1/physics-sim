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
                    //std::execution::par_unseq, TODO: what to do with double writes (both of colliding objects are affected)
                    std::begin(last_buffer), std::end(last_buffer),
                    std::begin(collisions),
                    std::back_inserter(active_buffer),
                    [dt, this](const SimulationObject& obj, const collision_t& collision) -> SimulationObject {
                        auto copy = obj;
                        copy.m_phys_item = m_integrator.update(dt, obj.m_phys_item);
                        //TODO: process collisions
                        auto rays_collide = [](glm::vec2 p0, glm::vec2 p1, glm::vec2 q0, glm::vec2 q1) -> bool {
                            auto diff = p0 - q0;
                            glm::vec2 rotated = {-diff.y, diff.x};
                            float first = glm::dot(q1 - q0, rotated);
                            float second = glm::dot(p1 - q0, rotated);
                            bool first_condition = (first >= 0 & second >= 0) | (first < 0 & second < 0);
                            
                            diff = p1 - p0;
                            rotated = {-diff.y, diff.x};
                            first = glm::dot(q1 - q0, rotated);
                            second = glm::dot(p1 - q0, rotated);
                            bool second_condition = (first >= 0 & second >= 0) | (first < 0 & second < 0);
                            return first_condition & second_condition;
                        };
                        
                        for (SimulationObject& collided_obj : collision) {
                            if (!rays_collide(
                                    copy.m_phys_item.position, copy.m_phys_item.speed,
                                    collided_obj.m_phys_item.position, collided_obj.m_phys_item.speed)) 
                            {
                                continue;
                            }

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
                        copy.m_collider.m_position = copy.m_phys_item.position;
                        return copy;                  
                    }
            );
            switch_buffers(); 
        }


    private:
        objects_t m_objects_arrays[2];
        integrator_t m_integrator;
        uint8_t m_active_index = 0; 

        objects_t& get_active_buffer() {
            return m_objects_arrays[m_active_index];
        }

        objects_t& get_last_buffer() {
            return m_objects_arrays[1 - m_active_index];
        }

        void switch_buffers() {
            m_active_index = 1 - m_active_index;
        }
    };

}

