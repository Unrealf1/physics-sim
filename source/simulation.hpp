#pragma once

#include <functional>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <chrono>
#include <mutex>

#include <spdlog/spdlog.h>

#include "glm/geometric.hpp"
#include "simulation_object.hpp"
#include "integrators.hpp"
#include "collision_detectors.hpp"


namespace Physics {
    template<Integrator integrator_t, CollisionDetector collision_detector_t>
    class Simulation {
    public:
        class ObjectReference {
        public:
            ObjectReference(const Simulation& sim, size_t idx)
                : simulation(sim), index(idx) {}

            const SimulationObject& get() const {
                return simulation.get_last_buffer()[index];
            }

        private:
            const Simulation& simulation;
            size_t index;
        };

        class ObjectRelatedPointReference : public PointReference {
        public:
            ObjectRelatedPointReference(const ObjectReference& obj, glm::vec2 offset)
                : m_object_ref(obj), m_offset(offset) {}

            glm::vec2 get() const override {
                return m_object_ref.get().m_phys_item.position + m_offset;
            }
        private:
            ObjectReference m_object_ref;
            glm::vec2 m_offset;
        };

    public:
        Simulation(glm::vec2 simulation_rectangle) 
            : m_simulation_rectangle(simulation_rectangle)
            , m_collision_detector(simulation_rectangle) {}

        using objects_t = std::vector<SimulationObject>;

        objects_t get_objects() const {
            std::lock_guard lg(m_objects_lock);
            return get_last_buffer();    
        }
        
        ObjectReference get_object_ref(const item_id_t& id) const {
            const auto& objects = get_last_buffer();
            auto it = std::find_if(objects.begin(), objects.end(), [&id](const SimulationObject& obj) { return obj.m_phys_item.id == id; });
            if (it == objects.end()) {
                throw std::out_of_range("Simulation does not have object with id " + std::to_string(id));
            }
            return { *this, std::distance(it, objects.begin()) };
        }

        const PointReference& get_point_ref(const ObjectReference& obj_ref, glm::vec2 offset = {0.0f, 0.0f}) {
            m_point_references.push_back(std::make_unique<ObjectRelatedPointReference>(obj_ref, offset));
            return *(m_point_references.back().get());
        }
        const PointReference& get_point_ref(glm::vec2 point) {
            m_point_references.push_back(std::make_unique<FixedPointReference>(point));
            return *(m_point_references.back().get());
        }

        const auto& get_static_colliders() const {
            return m_static_colliders;
        }

        ObjectReference add_circle(const SimulationObject& obj) {
            auto& last_buffer = get_last_buffer();
            last_buffer.push_back(obj);
            // so initial setup will work
            //get_active_buffer().push_back(obj);
            return { *this, last_buffer.size() - 1 };
        }

        void add_force(const Force& force) {
            m_integrator.add_force(force);
        }

        void add_static_collider(std::unique_ptr<StaticCollider>&& collider) {
            m_static_colliders.push_back(std::move(collider));
        }

        void step(float dt) {
            const auto& last_buffer = get_last_buffer();
            auto& active_buffer = get_active_buffer();
            active_buffer.clear();

            // not default phys_item construction so new item ids are not taken
            active_buffer.resize(last_buffer.size(), {{}, {0.0f, glm::vec2{0.0f, 0.0f}, glm::vec2{0.0f, 0.0f}, size_t(-1)}});
            
            const auto collisions = m_collision_detector.detect_collisions(last_buffer, m_static_colliders);
            //const auto integrated = last_buffer 
            //    | std::views::transform([this, dt](const SimulationObject& obj) { return m_integrator.update(dt, obj.m_phys_item); } );
            
            std::transform(
                std::execution::par_unseq,
                std::begin(last_buffer), std::end(last_buffer),
                std::begin(collisions),
                std::begin(active_buffer),
                // Call to member function instead of direct code in lambda resulted in -5fps.
                // TODO: investigate
                [this, dt](const SimulationObject& obj, const Collision& collision) { return process_object(dt, obj, collision); }
            );
            switch_buffers(); 
        }

        const glm::vec2& get_simulation_rectangle() const {
            return m_simulation_rectangle;
        }

    private:
        mutable std::mutex m_objects_lock;
        objects_t m_objects_arrays[2];
        collision_detector_t m_collision_detector;

        std::vector<std::unique_ptr<StaticCollider>> m_static_colliders;
        std::vector<std::unique_ptr<PointReference>> m_point_references;
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
            std::lock_guard lg(m_objects_lock);
            m_active_index = 1 - m_active_index;
        }

        SimulationObject process_object(float dt, SimulationObject copy, const Collision& collision) {
            // process collisions
            for (const SimulationObject& collided_obj : collision.objects) {
                //TODO: calc integrator step once
                //for every item beforehand
                auto is_unnecessary = [&]{
                    auto new_item = m_integrator.update(dt, copy.m_phys_item);
                    auto new_item2 = m_integrator.update(dt, collided_obj.m_phys_item);
                    auto cur_dist = glm::distance(copy.m_phys_item.position, collided_obj.m_phys_item.position);
                    auto new_dist = glm::distance(new_item.position, new_item2.position);
                    return new_dist > cur_dist;
                };
                if (is_unnecessary()) {
                    continue;
                }            
                auto m1 = copy.m_phys_item.mass;
                auto m2 = collided_obj.m_phys_item.mass;
                float dm = m1 - m2;
                float sm = m1 + m2;

                auto& v1 = copy.m_phys_item.speed;
                auto& v2 = collided_obj.m_phys_item.speed;
                auto dp = copy.m_phys_item.position - collided_obj.m_phys_item.position;
                glm::vec2 dp_rot = { -dp.y, dp.x };
                auto touch_vec = glm::normalize(dp_rot);

                auto proj_len_1 = glm::dot(touch_vec, v1);
                auto proj_1 = touch_vec * proj_len_1; // not changing during interaction
                auto other_1 = v1 - proj_1; //changes during interaction
                
                auto proj_len_2 = glm::dot(touch_vec, v2);
                auto proj_2 = touch_vec * proj_len_2; // not changing during interaction
                auto other_2 = v2 - proj_2; //changes during interaction

                auto changed_v = (2 * m2 * other_2 + other_1 * dm) / sm;

                copy.m_phys_item.speed = proj_1 + changed_v;
#ifdef _DEBUG                
                if (copy.m_phys_item.speed != copy.m_phys_item.speed) {
                    spdlog::warn("m1: {}, m2: {}, proj_len_1: {}, proj_len_2: {}, dp: {}, {}", m1, m2, proj_len_1, proj_len_2, dp.x, dp.y);
                }
#endif
            }

            for (const StaticCollider* st_ptr : collision.statics) {
                const auto& st = *st_ptr;
                auto is_unnecessary = [&]{
                    auto new_item = m_integrator.update(dt / 20.0f, copy.m_phys_item);
                    auto cur_dist = st.distance(copy.m_phys_item.position);
                    auto new_dist = st.distance(new_item.position);
                    return new_dist > cur_dist;
                };
                if (is_unnecessary()) {
                    continue;
                }
                auto collision_point = st.collision_point(copy.m_collider);
                auto change_direction = glm::normalize(collision_point - copy.m_collider.m_position);
                auto proj_changed_len = glm::dot(change_direction, copy.m_phys_item.speed);
                auto proj_changed = change_direction * proj_changed_len;
                copy.m_phys_item.speed -= proj_changed * 2.0f;
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

    };

}

