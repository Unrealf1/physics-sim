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
#include <Eigen/Core>
#include <Eigen/Dense>

#include "glm/geometric.hpp"
#include "simulation_object.hpp"
#include "integrators.hpp"
#include "collision_detectors.hpp"

using std::views::iota;
using std::views::transform;
using std::ranges::all_of;

namespace Physics {
    template<typename object_t, Integrator integrator_t, CollisionDetector<object_t> collision_detector_t>
    class Simulation {
    public:
        class ObjectReference {
        public:
            ObjectReference(const Simulation& sim, size_t idx)
                : simulation(sim), index(idx) {}

            const object_t& get() const {
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

        using objects_t = std::vector<object_t>;

        objects_t get_objects() const {
            std::lock_guard lg(m_objects_lock);
            return get_last_buffer();    
        }
        
        ObjectReference get_object_ref(const item_id_t& id) const {
            const auto& objects = get_last_buffer();
            auto it = std::find_if(objects.begin(), objects.end(), [&id](const object_t& obj) { return obj.m_phys_item.id == id; });
            if (it == objects.end()) {
                throw std::out_of_range("Simulation does not have object with id " + std::to_string(id));
            }
            return { *this, size_t(std::distance(objects.begin(), it)) };
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
        
        //TODO: rename
        ObjectReference add_circle(const object_t& obj) {
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
            active_buffer.resize(last_buffer.size(), {{}, {0.0f, glm::vec2{0.0f, 0.0f}, glm::vec2{0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, size_t(-1)}});
            
            const auto collisions = m_collision_detector.detect_collisions(last_buffer, m_static_colliders);
            //const auto integrated = last_buffer 
            //    | std::views::transform([this, dt](const object_t& obj) { return m_integrator.update(dt, obj.m_phys_item); } );
            
            std::transform(
                std::execution::par_unseq,
                std::begin(last_buffer), std::end(last_buffer),
                std::begin(collisions),
                std::begin(active_buffer),
                // Call to member function instead of direct code in lambda resulted in -5fps.
                // TODO: investigate
                [this, dt](const object_t& obj, const Collision<object_t>& collision) { return process_object(dt, obj, collision); }
            );
            if (!all_of(collisions, [](const auto& collision) { return collision.objects.empty(); })) {
                using namespace std::chrono_literals;
                //std::this_thread::sleep_for(1s);
            }
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

        object_t process_object(float dt, object_t copy, const Collision<object_t>& collision) {
            // process collisions
            for (const object_t& collided_obj : collision.objects) {
                auto cross = [](const auto& vec1, const auto& vec2) -> float {
                    return (vec1.x * vec2.y) - (vec1.y * vec2.x);
                };
                
                auto scalar_cross = [](float scalar, const auto& vec) {
                    return decltype(vec){-vec.y, vec.x} * scalar;
                };

                auto [collision_point1, collision_point2, collision_normal] = copy.m_collider.get_collision_points_and_normal(collided_obj.m_collider);
                
                // Maybe just check if the point is in the list?..
                bool this_penetrated = glm::distance(collision_point1, copy.m_phys_item.position) < glm::distance(collision_point2, copy.m_phys_item.position);
                auto& collision_point = collision_point1;
                
                // Impulse solution
                auto& n = collision_normal;
                auto m1 = copy.m_phys_item.mass;
                auto m2 = collided_obj.m_phys_item.mass;
                auto i1 = copy.m_phys_item.inertia;
                auto i2 = collided_obj.m_phys_item.inertia;

                auto v1 = copy.m_phys_item.speed;
                auto v2 = collided_obj.m_phys_item.speed;
                auto w1 = copy.m_phys_item.rotation_speed;
                auto w2 = collided_obj.m_phys_item.rotation_speed;

                auto r1 = collision_point - copy.m_phys_item.position;
                auto r2 = collision_point - collided_obj.m_phys_item.position;

                // velocity of the collision point on a given body
                auto vp1 = v1 + scalar_cross(w1, r1);
                auto vp2 = v2 + scalar_cross(w2, r2);
                
                // relative velocity of the points
                auto vp = vp1 - vp2;
                
                // relative normal velocity
                auto vpn = glm::dot(vp, n);
                /*if (this_penetrated) {
                    vpn *= -1.0f;
                }*/

                if (vpn > 0.0f) {
                    // bodies are moving away from each other
                    continue;
                }
                
                float elasticity = 0.3f;
                float rn1 = cross(r1, n);
                float rn2 = cross(r2, n);
                float impulse_strength = (-(1 + elasticity) * glm::dot(vp, n)) / (1 / m1 + 1 / m2 + rn1*rn1/i1 + rn2*rn2/i2);

                float impulse_sign = this_penetrated ? -1.0f : 1.0f;
                impulse_strength *= impulse_sign;
                copy.m_phys_item.speed += impulse_strength * n / m1;
                copy.m_phys_item.rotation_speed += cross(r1, impulse_strength * n) / i1;

                // Constraints solution
                /*bool first_closer = glm::distance(collision_point1, copy.m_phys_item.position) < glm::distance(collision_point2, copy.m_phys_item.position);

                auto& my_p = first_closer ? collision_point2 : collision_point1;
                auto& other_p = !first_closer ? collision_point2 : collision_point1;
                auto pen_distance = glm::distance(collision_point1, collision_point2); //TODO: is this right?
                pen_distance = 1.0f;

                auto& n = collision_normal;
                auto m1 = copy.m_phys_item.mass;
                auto m2 = collided_obj.m_phys_item.mass;
                auto i1 = copy.m_phys_item.inertia;
                auto i2 = collided_obj.m_phys_item.inertia;

                auto v1 = copy.m_phys_item.speed;
                auto v2 = collided_obj.m_phys_item.speed;
                auto w1 = copy.m_phys_item.rotation_speed;
                auto w2 = collided_obj.m_phys_item.rotation_speed;
                
                Eigen::DiagonalMatrix<float, 6> M(1.0f/m1, 1.0f/m1, 1.0f/i1, 1.0f/m2, 1.0f/m2, 1.0f/i2);

                auto V = Eigen::Vector<float, 6>(v1.x, v1.y, w1, v2.x, v2.y, w2).transpose();
                
                auto cross = [](const auto& vec1, const auto& vec2) -> float {
                    return (vec1.x * vec2.y) - (vec1.y * vec2.x);
                };

                auto ra = my_p - copy.m_phys_item.position;
                auto rb = other_p - collided_obj.m_phys_item.position;
                
                auto J = Eigen::Vector<float, 6>{n.x, n.y, cross(ra, n), -n.x, -n.y, -cross(rb, n)}.transpose();                
                auto bias = 0.5f * std::max(pen_distance - 0.1f, 0.0f) / dt;
                
                auto A = J * M * J.transpose();
                auto B = -bias - J.dot(V);
                auto lambda = B / float(A);
                auto dV = M * J.transpose() * lambda;

                copy.m_phys_item.speed.x += dV[0];
                copy.m_phys_item.speed.y += dV[1];
                copy.m_phys_item.rotation_speed += dV[2];*/
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
            copy.m_collider.m_rotation = copy.m_phys_item.orientation;
            return copy;                  
        }

    };

}

