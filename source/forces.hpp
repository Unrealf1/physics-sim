
#pragma once

#include <glm/glm.hpp>
#include <functional>
#include <cassert>

#include "physics_item.hpp"


namespace Physics {
    //TODO: somehow pass information about collider to force calculation and filter
    //TODO: get rid of std::funciton (maybe store void* in Force or something like that...)
    //TODO: instead of filtering store references to necessary items inside the Force? (for all-affecting forces maybe store ref to simulation)
    struct Force {
        //using filter_t = bool (*)(const PhysicsItem&);
        using filter_t = std::function<bool(const PhysicsItem&)>;
        using calc_t = std::function<glm::vec2(const PhysicsItem&)>;
        using calc_torque_t = std::function<float(const PhysicsItem&)>;

        bool is_affecting(const PhysicsItem& item) { return m_filter(item); }
        glm::vec2 calculate (const PhysicsItem& item) { return m_calc(item); }
        float calculate_torque (const PhysicsItem& item) { return m_calc_torque(item); }

        calc_t m_calc;
        calc_torque_t m_calc_torque = [](const PhysicsItem&) { return 0.0f; };
        filter_t m_filter = [](const PhysicsItem&) { return true; };
    };

    namespace Forces {
        inline float calc_universal_gravitation(float m1, float m2, float distance, float gravitational_constant) {
            return gravitational_constant * m1 * m2 / (distance * distance);
        }

        inline constexpr static float c_gravitational_constant = 6.67e-11f;

        inline Force universal_gravitation(float mass, glm::vec2 source, float gravitational_constant = c_gravitational_constant) {
            auto calc = [=](const PhysicsItem& item) { 
                float power = calc_universal_gravitation(item.mass, mass, glm::distance(item.position, source), gravitational_constant);
                glm::vec2 direction = glm::normalize(source - item.position);
                return direction * power;
            };
            return { calc };
        }

        inline Force earth_gravitation() {
            return { [](const PhysicsItem& item) -> glm::vec2 { return { 0.0f, 9.8f * item.mass }; } };
        }

        inline Force damping(float k) {
            auto calc = [=](const PhysicsItem& item) {
                return - k * item.speed;
            };
            return { calc };
        }
        
        Force spring(
                float k, 
                float initial_length, 
                const Physics::item_id_t& first, 
                const Physics::item_id_t& second,
                const PointReference& first_point,
                const PointReference& second_point
        );
    }
}
