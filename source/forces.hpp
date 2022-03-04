
#pragma once

#include <glm/glm.hpp>
#include <functional>

#include "physics_item.hpp"


namespace Physics {

    //TODO: somehow pass information about collider to force calculation and filter
    //TODO: get rid of std::funciton (maybe store void* in Force or something like that...)
    struct Force {
        using filter_t = bool (*)(const PhysicsItem&);
        using calc_t = std::function<glm::vec2(const PhysicsItem&)>;

        bool is_affecting(const PhysicsItem& item) { return m_filter(item); }
        glm::vec2 calculate (const PhysicsItem& item) { return m_calc(item); }

        calc_t m_calc;
        filter_t m_filter = [](const PhysicsItem&) { return true; };
    };

    
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

}
