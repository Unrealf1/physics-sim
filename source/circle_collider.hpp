#pragma once

#include "glm/geometric.hpp"
#include <glm/glm.hpp>


namespace Physics {

    struct CircleCollider {
    public:
        bool is_colliding(const glm::vec2& point) { return glm::distance(point, m_center) <= m_radius; }
        bool is_colliding(const CircleCollider& other) { return glm::distance(other.m_center, m_center) <= other.m_radius + m_radius; }

    public:
        glm::vec2 m_center;
        float m_radius;
    };



}
