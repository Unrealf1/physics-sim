#pragma once

#include "glm/geometric.hpp"
#include <cwchar>
#include <glm/glm.hpp>
#include <tuple>


namespace Physics {
    
    template<typename T>
    concept Movable = requires(T t, glm::vec2 point) {
        { t.move_to(point) };
        { t.move_by(point) };
        { t.position() } -> std::same_as<glm::vec2>;
    };
    
    template<typename T>
    concept Collider = 
    Movable<T> &&
    requires (T collider, glm::vec2 point) {
        { collider.is_colliding(point) } -> std::same_as<bool>;
        { collider.is_colliding(collider) } -> std::same_as<bool>;
    };

    struct BasicMovable {
        glm::vec2 m_position;

        void move_to(glm::vec2 position) {
            m_position = position;
        }

        void move_by(glm::vec2 position) {
            m_position += position;
        }

        glm::vec2 position() const {
            return m_position;
        }
    };
    
    struct EmptyCollider : public BasicMovable {
        bool is_colliding(const glm::vec2&) const { return false; }
        bool is_colliding(const EmptyCollider&) const { return false; }
    };

    struct CircleCollider : public BasicMovable {
    public:
        bool is_colliding(const glm::vec2& point) const { return glm::distance(point, m_position) <= m_radius; }
        bool is_colliding(const CircleCollider& other) const { return glm::distance(other.m_position, m_position) <= other.m_radius + m_radius; }

    public:
        float m_radius;
    };

}
