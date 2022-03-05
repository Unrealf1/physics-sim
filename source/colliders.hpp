#pragma once

#include "glm/geometric.hpp"
#include <climits>
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
    requires (T collider, float border, glm::vec2 point) {
        { collider.is_colliding_x(border) } -> std::same_as<bool>;
        { collider.is_colliding_y(border) } -> std::same_as<bool>;
        { collider.is_colliding(collider) } -> std::same_as<bool>;
        { collider.is_colliding(point, point) } -> std::same_as<bool>;
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
        bool is_colliding_x(float) const { return false; }
        bool is_colliding_y(float) const { return false; }
        bool is_colliding(const EmptyCollider&) const { return false; }
    };

    struct CircleCollider : public BasicMovable {
    public:
        bool is_colliding_x(float border) const {
            return std::abs(m_position.x - border) <= m_radius;
        }
        bool is_colliding_y(float border) const {
            return std::abs(m_position.y - border) <= m_radius;
        }
        bool is_colliding(const CircleCollider& other) const { return glm::distance(other.m_position, m_position) <= other.m_radius + m_radius; }
        bool is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
            float a = ray_point.x * ray_point.x + ray_point.y * ray_point.y;
            float b = 2.0f * (ray_start.x * ray_point.x - ray_point.x * m_position.x + ray_start.y * ray_point.y - ray_point.y * m_position.y);
            float c = ray_start.x * ray_start.x + m_position.x * m_position.x 
                + ray_start.y * ray_start.y + m_position.y * m_position.y
                - m_radius * m_radius
                - 2.0f * (ray_start.x * m_position.x + ray_start.y * m_position.y);

            return (b * b) >= 4.0f * a * c;
        }

    public:
        float m_radius;
    };

}
