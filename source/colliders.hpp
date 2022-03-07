#pragma once

#include "glm/geometric.hpp"
#include <climits>
#include <cwchar>
#include <glm/glm.hpp>
#include <tuple>
#include <cmath>

#include <spdlog/spdlog.h>

#include "visualizer.hpp"


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
        bool is_colliding(glm::vec2, glm::vec2) const { return false; };
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
            auto d = ray_point - ray_start;
            auto& p0 = ray_start;
            float a = d.x * d.x + d.y * d.y;
            float b = 2.0f * (p0.x * d.x - d.x * m_position.x + p0.y * d.y - d.y * m_position.y);
            float c = p0.x * p0.x + m_position.x * m_position.x 
                + p0.y * p0.y + m_position.y * m_position.y
                - m_radius * m_radius
                - 2.0f * (p0.x * m_position.x + p0.y * m_position.y);
            float D = std::sqrt((b * b) - 4.0f * a * c);
            return (D > b) | (D > -b);
        }

    public:
        float m_radius;
    };

    //TODO: return all collision data in one struct to reduce recalculation?
    struct StaticCollider {
    public:
        virtual bool is_colliding(const CircleCollider& circle) const = 0;
        virtual glm::vec2 collision_point(const CircleCollider& circle) const = 0;
        virtual float distance(glm::vec2 point) const = 0;
        virtual void visualize(Visualizer& vis) const = 0;
        virtual ~StaticCollider() = default;
    };

    struct StaticSegmentCollider : public StaticCollider {
        StaticSegmentCollider(glm::vec2 start, glm::vec2 end) 
            : m_start(start)
            , m_end(end)
            , m_diff(glm::normalize(end - start)) {}

        bool is_colliding(const CircleCollider& circle) const override {
            auto [t1, t2, ts_exist] = get_ts(circle);
            auto true_t = glm::length(m_end - m_start);

            return ts_exist & (((t1 >= 0.0f) & (t1 <= true_t)) | ((t2 >= 0.0f) & (t2 <= true_t)));
        }

        //TODO ?
        float distance(glm::vec2) const override {
            return 0.0f;
        }

        glm::vec2 collision_point(const CircleCollider& circle) const override {
            auto [t1, t2, ts_exist] = get_ts(circle);
            return m_start + (t1 + t2) / 2.0f * m_diff;
        }

        virtual void visualize(Visualizer& vis) const override {
            vis.draw_line(m_start, m_end, {255, 255, 255, 255});
        }

        std::tuple<float, float, bool> get_ts(const CircleCollider& circle) const {
            glm::vec2 delta = m_start - circle.m_position;
            float a = m_diff.x * m_diff.x + m_diff.y * m_diff.y;
            float b = 2.0f * (delta.x * m_diff.x + delta.y * m_diff.y);           
            float c = delta.x * delta.x + delta.y * delta.y - circle.m_radius * circle.m_radius;    
            float D = b * b - 4 * a * c;
            if (D < 0.0f) {
                return {0.0f, 0.0f, false};
            }

            float t1 = (-b + sqrtf(D)) / (2.0f * a);
            float t2 = (-b - sqrtf(D)) / (2.0f * a);
            return {t1, t2, true};
        }
    
        glm::vec2 m_start;
        glm::vec2 m_end;
        glm::vec2 m_diff;
    };

    struct StaticCircleCollider : public StaticCollider {
        StaticCircleCollider(glm::vec2 position, float radius) : m_collider{{position}, radius} {}
        
        bool is_colliding(const CircleCollider& circle) const override {
            return m_collider.is_colliding(circle);
        }

        float distance(glm::vec2 point) const override {
            return glm::distance(point, m_collider.m_position);
        }
        glm::vec2 collision_point(const CircleCollider& circle) const override {
            auto dp = m_collider.m_position - circle.m_position;
            return circle.m_position + glm::normalize(dp) * circle.m_radius;
        }

        void visualize(Visualizer& vis) const override  {
            vis.draw_circle(m_collider.m_position, m_collider.m_radius, {255, 255, 255, 255});
        }

        CircleCollider m_collider;
    };

}
