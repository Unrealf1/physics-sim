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
        float m_rotation = 0.0f;

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
        bool is_colliding_x(float border) const; 
        bool is_colliding_y(float border) const; 
        bool is_colliding(const CircleCollider& other) const;
        std::tuple<glm::vec2, glm::vec2, glm::vec2> get_collision_points_and_normal(const CircleCollider& other) const;
        bool is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const;

    public:
        float m_radius;
    };
    struct PolygonCollider : public BasicMovable {
    public:
        bool is_colliding_x(float border) const; 
        bool is_colliding_y(float border) const; 
        //bool is_colliding(const CircleCollider& other) const;
        bool is_colliding(const PolygonCollider& other) const;
        std::tuple<glm::vec2, glm::vec2, glm::vec2> get_collision_points_and_normal(const PolygonCollider& other) const;
        bool is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const;
        std::vector<glm::vec2> get_world_points() const;

    public:
        std::vector<glm::vec2> m_vertices;
    };

    //TODO: return all collision data in one struct to reduce recalculation?
    struct StaticCollider {
    public:
        virtual bool is_colliding(const CircleCollider& circle) const = 0;
        virtual bool is_colliding(const PolygonCollider& circle) const = 0;
        virtual glm::vec2 collision_point(const CircleCollider& circle) const = 0;
        virtual glm::vec2 collision_point(const PolygonCollider& circle) const = 0;
        virtual float distance(glm::vec2 point) const = 0;
        virtual void visualize(Visualizer& vis) const = 0;
        virtual ~StaticCollider() = default;
    };

    struct StaticSegmentCollider : public StaticCollider {
    public:
        StaticSegmentCollider(glm::vec2 start, glm::vec2 end);

        bool is_colliding(const CircleCollider& circle) const override;
        bool is_colliding(const PolygonCollider& circle) const override { return false; }

        //TODO implement or remove
        float distance(glm::vec2) const override;
        glm::vec2 collision_point(const CircleCollider& circle) const override;
        glm::vec2 collision_point(const PolygonCollider&) const override { return {}; };
        void visualize(Visualizer& vis) const override;
    
    private:
        std::tuple<float, float, bool> get_ts(const CircleCollider& circle) const;
    
        glm::vec2 m_start;
        glm::vec2 m_end;
        glm::vec2 m_diff;
    };

    struct StaticCircleCollider : public StaticCollider {
    public:
        StaticCircleCollider(glm::vec2 position, float radius);
        
        bool is_colliding(const CircleCollider& circle) const override;
        bool is_colliding(const PolygonCollider& circle) const override { return false; }
        float distance(glm::vec2 point) const override;
        glm::vec2 collision_point(const CircleCollider& circle) const override;
        glm::vec2 collision_point(const PolygonCollider&) const override { return {}; };
        void visualize(Visualizer& vis) const override;

    private:
        CircleCollider m_collider;
    };

}
