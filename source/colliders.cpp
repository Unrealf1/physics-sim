#include "colliders.hpp"

namespace Physics {

// CricleCollider
bool CircleCollider::is_colliding_x(float border) const {
    return std::abs(m_position.x - border) <= m_radius;
}

bool CircleCollider::is_colliding_y(float border) const {
    return std::abs(m_position.y - border) <= m_radius;
}

bool CircleCollider::is_colliding(const CircleCollider& other) const { 
    return glm::distance(other.m_position, m_position) <= other.m_radius + m_radius; 
}
bool CircleCollider::is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
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

// StaticSegmentCollider
StaticSegmentCollider::StaticSegmentCollider(glm::vec2 start, glm::vec2 end) 
    : m_start(start)
    , m_end(end)
    , m_diff(glm::normalize(end - start)) {}

bool StaticSegmentCollider::is_colliding(const CircleCollider& circle) const {
    auto [t1, t2, ts_exist] = get_ts(circle);
    auto true_t = glm::length(m_end - m_start);

    return ts_exist & (((t1 >= 0.0f) & (t1 <= true_t)) | ((t2 >= 0.0f) & (t2 <= true_t)));
}

glm::vec2 StaticSegmentCollider::collision_point(const CircleCollider& circle) const {
    auto [t1, t2, ts_exist] = get_ts(circle);
    return m_start + (t1 + t2) / 2.0f * m_diff;
}

void StaticSegmentCollider::visualize(Visualizer& vis) const {
    vis.draw_line(m_start, m_end, {255, 255, 255, 255});
}

//TODO implement or remove
float StaticSegmentCollider::distance(glm::vec2) const {
    return 0.0f;
}

std::tuple<float, float, bool> StaticSegmentCollider::get_ts(const CircleCollider& circle) const {
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

// StaticCircleCollider

StaticCircleCollider::StaticCircleCollider(glm::vec2 position, float radius) 
    : m_collider{{position}, radius} {}

bool StaticCircleCollider::is_colliding(const CircleCollider& circle) const {
    return m_collider.is_colliding(circle);
}

float StaticCircleCollider::distance(glm::vec2 point) const {
    return glm::distance(point, m_collider.m_position);
}
glm::vec2 StaticCircleCollider::collision_point(const CircleCollider& circle) const {
    auto dp = m_collider.m_position - circle.m_position;
    return circle.m_position + glm::normalize(dp) * circle.m_radius;
}

void StaticCircleCollider::visualize(Visualizer& vis) const {
    vis.draw_circle(m_collider.m_position, m_collider.m_radius, {255, 255, 255, 255});
}

}

