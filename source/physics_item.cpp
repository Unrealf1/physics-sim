#include "physics_item.hpp"


static Physics::item_id_t current_item_id = 0;

Physics::item_id_t Physics::generate_item_id() {
    return current_item_id++;    
}

// CricleCollider
bool Physics::SimulationCircle::is_colliding_x(float border) const {
    return std::abs(position.x - border) <= radius;
}

bool Physics::SimulationCircle::is_colliding_y(float border) const {
    return std::abs(position.y - border) <= radius;
}

bool Physics::SimulationCircle::is_colliding(const Physics::SimulationCircle& other) const {
    auto diff = other.position - position;
    auto square_dist = diff.x * diff.x + diff.y * diff.y;
    auto rad_sum = other.radius + radius;
    return square_dist <= rad_sum * rad_sum;
    //return glm::distance(other.position, position) <= other.radius + radius; 
}
bool Physics::SimulationCircle::is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
    auto d = ray_point - ray_start;
    auto& p0 = ray_start;
    float a = d.x * d.x + d.y * d.y;
    float b = 2.0f * (p0.x * d.x - d.x * position.x + p0.y * d.y - d.y * position.y);
    float c = p0.x * p0.x + position.x * position.x 
        + p0.y * p0.y + position.y * position.y
        - radius * radius
        - 2.0f * (p0.x * position.x + p0.y * position.y);
    float D = std::sqrt((b * b) - 4.0f * a * c);
    return (D > b) | (D > -b);
}
