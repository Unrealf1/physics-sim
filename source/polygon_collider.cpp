#include "colliders.hpp"
#include "glm/fwd.hpp"

#include <cmath>
#include <spdlog/spdlog.h>

#include <ranges>
#include <algorithm>
#include <deque>

using std::views::transform;
using std::ranges::minmax_element;
using std::ranges::max_element;

namespace Physics {


// PolygonCollider
std::vector<glm::vec2> PolygonCollider::get_world_points() const {
    float cos = std::cos(m_rotation);
    float sin = std::sin(m_rotation);
    auto rotate = [this, sin, cos](const glm::vec2& vertex) -> glm::vec2 { return {vertex.x * cos - vertex.y * sin, vertex.y * cos + vertex.x * sin}; };
    auto translate = [this](const glm::vec2& vertex) -> glm::vec2 {return vertex + m_position;};
    auto points = m_vertices | transform(rotate) | transform(translate);
    return {points.begin(), points.end()};
}

bool PolygonCollider::is_colliding_x(float border) const {
    auto points = get_world_points();
    auto dx = points
        | transform([this, border](const glm::vec2& vertex) { return vertex.x  - border; });
    const auto [min, max] = minmax_element(dx);
    return (*min) * (*max) < 0.0f;
}

bool PolygonCollider::is_colliding_y(float border) const {
    auto points = get_world_points();
    auto dy = points
        | transform([this, border](const glm::vec2& vertex) { return vertex.y - border; });
    const auto [min, max] = minmax_element(dy);
    return (*min) * (*max) < 0.0f;
}

struct Simplex {
    void push(glm::vec2 point) {
        points = {point, points[0], points[1]};
        size = std::min(size + 1, size_t(3));
    }

    bool has_origin() {
        return false;
    }

    std::array<glm::vec2, 3> points;
    size_t size = 0;
};

template<typename PointRange>
glm::vec2 FindFurthest(glm::vec2 direction, const PointRange& points) {
    auto dots = points | transform([&direction](const glm::vec2& point) { return glm::dot(direction, point); });
    auto max_it = max_element(dots);
    return *(points.begin() + (max_it - dots.begin()));
}

template<typename PointRange1, typename PointRange2>
glm::vec2 SupportMapping(glm::vec2 direction, const PointRange1& points1, const PointRange2& points2) {
    return FindFurthest(direction, points1) - FindFurthest(-direction, points2);
}

bool SameDirection(glm::vec2 first, glm::vec2 second) {
    return glm::dot(first, second) > 0.0f;
}

bool NextSimplex(Simplex& simplex, glm::vec2& direction) {
    //TODO: remove copypaste and make more readable
    if (simplex.size == 2) {
        // Line case
        glm::vec2 AB = simplex.points[1] - simplex.points[0];
        glm::vec2 AO = -simplex.points[0];
        if (SameDirection(AB, AO)) {
            // AB is still the simplex
            // direction is perpendicular to AB to the origin
            // TODO: remove if, use cross product?
            direction = glm::vec2(-AB.y, AB.x);
            if (!SameDirection(direction, AO)) {
                direction *= -1;
            }
        } else {
            // A is the simplex now
            simplex.size = 1;
            direction = AO;
        }

        return false;
    } else {
        // Triangle case
        glm::vec2 AC = simplex.points[2] - simplex.points[0];
        glm::vec2 BC = simplex.points[2] - simplex.points[1];
        glm::vec2 AB = simplex.points[1] - simplex.points[0];

        glm::vec2 ACn = glm::vec2(-AC.y, AC.x);
        if (!SameDirection(ACn, BC)) {
            ACn *= -1;
        }
        glm::vec2 ABn = glm::vec2(-AB.y, AB.x);
        if (SameDirection(ABn, BC)) {
            ABn *= -1;
        }

        glm::vec2 AO = -simplex.points[0];

        if (SameDirection(ACn, AO)) {
            if (!SameDirection(AC, AO)) {
                // AC is the simplex
                simplex.size = 2;
                simplex.points[1] = simplex.points[2];
                direction = ACn;
            } else {
                // A or AB
                if (SameDirection(AB, AO)) {
                    // AB
                    simplex.size = 2;
                    direction = ABn;
                } else {
                    // A
                    simplex.size = 1;
                    direction = AO;
                }
            }
        } else {
            if (SameDirection(ABn, AO)) {
                // A or AB
                if (SameDirection(AB, AO)) {
                    // AB
                    simplex.size = 2;
                    direction = ABn;
                } else {
                    // A
                    simplex.size = 1;
                    direction = AO;
                }
            } else {
                return true;
            }
            
        }
    }   
    return false;
}

bool PolygonCollider::is_colliding(const PolygonCollider& other) const {
    glm::vec2 initial_direction{1.0f, 0.0f};
    Simplex simplex;
    auto points1 = get_world_points();
    auto points2 = other.get_world_points();
    simplex.push(SupportMapping(initial_direction, points1, points2));
    auto current_direction = -simplex.points[0];
    //size_t iteration = 0;
    while (true) {
		auto new_point = SupportMapping(current_direction, points1, points2);
 
        if (!SameDirection(new_point, current_direction)) {
            return false;
        }

		simplex.push(new_point);
        if (NextSimplex(simplex, current_direction)) {
			return true;
		} 
        //++iteration;

        //if (iteration > 110) {
        //    spdlog::error("Too many simplex iterations");
        //    return false;
        //}
    }
}

glm::vec2 PolygonCollider::get_collision_point(const PolygonCollider& other) const {
    auto other_direction = other.m_position - m_position;
    auto points1 = get_world_points();
    auto points2 = other.get_world_points();
    //TODO: is this correct? :)
    auto p1 = FindFurthest(other_direction, points1);
    auto p2 = FindFurthest(other_direction, points2);

    auto d1 = glm::distance(m_position, p1) + glm::distance(p1, other.m_position);
    auto d2 = glm::distance(m_position, p2) + glm::distance(p2, other.m_position);
    if (d1 < d2) {
        return p1;
    } else {
        return p2;
    }
}

//TODO: ?
bool PolygonCollider::is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
    return false;
}

}
