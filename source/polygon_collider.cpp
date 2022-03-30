#include "colliders.hpp"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"

#include <cmath>
#include <functional>
#include <spdlog/spdlog.h>

#include <ranges>
#include <algorithm>
#include <deque>
#include <optional>
#include <limits>

using std::views::transform;
using std::ranges::minmax_element;
using std::views::drop;
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
            if (SameDirection(AC, AO)) {
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
    size_t iteration = 0;
    while (true) {
		auto new_point = SupportMapping(current_direction, points1, points2);
 
        if (!SameDirection(new_point, current_direction)) {
            return false;
        }

		simplex.push(new_point);
        if (iteration > 105) {
            spdlog::warn("situation:\ndirection: {},{}; simplex: {},{}; {},{}; {},{}; (size={}, dot={})\nfigure1: {},{}; {},{}; {},{}\nfigure2: {},{}; {},{}; {},{}",
                current_direction.x, current_direction.y,
                simplex.points[0].x, simplex.points[0].y,
                simplex.points[1].x, simplex.points[1].y,
                simplex.points[2].x, simplex.points[2].y,
                simplex.size,
                glm::dot(new_point, current_direction),
                (*(points1.begin())).x, (*(points1.begin())).y,
                (*(points1.begin() + 1)).x, (*(points1.begin() + 1)).y,
                (*(points1.begin() + 2)).x, (*(points1.begin() + 2)).y,
                (*(points2.begin())).x, (*(points2.begin())).y,
                (*(points2.begin() + 1)).x, (*(points2.begin() + 1)).y,
                (*(points2.begin() + 2)).x, (*(points2.begin() + 2)).y
            );
        }
        if (NextSimplex(simplex, current_direction)) {
			return true;
		}
        if (iteration > 105) {
        spdlog::warn("after simplex update:\ndirection: {},{}; simplex: {},{}; {},{}; {},{}; (size={})",
                current_direction.x, current_direction.y,
                simplex.points[0].x, simplex.points[0].y,
                simplex.points[1].x, simplex.points[1].y,
                simplex.points[2].x, simplex.points[2].y,
                simplex.size);
        }
        ++iteration;

        if (iteration > 110) {
            spdlog::error("Too many simplex iterations");
            return false;
        }
    }
}

template<typename PointRange>
bool is_inside(const glm::vec2 point, const PointRange& figure) {
    float sign = 0.0f;
    {
        glm::vec2 figure_point = *figure.begin();
        glm::vec2 previous_point = *figure.rbegin();
        auto vec = point - figure_point;
        auto edge = point - previous_point;
        sign = glm::dot(vec, edge) < 0.0f ? -1.0f : 1.0f;
    }
    auto previous_point = *figure.begin();
    for (const auto& figure_point : figure | drop(1)) {
        auto vec = point - figure_point;
        auto edge = point - previous_point;
        if (glm::dot(vec, edge) * sign > 0.0f) {
            return false;
        }
        previous_point = point;  
    }
    std::string str;
    for (const glm::vec2& p : figure ) {
        str += std::to_string(p.x) + ", " + std::to_string(p.y) + "; || ";
    }
    spdlog::info("point {},{} is inside this figure: {}", point.x, point.y, str);
    
    return true;
}

template<typename PointRange1, typename PointRange2>
std::optional<glm::vec2> find_vertex_inside(const PointRange1& points1, const PointRange2& points2) {
    for (const auto& p : points1) {
        if (is_inside(p, points2)) {
            return p;
        }
    }
    return {};
}

std::tuple<glm::vec2, glm::vec2, glm::vec2> PolygonCollider::get_collision_points_and_normal(const PolygonCollider& other) const {
    auto other_direction = other.m_position - m_position;
    auto points1 = get_world_points();
    auto points2 = other.get_world_points();
    
    auto p1 = find_vertex_inside(points1, points2);
    glm::vec2 penetration_point;
    bool first_penetrating = false;
    if (p1.has_value()) {
        penetration_point = p1.value();
        first_penetrating = true;
    } else {
        auto p2 = find_vertex_inside(points2, points1);
        //TODO: do I need to think about case where point is not found?
        penetration_point = p2.value_or(other.m_position);
    }
    float closeness = std::numeric_limits<float>::max();
    
    auto& penetrator = first_penetrating ? points1 : points2;
    auto& penetrated = !first_penetrating ? points1 : points2;
    glm::vec2 normal;
    glm::vec2 edge;
    glm::vec2 edge_point;
    auto previous_point = *penetrated.begin();
    for (const auto& point : penetrated | drop(1)) {
        float difference = glm::distance(penetration_point, previous_point) + glm::distance(penetration_point, point) - glm::distance(previous_point, point);
        if (difference < closeness) {
            edge_point = point;
            edge = point - previous_point;
            normal = glm::normalize(glm::vec2(-edge.y, edge.x));
            closeness = difference;
        }
        previous_point = point;  
    }
    
    {
        glm::vec2 point = *penetrated.begin();
        float difference = glm::distance(penetration_point, previous_point) + glm::distance(penetration_point, point) - glm::distance(previous_point, point);
        if (difference < closeness) {
            edge_point = point;
            edge = point - previous_point;
            normal = glm::normalize(glm::vec2(-edge.y, edge.x));
            closeness = difference;
        }
    }
    
    auto A = edge_point;
    auto a = edge;
    auto B = penetration_point;
    auto b = normal;
    auto tb = (a.x * (B.y - A.y) - a.y * (B.x - A.x)) / (b.x * a.y - b.y * a.x);
    auto projected_point = B + tb * b;

    return { penetration_point, projected_point, normal };
}

//TODO: ?
bool PolygonCollider::is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
    return false;
}

}
