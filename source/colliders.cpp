#include "colliders.hpp"

#include <ranges>
#include <algorithm>
#include <deque>

using std::views::transform;
using std::ranges::minmax_element;
using std::ranges::max_element;

namespace Physics {


// PolygonCollider
bool PolygonCollider::is_colliding_x(float border) const {
    auto dx = m_vertices
        | transform([this, border](const glm::vec2& vertex) { return (vertex.x + m_position.x) - border; });
    const auto [min, max] = minmax_element(dx);
    return (*min) * (*max) < 0.0f;
}
bool PolygonCollider::is_colliding_y(float border) const {
    auto dy = m_vertices
        | transform([this, border](const glm::vec2& vertex) { return (vertex.y + m_position.y) - border; });
    const auto [min, max] = minmax_element(dy);
    return (*min) * (*max) < 0.0f;
}

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

bool Line(std::deque<glm::vec2>& points, glm::vec2& direction) {
	glm::vec2 a = points[0];
	glm::vec2 b = points[1];

	glm::vec2 ab = b - a;
	glm::vec2 ao =   - a;

	if (SameDirection(ab, ao)) {
        direction = ao;
	} else {
		points = { a };
		direction = ao;
	}

	return false;
}

bool Triangle(std::deque<glm::vec2>& points, glm::vec2& direction) {
	glm::vec2 a = points[0];
	glm::vec2 b = points[1];
	glm::vec2 c = points[2];

	glm::vec2 ab = b - a;
	glm::vec2 ac = c - a;
	glm::vec2 ao =   - a;

    glm::vec2 abc = ab.cross(ac);

	if (SameDirection(abc.cross(ac), ao)) {
		if (SameDirection(ac, ao)) {
			points = { a, c };
			direction = ac.cross(ao).cross(ac);
		}

		else {
			return Line(points = { a, b }, direction);
		}
	}

	else {
		if (SameDirection(ab.cross(abc), ao)) {
			return Line(points = { a, b }, direction);
		}

		else {
			if (SameDirection(abc, ao)) {
				direction = abc;
			}

			else {
				points = { a, c, b };
				direction = -abc;
			}
		}
	}

	return false;
}

bool NextSimplex(std::deque<glm::vec2>& points, glm::vec2& direction) {
    if (points.size() == 2) {
        Line(points, direction);
    } else {
        Triangle(points, direction);
    }   
}

bool PolygonCollider::is_colliding(const PolygonCollider& other) const {
    glm::vec2 initial_direction{1.0f, 0.0f};
    std::deque<glm::vec2> simplex_points;
    auto points1 = m_vertices | transform([this](const glm::vec2& vertex) {return vertex + m_position;});
    auto points2 = other.m_vertices | transform([other](const glm::vec2& vertex) {return vertex + other.m_position;});
    simplex_points.push_front(SupportMapping(initial_direction, points1, points2));
    auto current_direction = -simplex_points[0];

    while (true) {
		auto new_point = SupportMapping(current_direction, points1, points2);
 
        if (glm::dot(new_point, current_direction) <= 0.0f) {
            return false;
        }

        if (simplex_points.size() > 2) {
            simplex_points.pop_back();
        }
		simplex_points.push_front(new_point);

        if (NextSimplex(simplex_points, current_direction)) {
			return true;
		}
    }
}

//TODO: ?
bool PolygonCollider::is_colliding(glm::vec2 ray_start, glm::vec2 ray_point) const {
    return false;
}

// CricleCollider
bool CircleCollider::is_colliding_x(float border) const {
    return std::abs(m_position.x - border) <= m_radius;
}

bool CircleCollider::is_colliding_y(float border) const {
    return std::abs(m_position.y - border) <= m_radius;
}

bool CircleCollider::is_colliding(const CircleCollider& other) const {
    auto diff = other.m_position - m_position;
    auto square_dist = diff.x * diff.x + diff.y * diff.y;
    auto rad_sum = other.m_radius + m_radius;
    return square_dist <= rad_sum * rad_sum;
    //return glm::distance(other.m_position, m_position) <= other.m_radius + m_radius; 
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

