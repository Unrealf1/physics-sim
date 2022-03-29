#include "integrators.hpp"

#include <numeric>
#include <algorithm>
#include <ranges>
#include <execution>

using std::views::filter;
using std::views::transform;

namespace Physics {

    // Two crossed vectors return a scalar
    float CrossProduct( const glm::vec2& a, const glm::vec2& b ) {
      return a.x * b.y - a.y * b.x;
    }

    // More exotic (but necessary) forms of the cross product
    // with a vector a and scalar s, both returning a vector
    glm::vec2 CrossProduct( const glm::vec2& a, float s ) {
      return glm::vec2( s * a.y, -s * a.x );
    }

    glm::vec2 CrossProduct( float s, const glm::vec2& a ) {
      return glm::vec2( -s * a.y, s * a.x );
    }

    PhysicsItem ForwardEuler::update(float dt, const PhysicsItem& item) {
        //TODO: test for speed (use of execution policies or threads)
        assert(("Mass is sane", item.mass > 0));
        
        auto is_force_affecting = [&item](Force& force){ return force.is_affecting(item); };
        auto force_to_vec = [&item](Force& force) -> glm::vec2 { return force.calculate(item); };
        auto force_to_torque = [&item](Force& force) -> float { return force.calculate_torque(item); };
        //TODO: optimize(no need to repeat filtering)
        auto vecs = m_forces 
            | filter(is_force_affecting) 
            | transform(force_to_vec);
        auto torqs = m_forces 
            | filter(is_force_affecting) 
            | transform(force_to_torque);
        glm::vec2 total_force = std::reduce(vecs.begin(), vecs.end());
        float total_torque = std::reduce(torqs.begin(), torqs.end());
        
        glm::vec2 acceleration = total_force / item.mass;
        glm::vec2 new_position = item.position + dt * item.speed;
        glm::vec2 new_speed = item.speed + dt * acceleration;
        float rotational_acceleration = total_torque / item.moment_of_inertia;
        float new_rotation_speed = item.rotation_speed + dt * rotational_acceleration;
        float new_orientation = item.orientation + dt * item.rotation_speed;

        return {
            item.mass,
            new_position,
            new_speed,
            item.moment_of_inertia,
            new_orientation,
            new_rotation_speed,
            item.id
        };   
    }
}
