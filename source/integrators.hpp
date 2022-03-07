#pragma once

#include <iterator>
#include <numeric>
#include <vector>
#include <algorithm>
#include <ranges>
#include <execution>

#include "physics_item.hpp"
#include "forces.hpp"


namespace Physics {

    template<typename T>
    concept Integrator = requires(T integrator, Force f, float dt, PhysicsItem item) {
        { integrator.add_force(f) };
        { integrator.update(dt, item) } -> std::same_as<PhysicsItem>; 
    };

    class BasicIntegrator {
    public:
        void add_force(const Force& f) {
            m_forces.push_back(f);
        }

    protected:
        std::vector<Force> m_forces;

    };

    class ForwardEuler : public BasicIntegrator{
    public:
        PhysicsItem update(float dt, const PhysicsItem& item) {
            //TODO: test for speed (use of execution policies)
            //TODO: rewrite with ranges
            //TODO: respect filtering
            assert(("Mass is sane", item.mass > 0));
            std::vector<glm::vec2> force_vectors;
            force_vectors.reserve(m_forces.size());
            std::transform(
                //std::execution::par_unseq, 
                m_forces.begin(), m_forces.end(),
                std::back_inserter(force_vectors),
                [&item](Force& f) -> glm::vec2 { return f.calculate(item); }
            );

            glm::vec2 total_force = std::reduce(
                std::execution::par_unseq,
                std::cbegin(force_vectors),
                std::cend(force_vectors)
            );
            glm::vec2 acceleration = total_force / item.mass;
            glm::vec2 new_position = item.position + dt * item.speed;
            glm::vec2 new_speed = item.speed + dt * acceleration;

            return {
                item.mass,
                new_position,
                new_speed,
                item.is_static,
                item.id
            };   
        }
    };

}
