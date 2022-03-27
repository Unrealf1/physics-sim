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
            //TODO: test for speed (use of execution policies or threads)
            assert(("Mass is sane", item.mass > 0));
            
            auto is_force_affecting = [&item](Force& force){ return force.is_affecting(item); };
            auto force_to_vec = [&item](Force& force) -> glm::vec2 { return force.calculate(item); };
            auto vecs = m_forces 
                | std::views::filter(is_force_affecting) 
                | std::views::transform(force_to_vec);
            glm::vec2 total_force = std::reduce(vecs.begin(), vecs.end());
            
            glm::vec2 acceleration = total_force / item.mass;
            glm::vec2 new_position = item.position + dt * item.speed;
            glm::vec2 new_speed = item.speed + dt * acceleration;

            return {
                item.mass,
                new_position,
                new_speed,
                item.id
            };   
        }
    };

}
