#pragma once

#include <vector>

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
        PhysicsItem update(float dt, const PhysicsItem& item);
    };
    class SemiImplicitEuler : public BasicIntegrator{
    public:
        PhysicsItem update(float dt, const PhysicsItem& item);
    };

}
