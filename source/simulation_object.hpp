#pragma once

#include "colliders.hpp"
#include "physics_item.hpp"


namespace Physics {
    template<typename collider_t>
    struct SimulationObject {
        collider_t m_collider;
        PhysicsItem m_phys_item;
    };
}

