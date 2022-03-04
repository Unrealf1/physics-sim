#pragma once

#include "colliders.hpp"
#include "physics_item.hpp"


namespace Physics {
    struct SimulationObject {
        CircleCollider m_collider;
        PhysicsItem m_phys_item;
    };
}

