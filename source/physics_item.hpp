#pragma once

#include <type_traits>
#include <concepts>

#include <glm/glm.hpp>

#include "colliders.hpp"


namespace Physics {

    using item_id_t = uint64_t;
    item_id_t generate_item_id();
    
    struct PhysicsItem {
        float mass;
        glm::vec2 position;
        glm::vec2 speed;
        item_id_t id = generate_item_id();
    };

    struct PointReference {
        virtual glm::vec2 get() const = 0;
        virtual ~PointReference() = default;
    };

    struct FixedPointReference : public PointReference {
        glm::vec2 get() const override {
            return m_point;
        }

        glm::vec2 m_point;
    };
}

