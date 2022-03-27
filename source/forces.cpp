#include "forces.hpp"

#include <spdlog/spdlog.h>


namespace Physics { namespace Forces {
    //TODO: attach spring to an arbitrary point in the item. (need Point ref class...)
    //TODO: spring that is tence at the start of the simulation
    Force spring(
            float k, 
            float initial_length, 
            const Physics::item_id_t& first, 
            const Physics::item_id_t& second,
            const PointReference& first_point,
            const PointReference& second_point
    ) {
        spdlog::info("creating spring between objects {} and {}", first, second);
        auto calc = [k, initial_length, &first, &second, &first_point, &second_point](const PhysicsItem& item) {
            auto point1 = first_point.get();
            auto point2 = second_point.get();
            auto diff = point1 - point2;
            float length = glm::length(diff);
            float power = -k * (length - initial_length);
            if (item.id == second) {
                power *= -1;
            }
            spdlog::info("power: {}; direction: {},{}", power, diff.x, diff.y);
            return glm::normalize(diff) * power;                
        };
        auto filt = [=](const PhysicsItem& item) {
            return item.id == first || item.id == second;
        };

        return { calc, filt };
    }


}}
