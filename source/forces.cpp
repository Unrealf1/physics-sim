#include "forces.hpp"


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
        auto calc = [k, initial_length, first, second, &first_point, &second_point](const PhysicsItem& item) {
            auto point1 = first_point.get();
            auto point2 = second_point.get();
            auto diff = point1 - point2;
            float length = glm::length(diff);
            float power = -k * (length - initial_length);
            if (item.id == second) {
                power *= -1;
            }
            return glm::normalize(diff) * power;                
        };
        
        //TODO: optimize/rework forces entirely
        auto torque = [calc, &first_point, &second_point, first, second](const PhysicsItem& item) {
            auto cross = [](const auto& vec1, const auto& vec2) -> float {
                return (vec1.x * vec2.y) - (vec1.y * vec2.x);
            };
            auto force = calc(item);
            glm::vec2 at;
            if (item.id == second) {
                at = second_point.get();
            } else {
                at = first_point.get();
            }

            return cross(at - item.position, force);
        };

        auto filt = [=](const PhysicsItem& item) {
            return item.id == first || item.id == second;
        };

        return { calc, torque, filt };
    }


}}
