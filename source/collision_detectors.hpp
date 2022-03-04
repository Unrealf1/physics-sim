#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <execution>

#include "simulation_object.hpp"


namespace Physics {
    using collision_t = std::vector<std::reference_wrapper<SimulationObject>>;


    template<typename T>
    concept CollisionDetector = requires(std::vector<SimulationObject>& objects) {
        { T::detect_collisions(objects) } -> std::same_as<std::vector<collision_t>>;
    };

    struct SimpleCollisionDetector {
        static std::vector<collision_t> detect_collisions(std::vector<SimulationObject>& objects) {
            std::vector<collision_t> result(objects.size());
            //TODO: for each inner vector reserve some constant memory

            for (size_t i = 0; i < objects.size(); ++i) {
                for (size_t j = i + 1; j < objects.size(); ++j) {
                    if (objects[i].m_collider.is_colliding(objects[j].m_collider)) {
                        result[i].emplace_back(objects[j]);
                        result[j].emplace_back(objects[i]);
                    }
                }
            }

            return result;
        } 
    };
}

