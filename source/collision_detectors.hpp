#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <execution>

#include "glm/geometric.hpp"
#include "simulation_object.hpp"


namespace Physics {
    using collision_t = std::vector<std::reference_wrapper<const SimulationObject>>;


    template<typename T>
    concept CollisionDetector = requires(std::vector<SimulationObject>& objects) {
        { T::detect_collisions(objects) } -> std::same_as<std::vector<collision_t>>;
    };

    struct SimpleCollisionDetector {
        static std::vector<collision_t> detect_collisions(const std::vector<SimulationObject>& objects) {
            std::vector<collision_t> result(objects.size());
            //return result;
            //TODO: for each inner vector reserve some constant memory

            for (size_t i = 0; i < objects.size(); ++i) {
                for (size_t j = i + 1; j < objects.size(); ++j) {
                    if (objects[i].m_collider.is_colliding(objects[j].m_collider)) {
                        //spdlog::warn("detected collision of objects: {} and {}", objects[i].m_phys_item.id, objects[j].m_phys_item.id);
                        result[i].emplace_back(objects[j]);
                        result[j].emplace_back(objects[i]);
                    } else {
                        /*spdlog::info("no collision.\npos1 = {},{}; r1 = {}\npos2 = {},{}; r2 = {}\ndistance: {}",
                                objects[i].m_collider.m_position.x, objects[i].m_collider.m_position.y, objects[i].m_collider.m_radius,
                                objects[j].m_collider.m_position.x, objects[j].m_collider.m_position.y, objects[j].m_collider.m_radius,
                                glm::distance(objects[i].m_collider.position(), objects[j].m_collider.position())
                        );*/
                    }
                }
            }

            return result;
        } 
    };
}

