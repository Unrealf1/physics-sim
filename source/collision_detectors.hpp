#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <execution>

#include "glm/geometric.hpp"
#include "simulation_object.hpp"


namespace Physics {
    using collision_t = std::vector<std::reference_wrapper<const SimulationObject>>;
    struct Link {
        size_t first_idx;
        size_t second_idx;
    };
    using links_t = std::vector<Link>;

    template<typename T>
    concept CollisionDetector = requires(std::vector<SimulationObject>& objects, links_t links) {
        { T::detect_collisions(objects, links) } -> std::same_as<std::vector<collision_t>>;
    };

    struct SimpleCollisionDetector {
        static std::vector<collision_t> detect_collisions(const std::vector<SimulationObject>& objects, const links_t& links) {
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
            for (const auto& link : links) {
                result[link.first_idx].emplace_back(objects[link.second_idx]);
                result[link.second_idx].emplace_back(objects[link.first_idx]);
            }

            return result;
        } 
    };
    struct EmptyCollisionDetector {
        static std::vector<collision_t> detect_collisions(const std::vector<SimulationObject>& objects, const links_t&) {
            std::vector<collision_t> result(objects.size());
            return result;
        } 
    };
}

