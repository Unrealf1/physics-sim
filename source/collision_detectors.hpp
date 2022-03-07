#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <execution>

#include "simulation_object.hpp"
#include "colliders.hpp"


namespace Physics {
    struct Collision {
        std::vector<std::reference_wrapper<const SimulationObject>> objects;
        std::vector<StaticCollider*> statics;
    };

    template<typename T>
    concept CollisionDetector = requires(const std::vector<SimulationObject>& objects, const std::vector<std::unique_ptr<StaticCollider>>& statics) {
        { T::detect_collisions(objects, statics) } -> std::same_as<std::vector<Collision>>;
    };

    struct SimpleCollisionDetector {
        static std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) {
            std::vector<Collision> result(objects.size());
            //TODO: for each inner vector reserve some constant memory

            for (size_t i = 0; i < objects.size(); ++i) {
                // Detect object collisions
                for (size_t j = i + 1; j < objects.size(); ++j) {
                    if (objects[i].m_collider.is_colliding(objects[j].m_collider)) {
                        result[i].objects.emplace_back(objects[j]);
                        result[j].objects.emplace_back(objects[i]);
                    } 
                }
                // Detect statics collisions
                for (const auto& st : statics) {
                    if (st->is_colliding(objects[i].m_collider)) {
                        result[i].statics.push_back(st.get());
                    }   
                }

            }

            return result;
        } 
    };

    /*
    struct BucketCollisionDetector {
        static std::vector<collision_t> detect_collisions(const std::vector<SimulationObject>& objects) {
            std::vector<collision_t> result(objects.size());
            //TODO: for each inner vector reserve some constant memory
            std::vector<float> xs;
            xs.reserve(objects.size());
            std::vector<float> ys;
            ys.reserve(objects.size());

            for (const auto& obj : objects) {
                xs.push_back(obj.m_collider.m_position.x);
                ys.push_back(obj.m_collider.m_position.y);
            }

            auto max_x = *std::max_element(xs.begin(), xs.end());
            auto min_x = *std::min_element(xs.begin(), xs.end());
            auto max_y = *std::max_element(ys.begin(), ys.end());
            auto min_y = *std::min_element(ys.begin(), ys.end());
            spdlog::info("max_y = {}; min_y = {};", max_y, min_y);
            //TODO: something more intellectual?
            size_t num_dim_buckets = 10;
            using bucket_t = std::vector<const SimulationObject*>;
            std::vector<bucket_t> buckets(num_dim_buckets * num_dim_buckets);
            auto get_bucket_idx = [&](const auto& obj) {
                auto x = obj.m_collider.m_position.x;
                auto x_bucket_idx = size_t(std::floor((x - min_x) / (max_x - min_x + 0.001f) * float(num_dim_buckets)));
                auto y = obj.m_collider.m_position.y;
                auto y_bucket_idx = size_t(std::floor((y - min_y) / (max_y - min_y + 0.001f) * float(num_dim_buckets)));
                auto idx = x_bucket_idx + y_bucket_idx * num_dim_buckets;
                if (idx >= buckets.size()) {
                    spdlog::error("size: {}, idx: {}. x_idx: {}; y_idx: {}; y = {}\n{}", 
                            buckets.size(), 
                            idx, 
                            x_bucket_idx, 
                            y_bucket_idx, 
                            y,
                            (y - min_y) / (max_y - min_y + 0.001f)
                   );
                }
                return idx;
            };

            for (const auto& obj : objects) {
                auto idx = get_bucket_idx(obj);
                buckets[idx].push_back(&obj);
            }

            auto check_bucket = [&](size_t b_idx, const SimulationObject& obj, size_t obj_idx) {
                spdlog::warn("sz: {}, idx: {}", buckets.size(), b_idx);
                for (size_t i = 0; i < buckets[b_idx].size(); ++i) {
                    const auto& b_obj = *buckets[b_idx][i];
                    if (obj.m_collider.is_colliding(b_obj.m_collider) && obj.m_phys_item.id != b_obj.m_phys_item.id) {
                        result[obj_idx].emplace_back(b_obj);
                    } 
                }
            };

            for (size_t i = 0; i < objects.size(); ++i) {
                const auto& obj = objects[i];
                auto idx = get_bucket_idx(obj);
                check_bucket(idx, obj, i);
                auto up = idx - num_dim_buckets;
                auto down = idx + num_dim_buckets;
                auto right = idx + 1;
                auto left = idx - 1;
                if (up < idx) {
                    check_bucket(up, obj, i);
                }
                if (down < buckets.size()) {
                    check_bucket(down, obj, i);
                }
                if (right % num_dim_buckets != 0) {
                    check_bucket(right, obj, i);
                }
                if (left < idx & left % num_dim_buckets != num_dim_buckets - 1) {
                    check_bucket(left, obj, i);
                }
            }

            return result;
        } 
    };*/

    struct EmptyCollisionDetector {
        static std::vector<Collision> detect_collisions(const std::vector<SimulationObject>& objects) {
            std::vector<Collision> result(objects.size());
            return result;
        } 
    };
}

