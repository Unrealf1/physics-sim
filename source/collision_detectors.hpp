#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <execution>

#include "simulation_object.hpp"
#include "colliders.hpp"


namespace Physics {
    using collision_objects_t = std::vector<std::reference_wrapper<const SimulationObject>>;

    struct Collision {
        collision_objects_t objects;
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
            for (auto& col : result) {
                col.objects.reserve(objects.size() / 100);
                col.statics.reserve(statics.size() / 50);
            }

            for (size_t i = 0; i < objects.size(); ++i) {
                // Detect object collisions
                for (size_t j = i + 1; j < objects.size(); ++j) {
                    if (objects[i].m_collider.is_colliding(objects[j].m_collider)) {
                        result[i].objects.emplace_back(objects[j]);
                        result[j].objects.emplace_back(objects[i]);
                    } 
                }
            }

            for (size_t i = 0; i < objects.size(); ++i) {
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

    
    struct BucketCollisionDetector {
        static std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) {
            std::vector<Collision> result(objects.size());
            for (auto& col : result) {
                col.objects.reserve(objects.size() / 100);
                col.statics.reserve(statics.size() / 50);
            }
            
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
            //spdlog::info("max_y = {}; min_y = {};", max_y, min_y);
            //TODO: something more intellectual?
            size_t num_dim_buckets = 10;
            float bucket_len = std::max(max_x - min_x, max_y - min_y) / float(num_dim_buckets);
            using bucket_t = std::vector<size_t>;
            //TODO: reserve memory for each bucket
            std::vector<bucket_t> buckets(num_dim_buckets * num_dim_buckets);

            auto get_bucket_start = [&](size_t bucket_idx) -> glm::vec2 {
                auto col_num = bucket_idx % num_dim_buckets;
                auto line_num = bucket_idx / num_dim_buckets;

                return {float(col_num) * bucket_len, float(line_num) * bucket_len};
            };
            
            for (size_t i = 0; i < objects.size(); ++i) {
                auto& obj = objects[i];
                for (size_t idx = 0; idx < num_dim_buckets * num_dim_buckets; ++idx) {
                    //TODO: keep bucket dimentions in loop, do not recalculate labda for it
                    
                    // TODO: for now assume, that bucket is not smaller than circle. may be bad.
                    auto bucket_start = get_bucket_start(idx);
                    glm::vec2 points_to_check[] = {
                        obj.m_collider.m_position, 
                        obj.m_collider.m_position + glm::vec2(0.0f, obj.m_collider.m_radius),
                        obj.m_collider.m_position + glm::vec2(0.0f, -obj.m_collider.m_radius),
                        obj.m_collider.m_position + glm::vec2(obj.m_collider.m_radius, 0.0f),
                        obj.m_collider.m_position + glm::vec2(-obj.m_collider.m_radius, 0.0f)
                    };
                    auto check_bucket_collision = [&](const glm::vec2& point) -> bool {
                        //TODO: test && vs &
                        return (point.x <= bucket_start.x + bucket_len)
                                & (point.x >= bucket_start.x)
                                & (point.y <= bucket_start.y + bucket_len)
                                & (point.y >= bucket_start.y);
                    };
                    bool bucket_collision = std::any_of(std::begin(points_to_check), std::end(points_to_check), check_bucket_collision);
                    if (bucket_collision) {
                        buckets[idx].push_back(i);
                    }
                }
            }

            for (const auto& bucket : buckets) {
                for (size_t i = 0; i < bucket.size(); ++i) {
                    for (size_t j = i + 1; j < bucket.size(); ++j) {
                        auto index = bucket[i];
                        auto other_index = bucket[j];
                        if (objects[index].m_collider.is_colliding(objects[other_index].m_collider)) {
                            result[index].objects.emplace_back(objects[j]);
                            result[other_index].objects.emplace_back(objects[i]);
                        } 
                    }
                }
            }

            for (size_t i = 0; i < objects.size(); ++i) {
                // Detect statics collisions
                for (const auto& st : statics) {
                    if (st->is_colliding(objects[i].m_collider)) {
                        result[i].statics.push_back(st.get());
                    }   
                }
            }
            //TODO: remove possible duplicates
            /*
            for (auto& res : result) {
                //TODO: properly use ranges and unique
                std::ranges::sort(res.objects, [](const SimulationObject& obj1, const SimulationObject& obj2) {
                        return obj1.m_phys_item.id < obj2.m_phys_item.id;
                });
                

            }*/
            return result;
        } 
    };

    struct EmptyCollisionDetector {
        static std::vector<Collision> detect_collisions(const std::vector<SimulationObject>& objects) {
            std::vector<Collision> result(objects.size());
            return result;
        } 
    };
}

