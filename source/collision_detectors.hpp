#pragma once

#include <algorithm>
#include <atomic>
#include <iterator>
#include <vector>
#include <execution>
#include <thread>

#include "simulation_object.hpp"
#include "colliders.hpp"


namespace Physics {
    using collision_objects_t = std::vector<std::reference_wrapper<const SimulationObject>>;

    struct Collision {
        collision_objects_t objects;
        std::vector<StaticCollider*> statics;
    };

    template<typename T>
    concept CollisionDetector = requires(T detector, const std::vector<SimulationObject>& objects, const std::vector<std::unique_ptr<StaticCollider>>& statics) {
        { detector.detect_collisions(objects, statics) } -> std::same_as<std::vector<Collision>>;
    };

    struct SimpleCollisionDetector {
        template<typename... Args>
        SimpleCollisionDetector(const Args&...) {}

        std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) const {
            std::vector<Collision> result(objects.size());
            for (auto& col : result) {
                col.objects.reserve(5);
                col.statics.reserve(5);
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


    template<uint64_t num_dim_buckets>
    struct BucketCollisionDetector {
        std::vector<float> m_xs;
        std::vector<float> m_ys;
        float m_max_x;
        float m_min_x;
        float m_max_y;
        float m_min_y;
        glm::vec2 m_simulation_rectangle;
        using bucket_t = std::vector<size_t>;
        std::vector<bucket_t> m_buckets;
        inline static constexpr uint64_t s_num_dim_buckets = num_dim_buckets;


        BucketCollisionDetector(glm::vec2 simulation_rectangle) : m_simulation_rectangle(simulation_rectangle) {

        }
        
        void update_bounds(const std::vector<SimulationObject>& objects) {
            /*m_max_x = m_simulation_rectangle.x;
            m_min_x = 0.0f;
            m_max_y = m_simulation_rectangle.y;
            m_min_y = 0.0f;
            return;*/
            m_xs.clear();
            m_ys.clear();
            for (const auto& obj : objects) {
                m_xs.push_back(obj.m_collider.m_position.x);
                m_ys.push_back(obj.m_collider.m_position.y);
            }

            m_max_x = *std::max_element(m_xs.begin(), m_xs.end());
            m_min_x = *std::min_element(m_xs.begin(), m_xs.end());
            m_max_y = *std::max_element(m_ys.begin(), m_ys.end());
            m_min_y = *std::min_element(m_ys.begin(), m_ys.end());
        }

        void update_buckets(const std::vector<SimulationObject>& objects) {
            //TODO: something more intellectual than constant number of uniform buckets?
            m_buckets.resize(num_dim_buckets * num_dim_buckets);
            for (auto& bucket : m_buckets) {
                bucket.clear();
            }

            float bucket_len_x = (m_max_x - m_min_x) / float(num_dim_buckets);
            float bucket_len_y = (m_max_y - m_min_y) / float(num_dim_buckets);
            if (!objects.empty()) {
              auto min_bucket_dim = objects.front().m_collider.m_radius * 2.0f * 1.5f; // 1.5 times diameter
              bucket_len_x = std::max(bucket_len_x, min_bucket_dim);
              bucket_len_y = std::max(bucket_len_y, min_bucket_dim);
            }
            const glm::vec2 bucket_len{bucket_len_x, bucket_len_y};
            
            for (size_t i = 0; i < objects.size(); ++i) {
                const auto& obj = objects[i];
                const glm::vec2 buckets_offset(m_min_x, m_min_y);
                glm::vec2 points_to_check[] = {
                    obj.m_collider.m_position,
                    obj.m_collider.m_position + glm::vec2(0.0f, obj.m_collider.m_radius),
                    obj.m_collider.m_position + glm::vec2(0.0f, -obj.m_collider.m_radius),
                    obj.m_collider.m_position + glm::vec2(obj.m_collider.m_radius, 0.0f),
                    obj.m_collider.m_position + glm::vec2(-obj.m_collider.m_radius, 0.0f)
                };
                for (const auto& point : points_to_check) {
                    glm::vec2 point_bucket_coords = (point - buckets_offset);
                    int bucket_x = std::floor(point_bucket_coords.x / bucket_len.x);
                    int bucket_y = std::floor(point_bucket_coords.y / bucket_len.y);
                    int bucket_idx = bucket_x + bucket_y * num_dim_buckets;
                    if (bucket_x < 0 || bucket_x >= num_dim_buckets) {
                      continue;
                    }
                    if (bucket_y < 0 || bucket_y >= num_dim_buckets) {
                      continue;
                    }
                    if (bucket_idx < 0 || bucket_idx >= m_buckets.size()) {
                      continue;
                    }
                    auto& bucket = m_buckets[bucket_idx];
                    if (!bucket.empty() && bucket.back() == i) {
                        continue;
                    }
                    bucket.push_back(i);
                }
            }
        }

        inline static const auto processor_count = std::thread::hardware_concurrency();
        std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) {
            
            std::vector<Collision> result(objects.size());
            for (auto& col : result) {
                col.objects.reserve(5);
                col.statics.reserve(5);
            }
            
            update_bounds(objects);
            
            update_buckets(objects);

//#define USE_THREADS
#ifdef USE_THREADS
            
            // TODO: try using some king of lock free vector?
            struct Spinlock {
                std::atomic_flag flag{};

                void lock() {
                    while (flag.test_and_set(std::memory_order_acquire)) {
                        while (flag.test(std::memory_order_relaxed)) {}
                    }
                }

                void unlock() {
                    flag.clear(std::memory_order_release);   
                }
            };
            std::vector<Spinlock> locks(objects.size());
#endif
            auto process_buckets = [&](auto start, auto end) {
                for (auto it = start; it != end; ++it) {
                    const auto& bucket = *it;
                    for (size_t i = 0; i < bucket.size(); ++i) {
                        for (size_t j = i + 1; j < bucket.size(); ++j) {
                            auto index = bucket[i];
                            auto other_index = bucket[j];
#ifdef USE_THREADS
                            if (index < other_index) {
                                locks[index].lock();
                                locks[other_index].lock();
                            } else {
                                locks[other_index].lock();
                                locks[index].lock();
                            }
#endif
                            if (objects[index].m_collider.is_colliding(objects[other_index].m_collider)) {
                                result[index].objects.emplace_back(objects[other_index]);
                                result[other_index].objects.emplace_back(objects[index]);
                            }
#ifdef USE_THREADS
                            locks[index].unlock();
                            locks[other_index].unlock();
#endif
                        }
                    }
                }
            };
#ifdef USE_THREADS
            std::vector<std::jthread> threads;
            threads.reserve(processor_count);
            size_t buckets_per_thread = m_buckets.size() / processor_count;
            for (size_t thread_id = 0; thread_id < processor_count; ++thread_id) {
                auto start = m_buckets.cbegin() + buckets_per_thread * thread_id;
                auto end = m_buckets.cbegin() + std::min(buckets_per_thread * (thread_id + 1), m_buckets.size());

                threads.emplace_back(process_buckets, start, end);
            }
            for (auto& thread : threads) {
                thread.join();
            }
#else
            process_buckets(m_buckets.cbegin(), m_buckets.cend());
#endif
            for (size_t i = 0; i < objects.size(); ++i) {
                // Detect statics collisions
                for (const auto& st : statics) {
                    if (st->is_colliding(objects[i].m_collider)) {
                        result[i].statics.push_back(st.get());
                    }   
                }
            }

            //TODO: is this necessary and/or worth it?
            for (auto& res : result) {
                auto ref_to_id = [](const auto& item) { return item.get().m_phys_item.id; };
                std::ranges::sort(res.objects, {}, ref_to_id);
                auto duplicates = std::ranges::unique(res.objects, {}, ref_to_id);
                res.objects.erase(duplicates.begin(), duplicates.end());
            }
            
            return result;
        } 
    };

    struct EmptyCollisionDetector {
        EmptyCollisionDetector(glm::vec2) {}
        std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) {
            
            std::vector<Collision> result(objects.size());
            return result;
        }
    };

    struct OnlyStaticCollisionDetector {
        OnlyStaticCollisionDetector(glm::vec2) {}
        std::vector<Collision> detect_collisions(
                const std::vector<SimulationObject>& objects, 
                const std::vector<std::unique_ptr<StaticCollider>>& statics
        ) {
            
            std::vector<Collision> result(objects.size());
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
}

