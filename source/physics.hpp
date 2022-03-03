#pragma once

#include <iterator>
#include <pstl/glue_execution_defs.h>
#include <tuple>
#include <utility>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <execution>

#include "physics_item.hpp"


namespace Physics {

    template<Integrator Integrator_t, PhysicsClass<Integrator_t>... Classes>
    class PhysicsEngine {
    public:
        template<typename T>
        using class_buffer_t = std::vector<T>;

    public:
        void update(float dt) {
            update_helper<0>(dt);
            switch_buffers();
        }
        
        template<typename Class>
        const class_buffer_t<Class>& get_items() const {
            return std::get<class_buffer_t<Class>>(get_last_buffer());
        }

        template<typename Class>
        void add_item(const Class& item) {
            auto& buffer = std::get<class_buffer_t<Class>>(get_last_buffer());
            buffer.push_back(item);
        }

    private:
        Integrator_t m_integrator;

        using buffers_t = std::tuple<class_buffer_t<Classes>...>;
        buffers_t m_buffers_holder[2];
        uint8_t m_active_index = 0;

        buffers_t& get_active_buffer() {
            return m_buffers_holder[m_active_index];
        }

        buffers_t& get_last_buffer() {
            return m_buffers_holder[1 - m_active_index];
        }

        void switch_buffers() {
            m_active_index = 1 - m_active_index;
        }
        
        template<size_t index>
        void update_helper(float dt) {
            if constexpr (index >= std::tuple_size_v<buffers_t>) {
                return;
            }

            auto& active_buffers = get_active_buffer();
            const auto& last_buffers = get_last_buffer();

            auto& current_active_buffer = std::get<index>(active_buffers);
            const auto& current_last_buffer = std::get<index>(last_buffers);
            current_active_buffer.clear();
            
            std::transform(
                    std::execution::par_unseq,
                    std::begin(current_last_buffer),
                    std::end(current_last_buffer),
                    std::back_inserter(current_active_buffer),
                    [dt, this](const auto& item) { return item.update(dt, m_integrator); }
            );

            update_helper<index + 1>(dt);
        }

    };

}

