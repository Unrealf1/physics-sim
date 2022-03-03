#pragma once

#include <tuple>
#include <vector>

#include "physics_item.hpp"


namespace Physics {

    template<typename Integrator, typename... Classes>
    class PhysicsEngine {
    public:
        void update();
        
        template<typename Class>
        const std::vector<Class>& get_items() const {
            return std::get<std::vector<Class>>(m_items);
        }

    private:
        Integrator m_integrator;
        std::tuple<std::vector<Classes>...> m_items;
    };

}


