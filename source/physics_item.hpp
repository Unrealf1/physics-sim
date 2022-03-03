#pragma once

#include <type_traits>
#include <concepts>
#include <glm/glm.hpp>


namespace Physics {

    template<typename T>
    concept Integrator = true;
    
    template<typename T>
    concept Movable = requires(T t, glm::vec2 point) {
        { t.move_to(point) };
        { t.move_by(point) };
        { t.position() } -> std::same_as<glm::vec2>;
    };

    template<typename T, typename I>
    concept PhysicsClass = 
    Movable<T> &&
    requires(T a, float dt, I integrator) {
        { a.update(dt, integrator) } -> std::same_as<T>; 
    };


    template<typename T>
    concept Collider = 
    Movable<T> &&
    requires (T collider, glm::vec2 point) {
        { collider.is_colliding(point) } -> std::same_as<bool>;
        { collider.is_colliding(collider) } -> std::same_as<bool>;
    };
    
    template<Collider collider_t>
    struct SimpleItem {
        float mass;
        collider_t collider;

        glm::vec2 speed;

    public:
        template<Integrator integrator_t>
        SimpleItem update(float dt, integrator_t& integrator) {
            //auto [new_position, new_speed] = integrator.update();
        }
    };

}


