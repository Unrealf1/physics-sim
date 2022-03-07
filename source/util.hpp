#pragma once

#include <chrono>
#include <stop_token>
#include <limits>

#include "simulation.hpp"


using namespace std::chrono_literals;


struct PhysicsParameters {
    float physics_step = 0.0f;
    float fps_limit = 0.0f;
    uint64_t measure_period = 100;
};

template<typename Sim>
void physics_thread(std::stop_token stoken, Sim* simulation, PhysicsParameters parameters) {
    auto& physics_step = parameters.physics_step;
    auto& fps_limit = parameters.fps_limit;
    auto& measure_period = parameters.measure_period;

    auto frame_duration = std::chrono::milliseconds(
            fps_limit > 0.0f ? uint64_t(1000.0f / fps_limit) : 0
    );
    auto last_frame = std::chrono::steady_clock::now() - 10ms;
    float fps = 0.0f;
    uint64_t frame = 1;
    auto last_measure_time = std::chrono::steady_clock::now();
    auto& sim = *simulation;
    while (!stoken.stop_requested()) {
        auto now = std::chrono::steady_clock::now();
        if (frame % measure_period == 0) {
            float ms = float(std::chrono::duration_cast<std::chrono::milliseconds>(now - last_measure_time).count());
            float local_fps = float(measure_period) * 1000.0f / ms;
            fps = local_fps;
            last_measure_time = now;
            spdlog::info("fps: {}", fps);
        }

        auto since_last_frame = now - last_frame;
        auto dt = physics_step > 0.0f 
            ? physics_step 
            : float(std::chrono::duration_cast<std::chrono::milliseconds>(since_last_frame).count()) / 1000.0f;
        
        sim.step(dt);   

        if (frame_duration > 0ms) {
            auto expected_end_time = now + frame_duration;
            std::this_thread::sleep_until(expected_end_time);
        }
        last_frame = now;
        ++frame;
    }
}

template<typename Sim>
auto make_physics_thread(Sim* simulation, PhysicsParameters parameters) {
    return [=](std::stop_token stoken) {
        return physics_thread(stoken, simulation, parameters);
    };
}

Physics::SimulationObject create_sim_object(glm::vec2 position, glm::vec2 velocity, float radius, float mass, bool is_static = false) {
    Physics::CircleCollider collider = {{ position }, radius};
    Physics::PhysicsItem item = {mass, collider.m_position, velocity, is_static};
    return {collider, item};
}

