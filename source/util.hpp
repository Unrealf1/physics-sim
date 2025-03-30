#pragma once

#include <chrono>
#include <stop_token>
#include <limits>
#include <fstream>

#include "simulation.hpp"


using namespace std::chrono_literals;


struct PhysicsParameters {
    float physics_step = 0.0f;
    float fps_limit = 0.0f;
    std::chrono::milliseconds start_delay = 0ms;
    uint64_t measure_period = 100;
    int64_t frames_to_run = -1;
    const char* perfomance_report_filename = nullptr;
};

template<typename Sim>
void physics_thread(std::stop_token stoken, Sim* simulation, const PhysicsParameters parameters) {
    std::this_thread::sleep_for(parameters.start_delay);
    auto& physics_step = parameters.physics_step;
    auto& fps_limit = parameters.fps_limit;
    auto& measure_period = parameters.measure_period;

    auto frame_duration = std::chrono::milliseconds(
            fps_limit > 0.0f ? uint64_t(1000.0f / fps_limit) : 0
    );
    const auto simulation_start = std::chrono::steady_clock::now();
    auto last_frame = simulation_start - 10ms;
    float fps = 0.0f;
    uint64_t frame = 1;
    auto last_measure_time = std::chrono::steady_clock::now();
    auto& sim = *simulation;

    struct PerfomanceItem{
        uint64_t frame_num;
        float fps;
    };
    std::vector<PerfomanceItem> perf_history;

    const bool keep_perfomance_report = parameters.perfomance_report_filename != nullptr;
    if (keep_perfomance_report) {
        perf_history.reserve(200);
    }

    while (!stoken.stop_requested()) {
        auto now = std::chrono::steady_clock::now();
        if (frame % measure_period == 0) {
            float ms = float(std::chrono::duration_cast<std::chrono::milliseconds>(now - last_measure_time).count());
            float local_fps = float(measure_period) * 1000.0f / ms;
            fps = local_fps;
            last_measure_time = now;
            if (keep_perfomance_report) {
                perf_history.push_back({frame, fps});
            }
            spdlog::info("[{}] fps: {:.2f}; time scale: {:.2f}", frame, fps, physics_step > 0.0f ? physics_step / (1.0f / fps) : 1.0f);
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
        if (int64_t(frame) == parameters.frames_to_run) {
            break;
        }
        ++frame;
    }
    sim.stop();
    if (keep_perfomance_report) {
        std::ofstream perf_out(parameters.perfomance_report_filename);
        for (auto item : perf_history) {
            perf_out << item.frame_num << '\t' << item.fps << '\n';
        }
    }
    auto finished_at = std::chrono::steady_clock::now();
    float ms = float(std::chrono::duration_cast<std::chrono::milliseconds>(finished_at - simulation_start).count());
    spdlog::info("Finished simulation in: {:.2f}ms", ms);
}

template<typename Sim>
auto make_physics_thread(Sim* simulation, PhysicsParameters parameters) {
    return [=](std::stop_token stoken) {
        return physics_thread(stoken, simulation, parameters);
    };
}

inline Physics::SimulationObject create_sim_object(glm::vec2 position, glm::vec2 velocity, float radius, float mass) {
    Physics::CircleCollider collider = {{ position }, radius};
    Physics::PhysicsItem item = {mass, collider.m_position, velocity};
    return {collider, item};
}

