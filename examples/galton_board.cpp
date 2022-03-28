#include <thread>
#include <chrono>
#include <iterator>
#include <memory>
#include <ranges>
#include <stop_token>
#include <string>
#include <thread>
#include <functional>
#include <limits>
#include <fstream>
#include <unordered_set>
#include <algorithm>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <SDL.h>

#include "colliders.hpp"
#include "collision_detectors.hpp"
#include "forces.hpp"
#include "integrators.hpp"
#include "physics_item.hpp"
#include "window.hpp"
#include "simulation.hpp"
#include "visualizer.hpp"
#include "util.hpp"

using namespace std::chrono_literals;
using sim_obj_t = Physics::SimulationObject<Physics::CircleCollider>;


std::filesystem::path get_prediction_dir() {
    return std::filesystem::path("./data");
}

std::filesystem::path get_prediction_path(size_t section_idx) {
    return get_prediction_dir()  / (std::to_string(section_idx) + ".txt");
}

int main(int, char *[]) {
    std::this_thread::sleep_for(500ms);
    WindowParams p {};
    p.w = 1000;
    p.h = 900;
    p.title = "Galton board";
    p.flags = SDL_WINDOW_RESIZABLE;

    Window w(p);
    
    uint32_t sim_width = 1000;
    uint32_t sim_height = 900;

    Physics::Simulation<sim_obj_t, Physics::ForwardEuler, Physics::BucketCollisionDetector<sim_obj_t, 9>> sim({sim_width, sim_height});
    //Physics::Simulation<Physics::ForwardEuler, Physics::OnlyStaticCollisionDetector> sim({sim_width, sim_height});
    sim.add_force(Physics::Forces::earth_gravitation());
    sim.add_force(Physics::Forces::damping(0.17f));

    float top_offset = 300.0f;
    float bottom_offset = 200.0f;
    float sides_offset = 50.0f;

    float field_width = float(sim_width) - 2.0f * sides_offset;
    float field_height = float(sim_height) - top_offset - bottom_offset;

    uint32_t num_sections = 10;
    float section_len = field_width / float(num_sections);
    uint32_t num_layers = uint32_t(std::ceil(field_height / section_len));
    uint32_t num_knobs = num_sections;
    float knob_radius = 10.0f;

    std::vector<std::unordered_set<Physics::item_id_t>> section_predictions(num_sections + 2);
    for (uint32_t section = 0; section < section_predictions.size(); ++section) {
        auto prediction_path = get_prediction_path(section);
        if (!std::filesystem::exists(prediction_path)) {
            spdlog::warn("can't find prediction for section {}. Skipping (it will be generated after this run)", section);
            continue;
        }
        std::ifstream ifs(prediction_path);
        auto iter = std::istream_iterator<Physics::item_id_t>(ifs);
        std::copy(
            iter, std::istream_iterator<Physics::item_id_t>{},
            std::inserter(section_predictions[section], section_predictions[section].begin())
        );
    }

    // Add separators
    for (uint32_t section = 0; section < num_sections + 1; ++section) {
        float x = sides_offset + float(section) * section_len;
        float y_top = float(sim_height) - bottom_offset;
        float y_bot = float(sim_height);
        float dx = section_len / 20.0f;
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x - dx, y_bot}, glm::vec2{x - dx, y_top}));
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x + dx, y_bot}, glm::vec2{x + dx, y_top}));
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x - dx, y_top}, glm::vec2{x + dx, y_top}));
    }

    // Add immovable circles
    for (uint32_t layer = 0; layer < num_layers; ++layer) {
        bool even_layer = layer % 2 == 0;
        uint32_t current_nobs = even_layer ? num_knobs - 1 : num_knobs;
        float top_start = top_offset + float(layer) * section_len;
        float left_start = sides_offset + section_len / 2.0f + even_layer * section_len / 2.0f;

        for (float knob = 0; knob < float(current_nobs); knob += 1.0f) {
            sim.add_static_collider(std::make_unique<Physics::StaticCircleCollider>(
                glm::vec2{left_start + knob * section_len, top_start}, knob_radius)
            );
        }
    }

    // Add falling items
    float circle_radius = knob_radius / 5.0f;
    glm::vec2 box_start = { float(sim_width) / 2.0f - section_len / 2.0f, circle_radius }; // left top
    glm::vec2 box_end = { box_start.x + section_len, top_offset - 2 * knob_radius - circle_radius }; // right bot
    float circle_area = circle_radius * 2.0f * 1.1f;
    float circles_w = std::floor((box_end.x - box_start.x) / (circle_area));
    float circles_h = std::floor((box_end.y - box_start.y) / (circle_area));
    uint64_t num_circles = 0;
    for (float j = 0.0f; j < circles_h; j += 1.0f) {
        for (float i = - circles_h + j; i < circles_w + circles_h - j; i += 1.0f) {
            ++num_circles;
            auto item = create_sim_object(
                {box_start + glm::vec2{ i * circle_area, j * circle_area }},
                {},
                circle_radius,
                1.0f
            );
            sim.add_circle(item);
        }
    }
    spdlog::info("launching simulaiton with {} circles", num_circles);

    // Add slides
    sim.add_static_collider(
        std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{0.0f, 0.0f}, glm::vec2{box_start.x, box_end.y})
    );
    sim.add_static_collider(
        std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{sim.get_simulation_rectangle().x, 0.0f}, box_end)
    );

    auto ref1 = sim.get_object_ref(100);
    auto ref2 = sim.get_object_ref(5000);

    auto& p1 = sim.get_point_ref(ref1);
    auto& p2 = sim.get_point_ref(ref2);

    auto spring = Physics::Forces::spring(5, 100, ref1.get().m_phys_item.id, ref2.get().m_phys_item.id, p1, p2);
    sim.add_force(spring);

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, { .physics_step = 0.005f, .fps_limit = 0.0f, .start_delay = 200ms, .measure_period = 50 } ));

    bool quit = false;
    SDL_Event event;
    while(!quit) {
        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    quit = true;
                    break;
            }
        }
        auto frame_objects = sim.get_objects();
        v.draw_line(p1.get(), p2.get(), {255, 0, 0, 255});
        for (const auto& item : frame_objects) {
            size_t section_idx = 0;
            for (; section_idx < section_predictions.size(); ++section_idx) {
                if (section_predictions[section_idx].contains(item.m_phys_item.id)) {
                    break;
                }
            }
            auto color = v.some_color(section_idx);
            if (item.m_phys_item.id == ref1.get().m_phys_item.id || item.m_phys_item.id == ref2.get().m_phys_item.id) {
                color = {255, 255, 255, 255};
            }
            v.draw_circle(item.m_phys_item.position, item.m_collider.m_radius, color);
            //v.draw_line(item.m_phys_item.position, item.m_phys_item.position+item.m_phys_item.speed * 0.1f * item.m_phys_item.mass, {200, 200, 0, 255});
        }
        v.draw_rectangle({0.0f, 0.0f}, sim.get_simulation_rectangle(), {128, 128, 0, 255});
        const auto& statics = sim.get_static_colliders();
        for (const auto& st_p : statics) {
            auto& st = *st_p;
            st.visualize(v); 
        }

        glm::vec2 total_impulse(0.0f, 0.0f);
        for (const auto& obj : frame_objects) {
            total_impulse += obj.m_phys_item.mass * obj.m_phys_item.speed;
        }
        glm::vec2 impulse_start{1200.0f, 300.0f};
        v.draw_line(impulse_start, impulse_start + total_impulse * 0.0001f, {255, 255, 0, 255});

        v.finish_frame();
        std::this_thread::sleep_for(10ms);
    }
    phys_thread.request_stop();

    auto frame_objects = sim.get_objects();
    float last_limit = 0.0f;

    if (!std::filesystem::exists(get_prediction_dir())) {
        std::filesystem::create_directory(get_prediction_dir());
    }

    for (size_t section_idx = 0; section_idx < num_sections + 2; ++section_idx) {
        float limit = sides_offset + float(section_idx) * section_len;
        auto prediction_path = get_prediction_path(section_idx);
        std::ofstream ofs(prediction_path);
        auto iter = std::ostream_iterator<std::string>(ofs, "\n");
        auto thing = std::views::all(frame_objects)
            | std::views::filter([&](const auto& obj) { return (obj.m_phys_item.position.x <= limit) & (obj.m_phys_item.position.x > last_limit); })
            | std::views::transform([](const auto& obj) { return std::to_string(obj.m_phys_item.id); });
        std::copy(thing.begin(), thing.end(), iter);
        last_limit = limit;
    }
    return 0;
}
