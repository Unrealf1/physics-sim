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


std::filesystem::path get_prediction_dir() {
    return std::filesystem::path("./data");
}

std::filesystem::path get_prediction_path(size_t section_idx) {
    return get_prediction_dir()  / (std::to_string(section_idx) + ".txt");
}

int main(int argc, const char** argv) {
    const char* fps_report_file_name = nullptr;
    if (argc > 1) {
      fps_report_file_name = argv[1];
      spdlog::info("Will record fps to \"{}\"", fps_report_file_name);
    }
    WindowParams p {};
    p.w = 1000;
    p.h = 900;
    p.title = "Galton board";
    p.flags = SDL_WINDOW_RESIZABLE;

    Window w(p);
    
    uint32_t sim_width = 200; // centimeters
    uint32_t sim_height = 200;

    Physics::Simulation<Physics::ForwardEuler, Physics::BucketCollisionDetector<50>> sim({sim_width, sim_height});
    sim.add_force(Physics::earth_gravitation());
    //sim.add_force(Physics::damping(0.005f));

    float top_offset = 60.0f;
    float bottom_offset = 40.0f;
    float sides_offset = 10.0f;

    float field_width = float(sim_width) - 2.0f * sides_offset;
    float field_height = float(sim_height) - top_offset - bottom_offset;

    uint32_t num_sections = 20;
    float section_len = field_width / float(num_sections);
    uint32_t num_layers = uint32_t(std::floor(field_height / section_len));
    uint32_t num_knobs = num_sections;
    float knob_radius = 2.5f;

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
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x - dx, y_top}, glm::vec2{x, y_top - 2.0f}));
        sim.add_static_collider(std::make_unique<Physics::StaticSegmentCollider>(glm::vec2{x, y_top - 2.0f}, glm::vec2{x + dx, y_top}));
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
    float circle_radius = 0.6f;
    glm::vec2 box_start = { float(sim_width) / 2.0f - section_len / 2.0f, circle_radius }; // left top
    glm::vec2 box_end = { box_start.x + section_len, top_offset - 2 * knob_radius - circle_radius }; // right bot
    float circle_area = circle_radius * 2.0f * 1.1f;
    float circles_w = std::floor((box_end.x - box_start.x) / (circle_area));
    float circles_h = std::floor((box_end.y - box_start.y) / (circle_area));
    const float circle_mass = circle_radius * circle_radius * std::numbers::pi_v<float> * 0.5f / 10000.0f * 100.0f;
    uint64_t num_circles = 0;
    for (float j = 0.0f; j < circles_h; j += 1.0f) {
        for (float i = - circles_h + j; i < circles_w + circles_h - j; i += 1.0f) {
            ++num_circles;
            auto item = create_sim_object(
                {box_start + glm::vec2{ i * circle_area, j * circle_area }},
                {},
                circle_radius,
                circle_mass
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

    Visualizer v(w);
    v.set_simulation_rectangle(sim.get_simulation_rectangle());
    std::jthread phys_thread(make_physics_thread(&sim, {
        .physics_step = 0.01f,
        .fps_limit = 200.0f,
        .start_delay = 200ms,
        .measure_period = 50,
        .frames_to_run = 5'000,
        .perfomance_report_filename = fps_report_file_name
    }));

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

        if (sim.is_stopped()) {
            quit = true;
        }

        auto frame_objects = sim.get_objects();

//#define DRAW_BUCKET_COLLIDER 1
#ifdef DRAW_BUCKET_COLLIDER
        // TODO: hide behind consexpr branch only for bucket collision detectors
        const auto& collision_detector = sim.get_collision_detector();
        const auto bucket_len = glm::vec2(
            collision_detector.m_max_x - collision_detector.m_min_x,
            collision_detector.m_max_y - collision_detector.m_min_y
        ) / float(collision_detector.s_num_dim_buckets);

        const glm::vec2 bucket_start(collision_detector.m_min_x, collision_detector.m_min_y);
        for (size_t idx = 0; idx < collision_detector.s_num_dim_buckets * collision_detector.s_num_dim_buckets; ++idx) {
          glm::vec2 bucket_start_cur = bucket_start + bucket_len * glm::vec2(idx % collision_detector.s_num_dim_buckets, idx / collision_detector.s_num_dim_buckets);
          glm::vec2 bucket_end = bucket_start_cur + bucket_len;
          size_t bucket_collisions = 0;
          v.draw_rectangle(
              bucket_start_cur + glm::vec2{1.0f, 1.0f},
              bucket_len,
              {0, 255, 255, 255}
          );
        }
        v.draw_rectangle(
            bucket_start,
            glm::vec2{collision_detector.m_max_x, collision_detector.m_max_y} - bucket_start,
            {200, 0, 200, 255}
        );
#endif // drawing bucket collider

        for (const auto& item : frame_objects) {
            size_t section_idx = 0;
            for (; section_idx < section_predictions.size(); ++section_idx) {
                if (section_predictions[section_idx].contains(item.m_phys_item.id)) {
                    break;
                }
            }
            auto color = v.some_color(section_idx);
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
