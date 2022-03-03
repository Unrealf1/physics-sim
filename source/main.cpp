#include <iostream>
#include <thread>
#include <chrono>

#include <spdlog/spdlog.h>

#include "window.hpp"


int main(int argc, char** argv) {
    spdlog::info("Hello, world!");

    engine::Window w({});
    w.SetClearColor(1, 0, 0, 0);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5s);
}
