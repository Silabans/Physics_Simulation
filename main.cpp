#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>


int main() {
    using clock = std::chrono::high_resolution_clock;

    const double dt = 0.01;
    auto start = clock::now();

    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;

    bool is_running = true;
    while (is_running) {
        auto current_time = clock::now();
        std::chrono::duration<double> frame_duration = current_time - start;
        start = current_time;

        double frame_time = frame_duration.count();

        if (frame_time > 0.20) {
            frame_time = 0.20;
        }
        accumulator += frame_time;

        while (accumulator > dt) {
            // Insert physics processing

            accumulator -= dt;
        }

    }

    // This part is for rendering

    return 0;
}