#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include "forces.hpp"

int main() {
    using clock = std::chrono::high_resolution_clock;

    const double dt = 0.01;
    auto start = clock::now();

    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;

    // body initialisation
    RigidBody bodyA = RigidBody{40.0, 10.0, 2.0, ShapeType::CIRCLE};
    RigidBody bodyB = RigidBody{180.0, 70.0, 1.5, ShapeType::CIRCLE};

    std::vector<RigidBody> allBodies = {bodyA, bodyB};

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
            update_contact(bodyA, bodyB);

            // updating velocity and position of each body
            for (RigidBody body : allBodies) {
                update_noncontact(body);
                body.calculate_velocity(dt);
                body.integrate_pos(dt);
            }
        }

    }

    // This part is for rendering

    return 0;
}