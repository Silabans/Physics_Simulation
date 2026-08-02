#include "raylib.h"
#include <vector>
#include <algorithm>
#include <thread>
#include <random>
#include "forces.hpp"
#include "rigidbody.hpp"


// reusable random device seeding
std::mt19937& random_engine() {
    // initialised once in the first function call (static var)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}

int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "2D Physics Engine");
    SetTargetFPS(60);


    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;
    const double dt = 0.01;

    // body initialisation using unique pointers (to store the objects in the heap rather
    // than the stack => more spacious)
    std::vector<std::unique_ptr<RigidBody>> bodies; 

    // create 10 randomly generated balls
    Color colors[7] = {RED, BLUE, YELLOW, GREEN, WHITE, PURPLE, ORANGE};
    for (int i = 0; i < 30; ++i) {
        std::uniform_int_distribution<std::size_t> distrib(0, 6);
        Color color = colors[distrib(random_engine())];

        float radius = 12.0f + static_cast<float>(rand() % 16); // radius from 15 to 45 pixels
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);
        float mass = radius*radius*radius * 0.002f; // mass proportional to size

        auto circle = std::make_unique<Circle>(radius);
        // make a unique pointer
        bodies.push_back(std::make_unique<RigidBody>(x, y, mass, std::move(circle), color));
    }

    while (!WindowShouldClose()) { // runs while window is open
        double frame_time = GetFrameTime(); // delta time in seconds
        if (frame_time > 0.20) {
            frame_time = 0.20;
        }
        accumulator += frame_time;

        int impulseIterations = 4;

        while (accumulator > dt) {
            // resolve 4 times to account for multibody collisions (more than 2 bodies colliding)
            for (int i = 0; i < impulseIterations; ++i) {
                resolve_all_collisions(bodies);
            }

            // updating velocity and position of each body
            for (auto& body : bodies) { // (*) -> pass in the pointer (not by value or reference)
                // pointer used because if passed by value, 
                // a temp copy will be made and destroyed after the the function completes (local only)
                update_noncollision(*body);
                body->calculate_velocity(dt);
                body->integrate_pos(dt);
                body->reset_forces(); // reset forces after integrating (to avoid the same force accumulating)

                resolve_boundaries(*body);
            }

            accumulator -= dt;
        }

        // This part is for rendering
        BeginDrawing();
            ClearBackground(BLACK);

            for (const auto& body : bodies) {
                const Shape* shape = body->getShape(); // Shape, not ShapeType

                if (shape->type == ShapeType::CIRCLE) {
                    const Circle* circle = static_cast<const Circle*>(shape); // convert shape to circle
                    DrawCircle(
                        static_cast<int>(body->getPositionX()),
                        static_cast<int>(body->getPositionY()),
                        static_cast<int>(circle->radius),
                        body->getColor()
                    );
                }   
            }

            DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}