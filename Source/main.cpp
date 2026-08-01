#include "raylib.h"
#include <vector>
#include <algorithm>
#include <thread>
#include "forces.hpp"
#include "rigidbody.hpp"

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

    // shapes initialisation using unique pointers
    auto circleA = std::make_unique<Circle>(30.0f);
    auto circleB = std::make_unique<Circle>(20.0f);

    // body initialisation
    RigidBody bodyA(500.0f, 550.0f, 2.0f, std::move(circleA)); // std::move() is used to transfer ownership since unique_ptr cannot be copied
    RigidBody bodyB(500.0f, 100.0f, 1.5f, std::move(circleB));

    std::vector<RigidBody*> allBodies = {&bodyA, &bodyB};

    while (!WindowShouldClose()) { // runs while window is open
        double frame_time = GetFrameTime(); // delta time in seconds
        if (frame_time > 0.20) {
            frame_time = 0.20;
        }

        accumulator += frame_time;

        while (accumulator > dt) {
            resolve_circle_collision(bodyA, bodyB);

            // updating velocity and position of each body
            for (RigidBody* body : allBodies) { // (*) -> pass in the pointer (not by value or reference)
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

            for (const RigidBody* body : allBodies) {
                const Shape* shape = body->getShape(); // Shape, not ShapeType

                if (shape->type == ShapeType::CIRCLE) {
                    const Circle* circle = static_cast<const Circle*>(shape); // convert shape to circle
                    DrawCircle(
                        static_cast<int>(body->getPositionX()),
                        static_cast<int>(body->getPositionY()),
                        static_cast<int>(circle->radius),
                        RED
                    );
                }   
            }

            DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}