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


Color colors[7] = {RED, BLUE, YELLOW, GREEN, WHITE, PURPLE, ORANGE};

std::unique_ptr<RigidBody> create_circle(float x, float y) {
    std::uniform_int_distribution<std::size_t> distrib(0, 6);
    Color color = colors[distrib(random_engine())];

    float radius = 15.0f + static_cast<float>(rand() % 20); // radius from 15 to 45 pixels
    float mass = radius*radius * 0.02f; // mass proportional to size

    auto circle = std::make_unique<Circle>(radius);

    return std::make_unique<RigidBody>(x, y, mass, std::move(circle), color);
} 


void apply_mouse_spring_force(RigidBody& ball, Vector2D mouse_delta) {
    const float stiffness = 7000.0f; // strength of spring
    const float damping = 350.0f;

    Vector2D spring_force = mouse_delta * stiffness;
    Vector2D damping_force = ball.getVelocity() * -damping;

    ball.add_forces(spring_force + damping_force);
}


int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "2D Physics Engine");
    SetTargetFPS(60);


    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (current_dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;
    const float dt = 0.01f;
    float current_dt = dt;

    const float spawn_frequency = 0.1f;
    float spawn_threshold = spawn_frequency;

    // body initialisation using unique pointers (to store the objects in the heap rather
    // than the stack => more spacious)
    std::vector<std::unique_ptr<RigidBody>> bodies; 


    // static ball in the middle of the screen
    auto pinballCircle = std::make_unique<Circle>(100.0f);
    auto staticPinball = std::make_unique<RigidBody>(400.0f, 300.0f, 0.0f, std::move(pinballCircle), RAYWHITE);
    staticPinball->setVelocity({0.0f, 0.0f});

    bodies.push_back(std::move(staticPinball));

    // create 10 randomly generated balls
    for (int i = 0; i < 5; ++i) {
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);
        bodies.push_back(create_circle(x, y));
    }

    while (!WindowShouldClose()) { // runs while window is open
        double frame_time = GetFrameTime(); // delta time in seconds
        if (frame_time > 0.20) {
            frame_time = 0.20;
        }
        accumulator += frame_time;

        int impulseIterations = 4;

        Vector2 raylib_mouse_pos = GetMousePosition();
        Vector2D mouse_pos = { raylib_mouse_pos.x, raylib_mouse_pos.y };


        while (accumulator > current_dt) {
            // resolve 4 times to account for multibody collisions (more than 2 bodies colliding)
            for (int i = 0; i < impulseIterations; ++i) {
                // pointer used because if passed by value, 
                // a temp copy will be made and destroyed after the the function completes (local only)
                resolve_all_collisions(bodies, sat);
            }
            for (auto& body : bodies) { // (*) -> pass in the pointer (not by value or reference)
                Vector2D delta = mouse_pos - body->getPosition();
                float dist = dot(delta, delta);
                if (IsKeyDown(KEY_A)) {
                    if (dist < squaring(120.0f)) {
                        apply_mouse_spring_force(*body, delta);
                    }
                }

                update_noncollision(*body);
                // updating velocity and position of each body
                body->calculate_velocity(current_dt);
                body->integrate_pos(current_dt);
                body->reset_forces(); // reset forces after integrating (to avoid the same force accumulating)
                resolve_boundaries(*body);

            }


            if (IsKeyPressed(KEY_SPACE)) {
                bodies.push_back(create_circle(mouse_pos.x, mouse_pos.y));
            } 
            else if (IsKeyDown(KEY_SPACE)) {
                if (spawn_threshold < 0.0f) {
                    bodies.push_back(create_circle(mouse_pos.x, mouse_pos.y));
                    spawn_threshold = spawn_frequency;
                }
                else spawn_threshold -= current_dt;
            } 

            accumulator -= current_dt;
        }

        // Key presses for user interaction
        if (IsKeyPressed(KEY_S)) {
            for (auto& body : bodies) {
                Vector2D delta = mouse_pos - body->getPosition();
                float dist = dot(delta, delta);

                if (dist < squaring(300.0f)) {
                    Vector2D blast_force = delta * -10000.0f;
                    body->add_forces(blast_force);
                }
            }
        }
        if (IsKeyDown(KEY_D)) {
            for (auto& body : bodies) {
                // reverses gravity 
                Vector2D reversed_g = {0.0f, -2000.0f};
                body->add_forces(reversed_g * body->getMass());
            }
        }


        // This part is for rendering
        BeginDrawing();
            // ClearBackground(BLACK);
            DrawRectangle(0, 0, 800, 600, Fade(BLACK, 0.6f)); // faded trails

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