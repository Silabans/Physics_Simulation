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
    float mass = PI * (radius*radius) * 0.02f; // mass proportional to size

    auto circle = std::make_unique<Circle>(radius);

    return std::make_unique<RigidBody>(x, y, mass, std::move(circle), color);
}


std::unique_ptr<RigidBody> create_box(float x, float y) {
    std::uniform_int_distribution<std::size_t> distrib(0, 6);
    Color color = colors[distrib(random_engine())];

    float side_len = 30.0f + static_cast<float>(rand() % 30);

    float mass = (side_len*side_len) * 0.02f; // mass proportional to size

    auto box = std::make_unique<Box>(side_len, side_len, 0.0f);

    return std::make_unique<RigidBody>(x, y, mass, std::move(box), color);
}


void apply_mouse_spring_force(RigidBody& body, Vector2D mouse_delta) {
    const float stiffness = 30000.0f; // strength of spring
    const float damping = 350.0f;

    Vector2D spring_force = mouse_delta * stiffness;
    Vector2D damping_force = body.getVelocity() * -damping;

    body.add_forces(spring_force + damping_force);
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
    const float dt = 0.01f;

    const float spawn_frequency = 0.1f;
    float spawn_threshold = spawn_frequency;

    // body initialisation using unique pointers (to store the objects in the heap rather
    // than the stack => more spacious)
    std::vector<std::unique_ptr<RigidBody>> bodies; 


    // static ball in the middle of the screen
    auto pinballCircle = std::make_unique<Circle>(100.0f);
    auto staticPinball = std::make_unique<RigidBody>(400.0f, 300.0f, 0.0f, std::move(pinballCircle), RAYWHITE);
    staticPinball->setVelocity({0.0f, 0.0f});

    //bodies.push_back(std::move(staticPinball));

    // create randomly generated balls
    for (int i = 0; i < 5; ++i) {
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);

        //bodies.push_back(create_circle(x, y));
    }

    for (int i = 0; i < 4; ++i) {
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);
        
        bodies.push_back(create_box(x, y));
    }

    // initialise walls as rigid bodies
    float thickness = 40.0f;
    auto leftWall = std::make_unique<Box>(thickness, (float)screenHeight, 0.0f);
    auto rightWall = std::make_unique<Box>(thickness, (float)screenHeight, 0.0f);
    auto topWall = std::make_unique<Box>((float)screenWidth, thickness, 0.0f);
    auto bottomWall = std::make_unique<Box>((float)screenWidth, thickness, 0.0f);
    Color wallColor = ORANGE;

    // vertical walls
    bodies.push_back(std::make_unique<RigidBody>(0.0f, screenHeight * 0.5f, 0.0f, std::move(leftWall), wallColor));
    bodies.push_back(std::make_unique<RigidBody>(screenWidth, screenHeight * 0.5f, 0.0f, std::move(rightWall), wallColor));
    // horizontal walls
    bodies.push_back(std::make_unique<RigidBody>(screenWidth * 0.5f, 0.0f, 0.0f, std::move(topWall), wallColor));
    bodies.push_back(std::make_unique<RigidBody>(screenWidth * 0.5f, screenHeight, 0.0f, std::move(bottomWall), wallColor));  

    while (!WindowShouldClose()) { // runs while window is open
        double frame_time = GetFrameTime(); // delta time in seconds
        if (frame_time > 0.20) {
            frame_time = 0.20;
        }
        accumulator += frame_time;

        int impulseIterations = 10;

        Vector2 raylib_mouse_pos = GetMousePosition();
        Vector2D mouse_pos = { raylib_mouse_pos.x, raylib_mouse_pos.y };


        while (accumulator > dt) {
            // resolve 4 times to account for multibody collisions (more than 2 bodies colliding)
            for (int i = 0; i < impulseIterations; ++i) {
                // pointer used because if passed by value, 
                // a temp copy will be made and destroyed after the the function completes (local only)
                resolve_all_collisions(bodies);
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
                body->calculate_velocity(dt);
                body->integrate_pos(dt);
                body->reset_forces(); // reset forces after integrating (to avoid the same force accumulating)

            }


            if (IsKeyPressed(KEY_SPACE)) {
                bodies.push_back(create_circle(mouse_pos.x, mouse_pos.y));
            } 
            else if (IsKeyDown(KEY_SPACE)) {
                if (spawn_threshold < 0.0f) {
                    bodies.push_back(create_circle(mouse_pos.x, mouse_pos.y));
                    spawn_threshold = spawn_frequency;
                }
                else spawn_threshold -= dt;
            }

            if (IsKeyPressed(KEY_B)) {
                bodies.push_back(create_box(mouse_pos.x, mouse_pos.y));
            } 
            else if (IsKeyDown(KEY_B)) {
                if (spawn_threshold < 0.0f) {
                    bodies.push_back(create_box(mouse_pos.x, mouse_pos.y));
                    spawn_threshold = spawn_frequency;
                }
                else spawn_threshold -= dt;
            } 

            accumulator -= dt;
        }

        // Key presses for user interaction
        if (IsKeyPressed(KEY_S)) {
            for (auto& body : bodies) {
                Vector2D delta = mouse_pos - body->getPosition();
                float dist = dot(delta, delta);

                if (dist < squaring(300.0f)) {
                    Vector2D blast_force = delta * -80000.0f;
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
            DrawRectangle(0, 0, 800, 600, Fade(BLACK, 0.85f)); // faded trails

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
                else if (shape->type == ShapeType::BOX) {
                    const Box* box = static_cast<const Box*>(shape); // convert shape to circle

                    Rectangle values = { body->getPositionX(), body->getPositionY(), box->width, box->height};
                    Vector2 origin = { box->halfExtents.x, box->halfExtents.y}; // centre of the box (point of rotation)
                    float angleDegrees = box->angle *   (180.0f / PI); // raylib requires degrees

                    DrawRectanglePro(
                        values,
                        origin,
                        angleDegrees,
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