#pragma once
#include <random>
#include "raylib.h"
#include <vector>
#include <algorithm>
#include <thread>
#include "forces.hpp"
#include "rigidbody.hpp"


Color colors[7] = {RED, BLUE, YELLOW, GREEN, WHITE, PURPLE, ORANGE};

// reusable random device seeding
std::mt19937& random_engine() {
    // initialised once in the first function call (static var)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}


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
    const float stiffness = 40000.0f; // strength of spring
    const float damping = 300.0f;

    Vector2D spring_force = mouse_delta * stiffness;
    Vector2D damping_force = body.getVelocity() * -damping;

    body.add_forces(spring_force + damping_force);
}


void userInteraction(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt) {
    Vector2 raylib_mouse_pos = GetMousePosition();
    Vector2D mouse_pos = { raylib_mouse_pos.x, raylib_mouse_pos.y };

    static const float spawn_frequency = 0.1f;
    static float spawn_threshold = spawn_frequency;

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


    //iteration required
    for (auto& body : bodies) {
        Vector2D delta = mouse_pos - body->getPosition();
        float dist = dot(delta, delta);

        if (IsKeyPressed(KEY_S)) {
            if (dist < squaring(300.0f)) {
                Vector2D blast_force = delta * -70000.0f;
                body->add_forces(blast_force);
            }
        }

        if (IsKeyDown(KEY_D)) {
            // reverses gravity 
            Vector2D reversed_g = {0.0f, -2000.0f};
            body->add_forces(reversed_g * body->getMass());
        }

        if (IsKeyDown(KEY_A)) {
            if (dist < squaring(60.0f)) {
                apply_mouse_spring_force(*body, delta);
            }
        }
    }
}


void physicsResolution(std::vector<std::unique_ptr<RigidBody>>& bodies, float dt, int FPS, int impulseIterations) {
    // resolve 10 times to account for smooth multibody collisions (more than 2 bodies colliding)
    for (int i = 0; i < impulseIterations; ++i) {
        // pointer used because if passed by value, 
        // a temp copy will be made and destroyed after the the function completes (local only)
        resolve_all_collisions(bodies);
    }
    for (auto& body : bodies) {
        update_noncollision(*body);
        // updating velocity and position of each body
        body->calculate_velocity(dt);
        body->integrate_pos(dt, FPS);
        body->reset_forces(); // reset forces after integrating (to avoid the same force accumulating)
        //resolve_boundaries(*body);
    }
}

// initialise walls as rigid bodies
void initialiseWalls(std::vector<std::unique_ptr<RigidBody>>& bodies, int screenHeight, int screenWidth) {
    float thickness = 100.0f;
    auto leftWall = std::make_unique<Box>(thickness, (float)screenHeight, 0.0f);
    auto rightWall = std::make_unique<Box>(thickness, (float)screenHeight, 0.0f);
    auto topWall = std::make_unique<Box>((float)screenWidth + thickness * 2.0f, thickness, 0.0f);
    auto bottomWall = std::make_unique<Box>((float)screenWidth + thickness * 2.0f, thickness, 0.0f);
    Color wallColor = DARKGRAY;

    // vertical walls
    bodies.push_back(std::make_unique<RigidBody>(-thickness * 0.4f, screenHeight * 0.5f, 0.0f, std::move(leftWall), wallColor));
    bodies.push_back(std::make_unique<RigidBody>(screenWidth + thickness * 0.4f, screenHeight * 0.5f, 0.0f, std::move(rightWall), wallColor));
    // horizontal walls
    bodies.push_back(std::make_unique<RigidBody>(screenWidth * 0.5f, -thickness * 0.4f, 0.0f, std::move(topWall), wallColor));
    bodies.push_back(std::make_unique<RigidBody>(screenWidth * 0.5f, screenHeight + thickness * 0.4f, 0.0f, std::move(bottomWall), wallColor));  
}

// body initialisations
// create randomly generated balls
void initialiseCircles(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    for (int i = 0; i < 5; ++i) {
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);

        bodies.push_back(create_circle(x, y));
    }
}


void initialiseBoxes(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    for (int i = 0; i < 4; ++i) {
        float x = 40.0f + static_cast<float>(rand() % 720);
        float y = 40.0f + static_cast<float>(rand() % 520);
        
        bodies.push_back(create_box(x, y));
    }
}


void initialiseUnmovable(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    // static ball in the middle of the screen
    auto pinballCircle = std::make_unique<Circle>(100.0f);
    auto staticPinball = std::make_unique<RigidBody>(400.0f, 300.0f, 0.0f, std::move(pinballCircle), RAYWHITE);
    staticPinball->setVelocity({0.0f, 0.0f});

    bodies.push_back(std::move(staticPinball));
}