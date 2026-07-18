#pragma once
#include <cmath>


enum class ShapeType { CIRCLE, BOX };

// shape superclass
struct Shape { ShapeType type; };

struct Circle : Shape {
    float radius;
    Circle(float r) { type = ShapeType::CIRCLE; radius = r; }
};

struct Box : Shape {
    float width, height;
    Box(float w, float h) { type = ShapeType::BOX; width = w; height = h; }
};


// Rigid Body calculations
struct Vector2D { float x; float y; };

class RigidBody {
private:
    Vector2D position;
    Vector2D velocity;
    Vector2D force;
    float inverseMass; 
    

    // the inverse is stored for calculating acceleration (a = F * 1/m)
    // divisions are more costly/slower than multiplications

public:
    RigidBody(float x, float y, float mass) {
        position = {x, y};
        velocity = {0.0f, 0.0f};
        force = {0.0f, 0.0f};
        inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    }

    void integrate_pos(float dt) {
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
    }

    void calculate_velocity(float dt) {
        float ax, ay = (force.x * inverseMass, force.y * inverseMass);
        velocity.x += ax * dt;
        velocity.y += ay * dt;
    }

};