#pragma once
#include <memory>


enum class ShapeType { CIRCLE, BOX };

// shape superclass
struct Shape { 
    ShapeType type;

    // this ensures that destruction occurs for all instantiated subclasses,
    // and not just the Shape superclass
    // -> virtual: to involve the subclass
    // -> ~Shape(): the ~ denotes a destructor of the superclass
    virtual ~Shape() = default;
};

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
    Vector2D totalForce;
    float inverseMass; 
    float mass;
    std::unique_ptr<Shape> shape; // smart pointer => handles destruction automatically
    Color color;
    
    // the inverse is stored for calculating acceleration (a = F * 1/m)
    // divisions are more costly/slower than multiplications

public:
    RigidBody(float x, float y, float objectMass, std::unique_ptr<Shape> objectShape, Color objectColor)
        : position{x, y},
          velocity{0.0f, 0.0f},
          totalForce{0.0f, 0.0f},
          mass(objectMass),
          inverseMass(objectMass > 0.0f ? 1.0f / objectMass : 0.0f),
          color(objectColor),
          shape(std::move(objectShape)) {}
        
    const Shape* getShape() const { return shape.get(); } // gets the actual instance using the pointer

    const float getPositionX() const { return position.x; }
    const float getPositionY() const { return position.y; }
    const Vector2D getPosition() const { return position; } // returns the pair of x and y 

    const float getVelocityX() const { return velocity.x; }
    const float getVelocityY() const { return velocity.y; }
    const Vector2D getVelocity() const { return velocity; }

    const float getMass() const { return mass; }
    const float getInverseMass() const { return inverseMass; }

    const Color getColor() const { return color; }

    void setVelocityX(float new_vel) { velocity.x = new_vel; }
    void setVelocityY(float new_vel) { velocity.y = new_vel; }
    void setVelocity(Vector2D new_vel) { velocity = new_vel; }

    // Position Setters to fix wall sinking
    void setPositionX(float new_pos) { position.x = new_pos; }
    void setPositionY(float new_pos) { position.y = new_pos; }
    void setPosition(Vector2D new_pos) { position = new_pos; }


    void applyImpulse(Vector2D impulse) {
        velocity.x += impulse.x * inverseMass;
        velocity.y += impulse.y * inverseMass;
    }

    void integrate_pos(float dt) {
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
    }

    void calculate_velocity(float dt) {
        float ax = totalForce.x * inverseMass; // Don't use 'tuple' unpacking -> not a C++ feature
        float ay = totalForce.y * inverseMass;

        velocity.x += ax * dt;
        velocity.y += ay * dt;
    }

    void add_forces(Vector2D force) {
        totalForce.x += force.x;
        totalForce.y += force.y;
    }

    void reset_forces() {
        totalForce = {0.0f, 0.0f};
    }
};