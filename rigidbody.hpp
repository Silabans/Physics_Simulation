#pragma once


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
    Shape* shape;
    

    // the inverse is stored for calculating acceleration (a = F * 1/m)
    // divisions are more costly/slower than multiplications

public:
    RigidBody(float x, float y, float objectMass, Shape* objectShape) {
        position = {x, y};
        velocity = {0.0f, 0.0f};
        totalForce = {0.0f, 0.0f};
        mass = objectMass;
        inverseMass = (objectMass > 0.0f) ? (1.0f / objectMass) : 0.0f;
        shape = objectShape;

    }

    const Shape* getShape() const { return shape; }

    const float& getPositionX() const { return position.x; }
    const float& getPositionY() const { return position.y; }

    const float& getVelocityX() const { return velocity.x; }
    const float& getVelocityY() const { return velocity.y; }

    const float& getMass() const { return mass; }
    const float& getInverseMass() const { return inverseMass; }

    void integrate_pos(float dt) {
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
    }

    void calculate_velocity(float dt) {
        float ax, ay = (totalForce.x * inverseMass, totalForce.y * inverseMass);
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

    // Destructor: Ensures that the object is fully removed from the heap memory to prevent 
    // taking up unnecessary space after the object is removed
    ~RigidBody() {
        delete shape;
    }

};