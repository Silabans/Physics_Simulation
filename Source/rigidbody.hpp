#pragma once
#include <memory>
#include <cmath>
#include <array>
#include <optional>

struct Vector2D { float x; float y; };

inline float squaring(float value) { 
    return value * value;
}

// Math operations
inline Vector2D operator-(const Vector2D& a, const Vector2D& b) { return {a.x - b.x, a.y - b.y}; }
inline Vector2D operator+(const Vector2D& a, const Vector2D& b) { return {a.x + b.x, a.y + b.y}; }
inline Vector2D operator*(const Vector2D& a, const Vector2D& b) { return {a.x * b.x, a.y * b.y}; }
inline Vector2D operator*(const Vector2D& a, const float scalar) { return {a.x * scalar, a.y * scalar}; }
inline float dot(const Vector2D& a, const Vector2D& b) { return a.x * b.x + a.y * b.y; }
// perpendicular (cross product)
inline Vector2D cross(const Vector2D& a, float scalar) { return {-a.y * scalar, a.x * scalar}; }
// gives the magnitude and direction of rotation (about the z-axis / out from or into the screen)
inline float cross(const Vector2D& a, Vector2D& b) { return a.x * b.y - a.y * b.x; }


// Rigid Body calculations
enum class ShapeType { CIRCLE, BOX };

// shape superclass
struct Shape { 
    ShapeType type;

    // this ensures that destruction occurs for all instantiated subclasses,
    // and not just the Shape superclass
    // -> virtual: to involve the subclass
    // -> ~Shape(): the ~ denotes a destructor of the superclass
    virtual ~Shape() = default;
    virtual float compute_inertia(float mass) const = 0; // virtual -> only for inheritance
};

struct Circle : public Shape { // inheritance
    float radius;
    Circle(float r)
        : radius(r) { 
        type = ShapeType::CIRCLE; 
        }

    // formula is stored in the shape subclasses, while the actual value will be calculated and stored in the rigidbody instance
    float compute_inertia(float mass) const override {
        return 0.5f * radius * mass * mass;
    }
};

struct Box : public Shape { // inheritance
    Vector2D halfExtents; // for an upright box, these are the lines from the centre to the right side (u0) and centre to the top side (u1)
    float width;
    float height;
    float angle; // in radius, measured from the centre and with respect to the horizontal
    
    Box(float w, float h, float angleRads)
        : halfExtents{w * 0.5f, h * 0.5f}, 
        angle(angleRads),
        width(w),
        height(h) { 
        type = ShapeType::BOX; 
    }

    float compute_inertia(float mass) const override {
        return mass * (width*width + height*height) / 12.0f;
    }

    std::array<Vector2D, 2> get_axes() const {
        float c = std::cos(angle);
        float s = std::sin(angle);

        // first one is the 'right' extent (the greater the angle the higher the y comp, the lower the x comp)
        // second one is the 'up' extent (the greater the angle, the x comp goes backwards/counterclockwise)
        // and the y comp becomes smaller
        return { Vector2D{c, s}, Vector2D{-s, c} };
    }

    std::array<Vector2D, 4> get_vertices(Vector2D centre) const {
        std::array<Vector2D, 2> axes = get_axes();
        Vector2D u0 = axes[0] * halfExtents.x; // right extent displacement
        Vector2D u1 = axes[1] * halfExtents.y; // top extent displacement

        return {
            centre - u0 - u1, // Bottom-left
            centre + u0 - u1, // Bottom-right
            centre + u0 + u1, // Top-right
            centre - u0 + u1  // Top-left
        };
    }
};

// Projections based on each axis
struct Projection {
    float max;
    float min;
    Projection(float maximum, float minimum)
        : max(maximum), min(minimum) {}

    bool check_overlap(const Projection& other) {
        return !(max < other.min || other.max < min);
    }

    float get_overlap(const Projection& other) {
        return std::min(max, other.max) - std::max(min, other.min); // the distance of overlap
    }
};

// find max and min for boxes
Projection find_max_min(const std::array<Vector2D, 4> verts, const Vector2D axis) {
    float max = dot(verts[0], axis);
    float min = max;
    
    for (int i = 1; i < 4; ++i) { // iterate through the other 3 vertices
        max = std::max(max, dot(verts[i], axis));
        min = std::min(min, dot(verts[i], axis));
    }

    return Projection{ max, min };
}


class RigidBody {
private:
    Vector2D totalForce;
    Vector2D position;
    Vector2D velocity;
    float angle;
    float angular_velocity;
    float inverseMass; 
    float mass;
    float inertia;
    float inverseInertia;
    std::unique_ptr<Shape> shape; // smart pointer => handles destruction automatically
    Color color;
    
    // the inverse is stored for calculating acceleration (a = F * 1/m)
    // divisions are more costly/slower than multiplications

public:
    RigidBody(float x, float y, float objectMass, std::unique_ptr<Shape> objectShape, Color objectColor)
        : position{x, y},
          velocity{0.0f, 0.0f},
          angle(0.0f),
          angular_velocity(0.0f),
          totalForce{0.0f, 0.0f},
          mass(objectMass),
          inverseMass(objectMass > 0.0f ? 1.0f / objectMass : 0.0f),
          color(objectColor),
          shape(std::move(objectShape)) 
          {
            // calculate within the square brackets because it requires more complex calculations and using calculated values (inertia for inverse inertia)
            inertia = shape->compute_inertia(mass);
            inverseInertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
          }
        
    const Shape* getShape() const { return shape.get(); } // gets the actual instance using the pointer

    const float getPositionX() const { return position.x; }
    const float getPositionY() const { return position.y; }
    const Vector2D getPosition() const { return position; } // returns the pair of x and y 

    const float getVelocityX() const { return velocity.x; }
    const float getVelocityY() const { return velocity.y; }
    const Vector2D getVelocity() const { return velocity; }

    const float getAngularVel() const { return angular_velocity; }

    const float getMass() const { return mass; }
    const float getInverseMass() const { return inverseMass; }

    const float getInertia() const { return inertia; }
    const float getInverseInertia() const { return inverseInertia; }

    const Color getColor() const { return color; }

    void setVelocityX(float new_vel) { velocity.x = new_vel; }
    void setVelocityY(float new_vel) { velocity.y = new_vel; }
    void setVelocity(Vector2D new_vel) { velocity = new_vel; }
    void resetVelocity() { velocity = (Vector2D){0.0f, 0.0f}; }

    void setAngularVel(float new_ang_vel) { angular_velocity = new_ang_vel; }
    void resetAngularVel() { angular_velocity = 0.0f; }

    // Position Setters to fix wall sinking
    void setPositionX(float new_pos) { position.x = new_pos; }
    void setPositionY(float new_pos) { position.y = new_pos; }
    void setPosition(Vector2D new_pos) { position = new_pos; }


    void applyImpulse(Vector2D impulse) {
        velocity.x += impulse.x * inverseMass;
        velocity.y += impulse.y * inverseMass;
    }

    void integrate_pos(float dt) {
        if (inverseMass == 0.0f) return;
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        angle += getAngularVel() * dt;

        if (getAngularVel() > 0.02f) {
            float angularDamping = 0.98f; // light damping caused by friction -> to cause eventual stop in rotations
            setAngularVel(getAngularVel() * std::pow(angularDamping, dt * 60.0f));
        }
        else {
            setAngularVel(0.0f);
        }
        

        if (shape->type == ShapeType::BOX) {
            static_cast<Box*>(shape.get())->angle = angle;
            // get() fetches the actual object (not just the pointer)
        }
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


struct SATresult {
    bool collision;
    Vector2D normal; // axis of separation
    float overlap_depth = 1e9; // must be -> the minimum vector translation (MVT)
    // This means that the boxes are translated along the axis where the overlap the least
};

SATresult check_box_collision(const RigidBody& a, const RigidBody& b) {
    const Box* box_a = static_cast<const Box*>(a.getShape());
    const Box* box_b = static_cast<const Box*>(b.getShape());

    auto a_axes = box_a->get_axes();
    auto b_axes = box_b->get_axes();
    auto test_axes = { a_axes[0], a_axes[1], b_axes[0], b_axes[1] };

    auto a_verts = box_a->get_vertices(a.getPosition());
    auto b_verts = box_b->get_vertices(b.getPosition());

    SATresult result;
    result.collision = true; // defaults to true if not separating axis is found
    
    for (Vector2D axis : test_axes) {
        Projection projection_a = find_max_min(a_verts, axis);
        Projection projection_b = find_max_min(b_verts, axis);

        if (!projection_a.check_overlap(projection_b)) {
            result.collision = false;
            return result;
        }
        
        float current_overlap = projection_a.get_overlap(projection_b);
        if (current_overlap < result.overlap_depth) {
            result.overlap_depth = current_overlap;
            result.normal = axis;
        }
    }

    // Ensure that the normal points from B to A
    // Hence, A will always move away from B (instead of moving closer to B, which may occur
    // if the normal points from A to B) because the normal dictates the direction of the impulse and separation
    Vector2D centreDist = a.getPosition() - b.getPosition();

    if (dot(centreDist, centreDist) > 0.00001f) { // false if the boxes share the same centre
        if (dot(centreDist, result.normal) < 0.0f) {
            result.normal = result.normal * -1.0f;
        }
    }

    return result;
}

