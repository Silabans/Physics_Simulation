#pragma once
#include "rigidbody.hpp"
#include <algorithm>
#include <cmath>


float squaring(float value) { 
    return value * value;
}

// Math operations
inline Vector2D operator-(const Vector2D& a, const Vector2D& b) { return {a.x - b.x, a.y - b.y}; }
inline Vector2D operator+(const Vector2D& a, const Vector2D& b) { return {a.x + b.x, a.y + b.y}; }
inline Vector2D operator*(const Vector2D& a, const Vector2D& b) { return {a.x * b.x, a.y * b.y}; }
inline float dot(const Vector2D& a, const Vector2D& b) { return a.x * b.x + a.y * b.y; }

// contanct forces
bool detect_collision(RigidBody& A, RigidBody& B) {
    if (A.getShape()->type == ShapeType::CIRCLE && B.getShape()->type == ShapeType::CIRCLE) {
        const Circle* circleA = static_cast<const Circle*>(A.getShape());
        const Circle* circleB = static_cast<const Circle*>(B.getShape());

        float ra = circleA->radius;
        float rb = circleB->radius;

        Vector2D Apos = A.getPosition();
        Vector2D Bpos = B.getPosition();

        Vector2D delta = Apos - Bpos;
        float square_dist = dot(delta, delta);

        float overlap = squaring(rb + ra) - square_dist;

        if (overlap > 0) {
            float total_radius = ra + rb;
            float x = total_radius - std::abs(delta.x);
            float y = total_radius - std::abs(delta.y);

            float restoration_x = (Apos.x > Bpos.x) ? x : -x;
            float restoration_y = (Apos.y > Bpos.y) ? y : -y;

            if (std::abs(delta.x) < total_radius) {
                A.setPositionX(Apos.x + restoration_x);
                B.setPositionX(Bpos.x - restoration_x);
            }

            if (std::abs(delta.y) < total_radius) {
                A.setPositionY(Apos.y + restoration_y);
                B.setPositionY(Bpos.y - restoration_y);
            }

            return true;
        }
        else {
            return false;
        }
    }

    // implement other types of collision (different shapes)
    return false;
}


Vector2D calculate_vrel(const RigidBody& a, const RigidBody& b) {
    // Impulse
    Vector2D va = a.getVelocity();
    Vector2D vb = b.getVelocity();
    Vector2D vrel = va - vb;

    return vrel;
}


void resolve_circle_collision(RigidBody& a, RigidBody& b) {
    if (a.getShape()->type != ShapeType::CIRCLE || b.getShape()->type != ShapeType::CIRCLE) return;

    const Circle* circleA = static_cast<const Circle*>(a.getShape());
    const Circle* circleB = static_cast<const Circle*>(b.getShape());

    float ra = circleA->radius;
    float rb = circleB->radius;
    float total_radius = ra + rb;

    Vector2D a_pos = a.getPosition();
    Vector2D b_pos = b.getPosition();

    Vector2D delta = a_pos - b_pos;
    float square_dist = dot(delta, delta);

    if (square_dist >= squaring(total_radius)) return;

    


    float e = 0.80f; // coefficient of restitution

    float invMassSum = a.getInverseMass() + b.getInverseMass();
    Vector2D va = a.getVelocity();
    Vector2D vb = b.getVelocity();
    Vector2D vrel = calculate_vrel(a, b);

    float jx = (-(1 + e)*vrel.x) / invMassSum;
    float jy = (-(1 + e)*vrel.y) / invMassSum;

    Vector2D delta_va = {jx*a.getInverseMass(), jy*a.getInverseMass()};
    Vector2D delta_vb = {jx*b.getInverseMass(), jy*b.getInverseMass()};

    a.setVelocity(va - delta_va);
    b.setVelocity(vb + delta_vb);
}


// non-contact forces
void update_noncollision(RigidBody& body) {
    float fg = 600.0f * body.getMass();
    body.add_forces({0.0f, fg});

    // air resistance
    // opposes the motion
    //float fx = body.getVelocityX() * std::abs(body.getVelocityX()) * -0.1f;
    //float fy = body.getVelocityY() * std::abs(body.getVelocityY()) * -0.1f;
    //body.add_forces({fx, fy});
}

void resolve_boundaries(RigidBody& body) {
    // boundary collisions
    const Shape* shape = body.getShape();
    float radius;

    if (shape->type == ShapeType::CIRCLE) {
        const Circle* circle = static_cast<const Circle*>(shape);
        radius = circle->radius;
    }

    float vx = body.getVelocityX();
    float vy = body.getVelocityY();

    int x = body.getPositionX();
    int y = body.getPositionY();

    float restitution = 0.8f;
    float restThreshold = 30.0f;

    if (x + radius >= 798) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(800.0f - radius);
        } else body.setVelocityX(-std::abs(vx) * restitution);

    } else if (x - radius <= 2) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setVelocityX(0.0f + radius);
        } else body.setVelocityX(std::abs(vx) * restitution);
    }

    if (y - radius <= 2) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(5.0f + radius);
        } else body.setVelocityY(std::abs(vy) * restitution);

    } else if (y + radius >= 598) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(598.0f - radius);
        } else body.setVelocityY(-std::abs(vy) * restitution);
    }
}