#pragma once
#include "rigidbody.hpp"


float squaring(float value) { 
    return value * value;
}

// contanct forces

bool detect_collision(RigidBody& A, RigidBody& B) {
    if (A.getShape()->type == ShapeType::CIRCLE && B.getShape()->type == ShapeType::CIRCLE) {
        const Circle* circleA = static_cast<const Circle*>(A.getShape());
        const Circle* circleB = static_cast<const Circle*>(B.getShape());

        float ra = circleA->radius;
        float rb = circleB->radius;

        float square_dist = (squaring(A.getPositionX() - B.getPositionX()) + squaring(A.getPositionY() - B.getPositionY()));

        return (squaring(rb + ra) > square_dist);
    }

    // implement other types of collision (different shapes)
    return false;
}


Vector2D calculate_force(const RigidBody& body) {
    // Momentum
    float mx = body.getVelocityX() * body.getMass();
    float my = body.getVelocityY() * body.getMass();

    // Force
    return {mx * 0.01, my * 0.01};
}

void update_contact(RigidBody& A, RigidBody& B) {
    if (detect_collision(A, B)) {
        A.add_forces(calculate_force(A));
        B.add_forces(calculate_force(B));
    }
}


// non-contact forces
void update_noncontact(RigidBody& body) {
    float fg = 9.8f * body.getMass();
    body.add_forces({0.0f, fg});

    // air resistance
    float fx = squaring(body.getVelocityX()) * -0.1f;
    float fy = squaring(body.getVelocityY()) * -0.1f;
    body.add_forces({fx, fy});
}