#pragma once
#include "rigidbody.hpp"
#include <algorithm>


Vector2D calculate_vrel(const RigidBody& a, const RigidBody& b) {
    Vector2D va = a.getVelocity();
    Vector2D vb = b.getVelocity();

    return va - vb;
}


void resolve_circle_collision(RigidBody& a, RigidBody& b) {
    if (a.getShape()->type != ShapeType::CIRCLE || b.getShape()->type != ShapeType::CIRCLE) return;

    const Circle* circleA = static_cast<const Circle*>(a.getShape());
    const Circle* circleB = static_cast<const Circle*>(b.getShape());

    float ra = circleA->radius;
    float rb = circleB->radius;
    float total_radius = ra + rb;

    float invMassSum = a.getInverseMass() + b.getInverseMass();
    Vector2D a_pos = a.getPosition();
    Vector2D b_pos = b.getPosition();

    Vector2D delta = a_pos - b_pos;
    float square_dist = dot(delta, delta);
    float dist = std::sqrt(square_dist);

    if (square_dist >= squaring(total_radius) || square_dist == 0.0f) return;

    Vector2D va = a.getVelocity();
    Vector2D vb = b.getVelocity();
    Vector2D vrel = calculate_vrel(a, b);

    Vector2D inverseDist = {(1.0f / dist), (1.0f / dist)};
    Vector2D normal = delta * inverseDist;
    float projectedVel = dot(vrel, normal);

    // if A and B are moving away , skip the impulse resolution 
    if (projectedVel < 0.0f) { 
        float e = 0.70f; // coefficient of restitution

        float j = -(1.0f + e)*projectedVel / invMassSum; // magnitude of impulse
        Vector2D impulse = normal * j;

        a.setVelocity(va + impulse*a.getInverseMass());
        b.setVelocity(vb - impulse*b.getInverseMass());
    }

    // resolve overlapping circles
    float overlap = total_radius - dist;
    float percent = 0.6f;

    // inverse mass is included to ensure that the distance corrected is inversely 
    // proportional to mass (the larger ball is displaced by a smaller amount)
    Vector2D correction = normal * (overlap / invMassSum) * percent; 
    a.setPosition(a.getPosition() + correction * a.getInverseMass()); // divide back by its mass
    b.setPosition(b.getPosition() - correction * b.getInverseMass());
}


void resolve_box_collision(RigidBody& a, RigidBody& b, SATresult sat) {
    if (!sat.collision) return;

    // Find the vertex p in the other box (deepest in the box -> lowest dot product value)
    std::array<Vector2D, 4> verts_b;
    Vector2D p = verts_b[0];
    float min_overlap = dot(sat.normal, p);

    for (int i = 1; i < 4; ++i) {
        float overlap = dot(sat.normal, verts_b[i]);
        if (overlap < min_overlap) {
            min_overlap = overlap;
        }
    }

    
}


void resolve_all_collisions(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    int count = bodies.size();

    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            resolve_circle_collision(*bodies[i], *bodies[j]);
        }
    }
}


// non-contact forces
void update_noncollision(RigidBody& body) {
    float fg = 500.0f * body.getMass();
    body.add_forces({0.0f, fg});

    // wind
    //body.add_forces({200.0f, 0.0f});

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

    float restitution = 0.7f;
    float restThreshold = 50.0f;

    if (x + radius >= 800.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(800.0f - radius);
        } else body.setVelocityX(-std::abs(vx) * restitution);

    } else if (x - radius <= 0.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(radius);
        } else body.setVelocityX(std::abs(vx) * restitution);
    }

    if (y - radius <= 0.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(radius);
        } else body.setVelocityY(std::abs(vy) * restitution);

    } else if (y + radius >= 600.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(600.0f - radius);
        } else body.setVelocityY(-std::abs(vy) * restitution);
    }
}