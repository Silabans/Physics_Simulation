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
        float e = 0.75f; // coefficient of restitution

        float j = -(1.0f + e)*projectedVel / invMassSum; // magnitude of impulse
        Vector2D impulse = normal * j;

        a.setVelocity(va + impulse*a.getInverseMass());
        b.setVelocity(vb - impulse*b.getInverseMass());
    }

    // resolve overlapping circles
    float overlap = total_radius - dist;
    float percent = 0.8f;

    // inverse mass is included to ensure that the distance corrected is inversely 
    // proportional to mass (the larger ball is displaced by a smaller amount)
    Vector2D correction = normal * (overlap / invMassSum) * percent; 
    a.setPosition(a.getPosition() + correction * a.getInverseMass()); // divide back by its mass
    b.setPosition(b.getPosition() - correction * b.getInverseMass());
}


void resolve_box_collision(RigidBody& a, RigidBody& b, SATresult sat) {
    if (!sat.collision) return;

    // cast the shape (box) into actual box instances
    const Box* box_a = static_cast<const Box*>(a.getShape());
    const Box* box_b = static_cast<const Box*>(b.getShape());


    // Find the vertex p in the other box (deepest in the box -> lowest dot product value)
    std::array<Vector2D, 4> verts_b = box_b->get_vertices(a.getPosition());
    Vector2D p = verts_b[0];
    float min_overlap = dot(sat.normal, p);

    for (int i = 1; i < 4; ++i) {
        float overlap = dot(sat.normal, verts_b[i]);
        if (overlap < min_overlap) {
            min_overlap = overlap;
        }
    }

    // Lever from the centers of the bodies to point of contact (perpendicular distances between pivot/COG and point of contact)
    Vector2D ra = p - a.getPosition();
    Vector2D rb = p - b.getPosition();

    // Calculating jasper's level of gayness (adding up angular velocity)
    Vector2D v_ra = a.getVelocity() + cross(ra, a.getAngularVel()); 
    Vector2D v_rb = b.getVelocity() + cross(rb, b.getAngularVel());
    Vector2D vrel = v_ra - v_rb;

    float projected_speed = dot(vrel, sat.normal);
    if (projected_speed > 0.0f) return; // moving apart already

    // effective torque (if normal and lever are parallel -> zero torque, as sin(0) = 0)
    float ra_cross_normal = cross(ra, sat.normal);
    float rb_cross_normal = cross(rb, sat.normal);

    float totalInverseMass = a.getInverseMass() + b.getInverseMass();

    // Why is the cross product squared?
    // change in angular velocity (w) = Inverse mass * cross product
    // w changes the contact point's velocity via cross(r, w) -> so the cross product feeds into itself (increase in angular velocity leads to higher point velocity)
    // change in v_point = cross(r, w) -> done to find the actual change in velocity (not just rotation) of the point
    // This is because v = w x r
    float inverseMassSum = totalInverseMass + squaring(ra_cross_normal) * a.getInverseInertia() + squaring(rb_cross_normal) * b.getInverseInertia();
    

    // Calculating impulse magnitude j
    float e = 0.5f;
    float j = -(1.0 + e) * projected_speed / inverseMassSum;
    Vector2D impulse = sat.normal * j; // j in the direction of the normal

    // updating velocity and angular velocity
    a.setVelocity(a.getVelocity() + impulse * a.getInverseMass());
    a.setAngularVel(a.getAngularVel() + cross(ra, impulse) * a.getInverseMass());

    b.setVelocity(b.getVelocity() - impulse * b.getInverseMass());
    b.setAngularVel(b.getAngularVel() - cross(rb, impulse) * b.getInverseMass());

    float percentage_change = 0.6f;
    if (totalInverseMass > 0.0f) {
        // large total mass and overlap depth -> larger correction, 
        Vector2D correction = sat.normal * ((sat.overlap_depth / totalInverseMass) * percentage_change);

        // the heavier object will be displaced less (more inertia)
        a.setPosition(a.getPosition() + correction * a.getInverseMass());
        b.setPosition(b.getPosition() - correction * b.getInverseMass());
    }

}


void resolve_all_collisions(std::vector<std::unique_ptr<RigidBody>>& bodies, SATresult sat) {
    int count = bodies.size();

    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) { // starts at i to skip the collisions between repeating pairs of bodies
            const Shape shape1 = *bodies[i]->getShape();
            const Shape shape2 = *bodies[j]->getShape();

            if (shape1.type == ShapeType::CIRCLE && shape2.type == ShapeType::CIRCLE) {
                resolve_circle_collision(*bodies[i], *bodies[j]);
            }

            SATresult sat = check_box_collision(*bodies[i], *bodies[j]);
            if (sat.collision == false) return;

            if (shape1.type == ShapeType::BOX && shape2.type == ShapeType::BOX) {
                resolve_box_collision(*bodies[i], *bodies[j], sat);
            }

            else {
                // implement box-to-circle collisions
            }
            
        }
    }
}


// non-contact forces
void update_noncollision(RigidBody& body) {
    float fg = 900.0f * body.getMass();
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

    if (x + radius > 800.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(800.0f - radius);
        } else body.setVelocityX(-std::abs(vx) * restitution);

    } else if (x - radius < 0.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(radius);
        } else body.setVelocityX(std::abs(vx) * restitution);
    }

    if (y - radius < 0.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(radius);
        } else body.setVelocityY(std::abs(vy) * restitution);

    } else if (y + radius > 600.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(600.0f - radius);
        } else body.setVelocityY(-std::abs(vy) * restitution);
    }
}