#pragma once
#include "rigidbody.hpp"
#include <algorithm>


Vector2D calculate_vrel(const RigidBody& a, const RigidBody& b) {
    Vector2D va = a.getVelocity();
    Vector2D vb = b.getVelocity();

    return va - vb;
}


void resolve_circle_collision(RigidBody& a, RigidBody& b) {
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

        float restThreshold = 15.0f;

        if (a.getVelocityX() < restThreshold) a.setVelocityX(0.0f);
        if (b.getVelocityX() < restThreshold) b.setVelocityX(0.0f);
        if (b.getVelocityY() < restThreshold) b.setVelocityY(0.0f);


        a.setVelocity(va + impulse*a.getInverseMass());
        b.setVelocity(vb - impulse*b.getInverseMass());
    }

    // resolve overlapping circles
    float overlap = total_radius - dist;
    float percent = 0.7f;

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


    // Find the vertex p of BOTH BOXES (deepest in the box -> lowest dot product value)
    std::array<Vector2D, 4> verts_a = box_a->get_vertices(a.getPosition());
    std::array<Vector2D, 4> verts_b = box_b->get_vertices(b.getPosition());

    Vector2D p = verts_a[0];
    float min_overlap = dot(sat.normal, p);

    for (int i = 1; i < 4; ++i) {
        float overlap = dot(sat.normal, verts_a[i]);
        if (overlap < min_overlap) {
            min_overlap = overlap;
            p = verts_a[i];
        }
    }

    for (int i = 0; i < 4; ++i) {
        float overlap = dot(sat.normal * -1.0f, verts_b[i]); // negative to reverse the direction from BA to AB
        if (overlap < min_overlap) {
            min_overlap = overlap;
            p = verts_b[i];
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

    if (inverseMassSum <= 0.0001f) return; // prevents Zero Division Error

    // Calculating impulse magnitude j
    float e = 0.5f;
    float j = -(1.0 + e) * projected_speed / inverseMassSum;
    Vector2D impulse = sat.normal * j; // j in the direction of the normal

    // updating velocity and angular velocity
    a.setVelocity(a.getVelocity() + impulse * a.getInverseMass());
    a.setAngularVel(a.getAngularVel() + cross(ra, impulse) * a.getInverseInertia());

    b.setVelocity(b.getVelocity() - impulse * b.getInverseMass());
    b.setAngularVel(b.getAngularVel() - cross(rb, impulse) * b.getInverseInertia());

    float restThreshold = 4.0f;

    if (a.getVelocityX() < restThreshold) a.setVelocityX(0.0f);
    if (b.getVelocityX() < restThreshold) b.setVelocityX(0.0f);
    if (b.getVelocityY() < restThreshold) b.setVelocityY(0.0f);

    float max_correction = 10.0f; // preventing weird teleportation
    float corrected_depth = std::min(sat.overlap_depth, max_correction);

    float percentage_change = 0.8f;
    if (totalInverseMass > 0.0f) {
        // large total mass and overlap depth -> larger correction, 
        Vector2D correction = sat.normal * ((corrected_depth / totalInverseMass) * percentage_change);

        // the heavier object will be displaced less (more inertia)
        a.setPosition(a.getPosition() + correction * a.getInverseMass());
        b.setPosition(b.getPosition() - correction * b.getInverseMass());
    }

}


void resolve_box_circle_collision(RigidBody& a, RigidBody& b) {
    const Box* box = static_cast<const Box*>(a.getShape());
    const Circle* circle = static_cast<const Circle*>(b.getShape());

    // we need to transform into the box local space/xy graph (from the world space)
    // => This is to ensure that the relative position between the box and the circle when the box is "unrotated" to be made upright
    // why do we need the box to be upright? It is so that we can use halfExtents x and y to gauge whether the circle is overlapping
    // with the box or not, and to find the point where the circle and box are overlapping

    Vector2D d_world = b.getPosition() - a.getPosition();
    // Vector AB => first step: translation (taking the centre of the box A as the local origin)

    std::array<Vector2D, 2> axes = box->get_axes();
    float localX = dot(d_world, axes[0]);
    float localY = dot(d_world, axes[1]);
    //2nd step: x_local = dx*cos(t) + dysin(t) || y_local = -dx*sin(t) + dy*cos(t)

    Vector2D halfExtents = box->halfExtents;

    // if the bodies aren't colliding, skip
    if (std::abs(localX) > halfExtents.x + circle->radius && std::abs(localY) > halfExtents.y + circle->radius) return;

    // Let p be the point of contact between the two bodies
    // if the localX is within the horizontal length of the box, the contact is on a horizontal surface
    // if localX lies beyond, the contact is on a corner (which is halfExtent.x distance away from the box's centre)
    float px = std::clamp(localX, -halfExtents.x, halfExtents.x);
    float py = std::clamp(localY, -halfExtents.y, halfExtents.y);
    

    // transforming these coordinates back to the world space
    float angle = box->angle;
    float x = px*std::cos(angle) - py*std::sin(angle);
    float y = px*std::sin(angle) + py*std::cos(angle);

    Vector2D world_p = (Vector2D){x, y} + a.getPosition();
    Vector2D delta = b.getPosition() - world_p; // contact point to circle centre (PC)

    float squaredist = dot(delta, delta);
    float dist = std::sqrt(squaredist);
    float inversedist = 1.0f / dist;

    Vector2D normal = delta*inversedist;

    Vector2D local_contact = a.getPosition() - world_p; // vector from box centre to contact point
    Vector2D vrel = b.getVelocity() - (a.getVelocity() + cross(local_contact, a.getAngularVel()));
    float projected_speed = dot(vrel, normal);

    float normal_cross_product = cross(local_contact, normal);
    float rotational_inertia_a = a.getInverseInertia() * squaring(normal_cross_product);
    float inverseMassSum = b.getInverseMass() + a.getInverseMass() + rotational_inertia_a;

    // calculating impulse
    float e = 0.6;
    float j = -(1 + e) * projected_speed / inverseMassSum;
    Vector2D impulse = normal * j; // gives direction (along the normal)

    a.setVelocity(a.getVelocity() - impulse * a.getInverseMass());
    a.setAngularVel(a.getAngularVel() - cross(local_contact, impulse) * a.getInverseInertia());

    b.setVelocity(b.getVelocity() + impulse * b.getInverseMass());

    // spatial/overlap correction
    float overlap = std::sqrt() + ra

}


void resolve_all_collisions(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    int count = bodies.size();

    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) { // starts at i to skip the collisions between repeating pairs of bodies
            RigidBody& a = *bodies[i];
            RigidBody& b = *bodies[j];

            if (a.getShape()->type == ShapeType::CIRCLE && b.getShape()->type == ShapeType::CIRCLE) {
                resolve_circle_collision(*bodies[i], *bodies[j]);
            }

            if (a.getShape()->type == ShapeType::BOX && b.getShape()->type == ShapeType::BOX) {
                SATresult sat = check_box_collision(a, b);
                resolve_box_collision(a, b, sat);
            }

            if (a.getShape()->type == ShapeType::BOX && b.getShape()->type == ShapeType::CIRCLE) {
                resolve_box_circle_collision(a, b);
            }

            else {
                // implement other types of collisions
            }
            
        }
    }
}

// non-contact forces
void update_noncollision(RigidBody& body) {
    if (body.getInverseMass() == 0.0f) return;

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


// no longer in use
void resolve_boundaries(RigidBody& body) {
    if (body.getInverseMass() == 0.0f) return;

    // boundary collisions
    const Shape* shape = body.getShape();
    float halfW; float halfH;

    if (shape->type == ShapeType::CIRCLE) {
        const Circle* circle = static_cast<const Circle*>(shape);
        halfW = halfH = circle->radius;
    }
    else if (shape->type == ShapeType::BOX) {
        const Box* box = static_cast<const Box*>(shape);
        
        // find the extreme points along the x and y axes
        std::array<Vector2D, 4> verts = box->get_vertices(body.getPosition());
        float minX = verts[0].x; float maxX = verts[0].x;
        float minY = verts[0].y; float maxY = verts[0].y;

        for (int i = 1; i < 4; ++i) { // from the second to fourth vertex (the first is the default vertex)
            minX = std::min(minX, verts[i].x);
            minY = std::min(minY, verts[i].y);
            maxX = std::max(maxX, verts[i].x);
            maxY = std::max(maxY, verts[i].y);
        }

        // measure how far the furthest point is (along the x or y axis) from the centre of the box
        halfH = maxY - body.getPositionY(); 
        halfW = maxX - body.getPositionX();
    }


    float vx = body.getVelocityX();
    float vy = body.getVelocityY();

    int x = body.getPositionX();
    int y = body.getPositionY();

    float restitution = 0.7f;
    float restThreshold = 50.0f;

    if (x + halfW > 800.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(800.0f - halfW);
        } else body.setVelocityX(-std::abs(vx) * restitution);

    } else if (x - halfW < 0.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(halfW);
        } else body.setVelocityX(std::abs(vx) * restitution);
    }

    if (y - halfH < 0.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(halfH);
        } else body.setVelocityY(std::abs(vy) * restitution);

    } else if (y + halfH > 600.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(600.0f - halfH);
        } else body.setVelocityY(-std::abs(vy) * restitution);

        float surfaceFriction = 0.95f;
        // angular vel decreases due to frictional forces
        body.setAngularVel(body.getAngularVel() * surfaceFriction);
    }
}