#pragma once
#include "tools.hpp"


void render(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    BeginDrawing();
        // ClearBackground(BLACK);
        DrawRectangle(0, 0, 800, 600, Fade(BLACK, 0.85f)); // faded trails

        for (const auto& body : bodies) {
            const Shape* shape = body->getShape(); // Shape, not ShapeType

            if (shape->type == ShapeType::CIRCLE) {
                const Circle* circle = static_cast<const Circle*>(shape); // convert shape to circle
                DrawCircle(
                    static_cast<int>(body->getPositionX()),
                    static_cast<int>(body->getPositionY()),
                    static_cast<int>(circle->radius),
                    body->getColor()
                );
            }
            else if (shape->type == ShapeType::BOX) {
                const Box* box = static_cast<const Box*>(shape); // convert shape to circle

                Rectangle values = { body->getPositionX(), body->getPositionY(), box->width, box->height};
                Vector2 origin = { box->halfExtents.x, box->halfExtents.y}; // centre of the box (point of rotation)
                float angleDegrees = box->angle *   (180.0f / PI); // raylib requires degrees

                DrawRectanglePro(
                    values,
                    origin,
                    angleDegrees,
                    body->getColor()
                );
            }
        }
        DrawFPS(10, 10);
    EndDrawing();
}