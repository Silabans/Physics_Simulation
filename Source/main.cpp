#include "rendering.hpp"


int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    const int FPS = 120;
    InitWindow(screenWidth, screenHeight, "2D Physics Engine");
    SetTargetFPS(FPS);


    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;
    const float dt = 0.01f;
    int impulseIterations = 10;

    // body initialisation using unique pointers (to store the objects in the heap rather
    // than the stack => more spacious)
    std::vector<std::unique_ptr<RigidBody>> bodies; 

    //initialiseUnmovable(bodies);
    initialiseWalls(bodies, screenHeight, screenWidth);
    initialiseCircles(bodies);
    initialiseBoxes(bodies);
    initialiseUnmovable(bodies);

    while (!WindowShouldClose()) { // runs while window is open
        double frame_time = GetFrameTime(); // delta time in seconds
        if (frame_time > 0.20) {
            frame_time = 0.20;
        }
        accumulator += frame_time;

        while (accumulator > dt) {
            userInteraction(bodies, dt);
            physicsResolution(bodies, dt, FPS, impulseIterations);
            accumulator -= dt;
        }

        render(bodies);
        }
    CloseWindow();
    return 0;
}

