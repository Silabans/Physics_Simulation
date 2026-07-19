#include "raylib.h"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "Raylib 6.0 Setup");

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(BLACK);

        DrawText("Raylib is working!", 20, 20, 24, WHITE);

        DrawCircle(screenWidth / 2, screenHeight / 2, 80, GREEN);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
