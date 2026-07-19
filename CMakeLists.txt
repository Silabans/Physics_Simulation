cmake_minimum_required(VERSION 3.24)

# Get project name from current folder
get_filename_component(PROJECT_NAME_FROM_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# Replace spaces with underscores
string(REPLACE " " "_" PROJECT_NAME_SAFE "${PROJECT_NAME_FROM_FOLDER}")

project(${PROJECT_NAME_SAFE} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Do not build raylib examples
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# Fetch raylib 6.0
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 6.0
    GIT_SHALLOW ON
)

FetchContent_MakeAvailable(raylib)

# make project folders
set(PROJECT_FOLDERS
    Source
    Include
    Assets
    Assets/Fonts
    Assets/Sprites
    Assets/Objects
    Assets/Sounds
    Assets/Data
    Assets/Extra
)

# Create the folders if they don't exist
foreach(folder ${PROJECT_FOLDERS})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${folder}")
endforeach()

# Create a main.cpp file with a simple raylib example if it doesn't exist
set(MAIN_CPP_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Source/main.cpp")

if(NOT EXISTS "${MAIN_CPP_PATH}")
    file(WRITE "${MAIN_CPP_PATH}"
"#include \"raylib.h\"

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, \"Raylib 6.0 Setup\");

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(BLACK);

        DrawText(\"Raylib is working!\", 20, 20, 24, WHITE);

        DrawCircle(screenWidth / 2, screenHeight / 2, 80, GREEN);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
")
endif()

# Gather all source and header files
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.cpp"
)

file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Include/*.hpp"
)

# Create the executable target
add_executable(${PROJECT_NAME_SAFE}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_include_directories(${PROJECT_NAME_SAFE} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Include"
)

# Link raylib to the project
target_link_libraries(${PROJECT_NAME_SAFE} PRIVATE raylib)

# Copy the Assets folder to the output directory after building
add_custom_command(
    TARGET ${PROJECT_NAME_SAFE} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Assets"
        "$<TARGET_FILE_DIR:${PROJECT_NAME_SAFE}>/Assets"
    COMMENT "Copying Assets folder..."
)

# Make this the startup project in Visual Studio, just for convenience
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME_SAFE})