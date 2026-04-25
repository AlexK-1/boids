// compile: cc main.c -o boids -lraylib -lm -O2

#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// <======================================== MACROS AND DEFINITIONS ========================================>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 450

#define BOIDS_COUNT 1000
#define BOID_WIDTH 20
#define BOID_HEIGHT 30

#define BOID_MAX_SPEED 1.0f
#define BOID_MIN_SPEED 0.4f

#define BOID_BOUND_PADDING 50
#define BOID_AVOID_RADIUS 30
#define BOID_MOUSE_AVOID_RADIUS 70
#define BOID_VISIBLE_RADIUS 70

#define BOID_BOUND_FACTOR 0.08f
#define BOID_AVOID_FACTOR 0.03f
#define BOID_ALIGNMENT_FACTOR 0.05f
#define BOID_COHESION_FACTOR 0.001f

#define BOID_AVOID_RADIUS_SQ (BOID_AVOID_RADIUS * BOID_AVOID_RADIUS)
#define BOID_MOUSE_AVOID_RADIUS_SQ (BOID_MOUSE_AVOID_RADIUS * BOID_MOUSE_AVOID_RADIUS)
#define BOID_VISIBLE_RADIUS_SQ (BOID_VISIBLE_RADIUS * BOID_VISIBLE_RADIUS)

#define CHUNK_SIZE_BOIDS 1024
#define CHUNK_SIZE_PIXELS BOID_VISIBLE_RADIUS

// <======================================== STRUCTURES AND TYPEDEFS =======================================>

typedef struct {
    Vector2 pos;
    Vector2 speed;
    Vector2 visualSpeed;
} Boid;

typedef uint16_t BoidIndex;
typedef uint16_t ChunkSize;

typedef struct {
    BoidIndex boids[CHUNK_SIZE_BOIDS]; // Array of boids
    ChunkSize count; // Boids count
} Chunk;

typedef struct {
    Chunk *chunks;
    int screenWidth, screenHeight;
    uint16_t rows, cols; // Number of chunks by width/height
    uint32_t chunksCount;
} Grid;

// <============================================ GRID AND CHUNKS ===========================================>

// Fill the chunks with zeros (clear)
void ClearGrid(Grid *grid) {
    for (uint32_t i = 0; i < grid->chunksCount; i++) {
        grid->chunks[i].count = 0;
    }
}

// Fill chunks with boids
void FillGrid(Grid *grid, Boid *boids) {
    for (uint16_t i = 0; i < BOIDS_COUNT; i++) {
        Boid *boid = &boids[i];

        float x = boid->pos.x;
        float y = boid->pos.y;

        if (x < 0) x = 0;
        else if (x > grid->screenWidth) x = grid->screenWidth;
        if (y < 0) y = 0;
        else if (y > grid->screenHeight) y = grid->screenHeight;
        
        uint16_t chunkX = x / CHUNK_SIZE_PIXELS;
        uint16_t chunkY = y / CHUNK_SIZE_PIXELS;
        uint32_t chunkIndex = chunkX + chunkY*grid->cols;

        Chunk *chunk = &grid->chunks[chunkIndex];

        if (chunk->count < CHUNK_SIZE_BOIDS)
            chunk->boids[chunk->count++] = i;
    }
}

// Add data to the grid, completely refill chunks (delete and recrate all chunks)
void InitGrid(Grid *grid, Boid *boids, int screenWidth, int screenHeight) {
    grid->screenWidth = screenWidth;
    grid->screenHeight = screenHeight;
    grid->cols = (uint16_t)(screenWidth/CHUNK_SIZE_PIXELS) + 1;
    grid->rows = (uint16_t)(screenHeight/CHUNK_SIZE_PIXELS) + 1;
    grid->chunksCount = grid->rows * grid->cols;

    if (grid->chunks != NULL) {
        free(grid->chunks);
    }
    grid->chunks = RL_CALLOC(grid->chunksCount, sizeof(Chunk));

    FillGrid(grid, boids);
}

// <================================================= BOIDS ================================================>

// Push the boid away from the bound
void BoidBound(Boid *boid, int screenWidth, int screenHeight) {
    if (boid->pos.x < BOID_BOUND_PADDING)
        boid->speed.x += BOID_BOUND_FACTOR;
    else if (boid->pos.x > screenWidth-BOID_BOUND_PADDING)
        boid->speed.x -= BOID_BOUND_FACTOR;
    if (boid->pos.y < BOID_BOUND_PADDING)
        boid->speed.y += BOID_BOUND_FACTOR;
    else if (boid->pos.y > screenHeight-BOID_BOUND_PADDING)
        boid->speed.y -= BOID_BOUND_FACTOR;
}

// Speed up or slow down the boid so that its speed is between BOID_MIN_SPEED and BOID_MAX_SPEED
void BoidNormalSpeed(Boid *boid) {
    float speed = Vector2Length(boid->speed);
    if (speed == 0) // Randomize boid's speed, if it stops
        boid->speed = (Vector2){GetRandomValue(-10, 10)/10.0, GetRandomValue(-10, 10)/10.0};
    if (speed > BOID_MAX_SPEED)
        boid->speed = Vector2Scale(boid->speed, BOID_MAX_SPEED/speed);
    else if (speed < BOID_MIN_SPEED)
        boid->speed = Vector2Scale(boid->speed, BOID_MIN_SPEED/speed);
}

/* Update boid rules:
    1. separation: steer to avoid crowding local flockmates
    2. alignment: steer towards the average heading of local flockmates
    3. cohesion: steer to move towards the average position (center of mass) of local flockmates */
void UpdateBoid(Boid *boids, Grid *grid, BoidIndex boidIndex, bool mouseAvoid, Vector2 mousePosition) {
    Boid *boid = &boids[boidIndex];
    Vector2 close = { 0 }; // Avoid vector
    BoidIndex neighborsCount = 0;
    Vector2 neighborsSpeed = { 0 }; // Alignment speed vector
    Vector2 neighborsPos = { 0 }; // Cohesion position vectorallNeighborsCount
    
    // Position limitation by screen size
    Vector2 pos = boid->pos;
    if (pos.x < 0) pos.x = 0;
    else if (pos.x > grid->screenWidth) pos.x = grid->screenWidth;
    if (pos.y < 0) pos.y = 0;
    else if (pos.y > grid->screenHeight) pos.y = grid->screenHeight;

    // Get boid's chunk
    int32_t chunkX = boid->pos.x / CHUNK_SIZE_PIXELS;
    int32_t chunkY = boid->pos.y / CHUNK_SIZE_PIXELS;

    // 8 neighboring chunks + current chunk
    for (int32_t x = chunkX-1; x <= chunkX+1; x++) {
        for (int32_t y = chunkY-1; y <= chunkY+1; y++) {
            if ((x < 0) || (x >= grid->cols) || (y < 0) || y >= grid->rows) continue;

            uint32_t chunkIndex = x + y*grid->cols;
            Chunk *chunk = &grid->chunks[chunkIndex];

            // Processing boids from neighboring chunks
            for (BoidIndex i = 0; i < chunk->count; i++) {
                BoidIndex otherBoidIndex = chunk->boids[i];
                if (otherBoidIndex == boidIndex) continue;

                Boid *otherBoid = &boids[otherBoidIndex];
                Vector2 distanceV = Vector2Subtract(boid->pos, otherBoid->pos);
                float distance_sqr = Vector2LengthSqr(distanceV);
        
                if (distance_sqr < BOID_AVOID_RADIUS_SQ) {
                    close = Vector2Add(close, distanceV);  // Calculate avoid
                } else if (distance_sqr < BOID_VISIBLE_RADIUS_SQ) {
                    neighborsCount++;
                    neighborsSpeed = Vector2Add(neighborsSpeed, otherBoid->speed); // Calculate alignment
                    neighborsPos = Vector2Add(neighborsPos, otherBoid->pos); // Calculate cohesion
                }
            }
        }
    }

    // Alignment
    if (neighborsCount > 0) {
        neighborsSpeed = Vector2Scale(neighborsSpeed, 1.0f / neighborsCount);
        boid->speed = Vector2Add(boid->speed, Vector2Scale(neighborsSpeed, BOID_ALIGNMENT_FACTOR));
    }
    
    // Cohesion
    if (neighborsCount > 0) {
        neighborsPos = Vector2Scale(neighborsPos, 1.0f / neighborsCount);
        neighborsPos = Vector2Subtract(neighborsPos, boid->pos);
        boid->speed = Vector2Add(boid->speed, Vector2Scale(neighborsPos, BOID_COHESION_FACTOR));
    }

    // Avoid mouse
    if (mouseAvoid) {
        Vector2 distance = Vector2Subtract(boid->pos, mousePosition);
        if (Vector2LengthSqr(distance) < BOID_MOUSE_AVOID_RADIUS_SQ)
            close = Vector2Add(close, Vector2Scale(distance, 3));
    }

    // Avoid
    boid->speed = Vector2Add(boid->speed, Vector2Scale(close, BOID_AVOID_FACTOR));
}

// Drawing boid's rotated triangle
void DrawBoid(Boid *boid, Color color) {
    Vector2 normalSpeed = Vector2Normalize(boid->visualSpeed);
    float sinRotation = normalSpeed.x;
    float cosRotation = -normalSpeed.y;

    Vector2 p1 = {boid->pos.x + 0*cosRotation + (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y + 0*sinRotation - (BOID_HEIGHT/2.0)*cosRotation};
    Vector2 p2 = {boid->pos.x - (BOID_WIDTH/2.0)*cosRotation - (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y - (BOID_WIDTH/2.0)*sinRotation + (BOID_HEIGHT/2.0)*cosRotation};
    Vector2 p3 = {boid->pos.x + (BOID_WIDTH/2.0)*cosRotation - (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y + (BOID_WIDTH/2.0)*sinRotation + (BOID_HEIGHT/2.0)*cosRotation};

    DrawTriangle(p1, p2, p3, color);
}

// <================================================= MAIN =================================================>

int main(int argc, char *argv[]) {
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Boids");
    SetTargetFPS(60);

    Boid *boids = RL_CALLOC(BOIDS_COUNT, sizeof(Boid)); // Boids array
    Grid grid = { 0 }; // Grid of chunks

    for (BoidIndex i = 0; i < BOIDS_COUNT; i++) {
        Boid *boid = &boids[i];
        boid->pos = (Vector2){GetRandomValue(0, WINDOW_WIDTH), GetRandomValue(0, WINDOW_HEIGHT)};
        boid->speed = (Vector2){GetRandomValue(-10, 10)/10.0, GetRandomValue(-10, 10)/10.0};
    }
    InitGrid(&grid, boids, GetScreenWidth(), GetScreenHeight());

    bool pause = false, showVisibleCircles = false, mouseAvoid = false;

    Camera2D camera = { 0 };
    camera.zoom = 1.0f;

    while (!WindowShouldClose()) {
        // Keys
        if (IsKeyPressed(KEY_SPACE)) pause = !pause;
        if (IsKeyPressed(KEY_S)) showVisibleCircles = !showVisibleCircles;
        if (IsKeyPressed(KEY_C)) mouseAvoid = !mouseAvoid;

        // Camera
        float wheel = GetMouseWheelMove();
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) wheel = -1.0;
        if (IsKeyPressed(KEY_KP_ADD) || (IsKeyPressed(KEY_EQUAL) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)))) wheel = 1.0;
        if (wheel != 0) {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
            camera.offset = GetMousePosition();
            camera.target = mouseWorldPos;
            float scale = 0.2f*wheel;
            camera.zoom = Clamp(expf(logf(camera.zoom)+scale), 0.125f, 64.0f);
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            delta = Vector2Scale(delta, -1.0f/camera.zoom);
            camera.target = Vector2Add(camera.target, delta);
        }

        Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
        int screenWidth = GetScreenWidth();
        int screenHeight = GetScreenHeight();
        
        if (!pause || (pause && IsKeyPressed(KEY_ENTER))) {
            if ((grid.screenWidth != screenWidth) || (grid.screenHeight != screenHeight))
                InitGrid(&grid, boids, screenWidth, screenHeight);
            
            for (BoidIndex i = 0; i < BOIDS_COUNT; i++) {
                Boid *boid = &boids[i];

                BoidBound(boid, screenWidth, screenHeight);
                UpdateBoid(boids, &grid, i, mouseAvoid, mousePosition);
                BoidNormalSpeed(boid);

                boid->visualSpeed.x = boid->visualSpeed.x*0.95f + boid->speed.x*0.05f;
                boid->visualSpeed.y = boid->visualSpeed.y*0.95f + boid->speed.y*0.05f;

                boid->pos = Vector2Add(boid->pos, boid->speed);
            }

            ClearGrid(&grid);
            FillGrid(&grid, boids);
        }

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode2D(camera);

            // Draw boid's visible area
            if (showVisibleCircles) {
                for (BoidIndex i = 0; i < BOIDS_COUNT; i++) {
                    Boid *boid = &boids[i];
                    DrawCircleV(boid->pos, BOID_VISIBLE_RADIUS, (i == 0)? (Color){230, 41, 55, 10} : (Color){0, 121, 241, 10});                
                }
            }

            // Draw boids
            for (BoidIndex i = 0; i < BOIDS_COUNT; i++) {
                Boid *boid = &boids[i];
                DrawBoid(boid, (i == 0)? RED : BLUE);
                // DrawText(TextFormat("%d", i), boid->pos.x, boid->pos.y, 20, BLACK);
            }

            // Draw a circle around the mouse
            if (mouseAvoid)
                DrawCircleV(mousePosition, BOID_MOUSE_AVOID_RADIUS, (Color){230, 41, 55, 10});

            EndMode2D();

            // Draw "Paused" label, if the simulation is paused
            if (pause)
                DrawText("Paused", screenWidth - 85, 10, 20, BLACK);
            
            DrawFPS(10, 10);
        EndDrawing();
    }

    RL_FREE(boids);
    RL_FREE(grid.chunks);
    CloseWindow();
    
    return 0;
}
