// compile: cc main.c -o boids -lraylib -O2

#include <raylib.h>
#include <raymath.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>

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
#define BOID_COHESION_FACTOR 0.0005f

#define BOID_AVOID_RADIUS_SQ (BOID_AVOID_RADIUS * BOID_AVOID_RADIUS)
#define BOID_MOUSE_AVOID_RADIUS_SQ (BOID_MOUSE_AVOID_RADIUS * BOID_MOUSE_AVOID_RADIUS)
#define BOID_VISIBLE_RADIUS_SQ (BOID_VISIBLE_RADIUS * BOID_VISIBLE_RADIUS)


typedef struct {
    Vector2 pos;
    Vector2 speed;
    Vector2 visualSpeed;
    // float direction;
} Boid;


// Push the boid away from the bound
void BoidBound(Boid *boid) {
    if (boid->pos.x < BOID_BOUND_PADDING)
        boid->speed.x += BOID_BOUND_FACTOR;
    else if (boid->pos.x > GetScreenWidth()-BOID_BOUND_PADDING)
        boid->speed.x -= BOID_BOUND_FACTOR;
    if (boid->pos.y < BOID_BOUND_PADDING)
        boid->speed.y += BOID_BOUND_FACTOR;
    else if (boid->pos.y > GetScreenHeight()-BOID_BOUND_PADDING)
        boid->speed.y -= BOID_BOUND_FACTOR;
}

// Steer to avoid crowding local flockmates
void BoidAvoid(Boid *boids, size_t boidIndex, bool mouseAvoid) {
    Boid *boid = &boids[boidIndex];
    Vector2 close = { 0 };

    for (size_t i = 0; i < BOIDS_COUNT; i++) {
        if (i == boidIndex) continue;
        Boid *otherBoid = &boids[i];

        Vector2 distance = Vector2Subtract(boid->pos, otherBoid->pos);
        if (Vector2LengthSqr(distance) < BOID_AVOID_RADIUS_SQ)
            close = Vector2Add(close, distance);
    }

    // Avoid mouse
    if (mouseAvoid) {
        Vector2 distance = Vector2Subtract(boid->pos, GetMousePosition());
        if (Vector2LengthSqr(distance) < BOID_MOUSE_AVOID_RADIUS_SQ)
            close = Vector2Add(close, Vector2Scale(distance, 3));
    }

    boid->speed = Vector2Add(boid->speed, Vector2Scale(close, BOID_AVOID_FACTOR));
}

// Steer towards the average heading of local flockmates
void BoidAlign(Boid *boids, size_t boidIndex) {
    Boid *boid = &boids[boidIndex];
    size_t neighborsCount = 0;
    Vector2 neighborsSpeed = { 0 };

    for (size_t i = 0; i < BOIDS_COUNT; i++) {
        if (i == boidIndex) continue;
        Boid *otherBoid = &boids[i];

        Vector2 distance = Vector2Subtract(boid->pos, otherBoid->pos);
        if (Vector2LengthSqr(distance) < BOID_VISIBLE_RADIUS_SQ) {
            neighborsCount++;
            neighborsSpeed = Vector2Add(neighborsSpeed, otherBoid->speed);
        }
    }

    if (neighborsCount > 0) {
        neighborsSpeed = Vector2Scale(neighborsSpeed, 1.0f / neighborsCount);
        boid->speed = Vector2Add(boid->speed, Vector2Scale(neighborsSpeed, BOID_ALIGNMENT_FACTOR));
    }
}

// Steer to move towards the average position (center of mass) of local flockmates
void BoidCohesion(Boid* boids, size_t boidIndex) {
    Boid *boid = &boids[boidIndex];
    size_t neighborsCount = 0;
    Vector2 neighborsPos = { 0 };

    for (size_t i = 0; i < BOIDS_COUNT; i++) {
        if (i == boidIndex) continue;
        Boid *otherBoid = &boids[i];

        Vector2 distance = Vector2Subtract(boid->pos, otherBoid->pos);
        if (Vector2LengthSqr(distance) < BOID_VISIBLE_RADIUS_SQ) {
            neighborsCount++;
            neighborsPos = Vector2Add(neighborsPos, otherBoid->pos);
        }
    }

    if (neighborsCount > 0) {
        neighborsPos = Vector2Scale(neighborsPos, 1.0f / neighborsCount);
        neighborsPos = Vector2Subtract(neighborsPos, boid->pos);
        boid->speed = Vector2Add(boid->speed, Vector2Scale(neighborsPos, BOID_COHESION_FACTOR));
    }
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

// Drawing boid's rotated triangle
void DrawBoid(Boid *boid) {
    Vector2 normalSpeed = Vector2Normalize(boid->visualSpeed);
    float sinRotation = normalSpeed.x;
    float cosRotation = -normalSpeed.y;
    // float sinRotation = sinf(boid->direction);
    // float cosRotation = cosf(boid->direction);

    Vector2 p1 = {boid->pos.x + 0*cosRotation + (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y + 0*sinRotation - (BOID_HEIGHT/2.0)*cosRotation};
    Vector2 p2 = {boid->pos.x - (BOID_WIDTH/2.0)*cosRotation - (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y - (BOID_WIDTH/2.0)*sinRotation + (BOID_HEIGHT/2.0)*cosRotation};
    Vector2 p3 = {boid->pos.x + (BOID_WIDTH/2.0)*cosRotation - (BOID_HEIGHT/2.0)*sinRotation,
                  boid->pos.y + (BOID_WIDTH/2.0)*sinRotation + (BOID_HEIGHT/2.0)*cosRotation};

    DrawTriangle(p1, p2, p3, BLUE);
}


int main(int argc, char *argv[]) {
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Boids");
    SetTargetFPS(60);

    // Boids array
    Boid *boids = RL_CALLOC(BOIDS_COUNT, sizeof(Boid));

    for (size_t i = 0; i < BOIDS_COUNT; i++) {
        Boid *boid = &boids[i];
        boid->pos = (Vector2){GetRandomValue(0, WINDOW_WIDTH), GetRandomValue(0, WINDOW_HEIGHT)};
        boid->speed = (Vector2){GetRandomValue(-10, 10)/10.0, GetRandomValue(-10, 10)/10.0};
        // boid->direction = atan2f(boid->speed.x, -boid->speed.y);
    }

    bool pause = false, showVisibleCircles = false, mouseAvoid = false;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) pause = !pause;
        if (IsKeyPressed(KEY_S)) showVisibleCircles = !showVisibleCircles;
        if (IsKeyPressed(KEY_C)) mouseAvoid = !mouseAvoid;
        
        if (!pause) {
            for (size_t i = 0; i < BOIDS_COUNT; i++) {
                Boid *boid = &boids[i];
                // Vector2 oldSpeed = boid->speed;

                BoidBound(boid);
                BoidAlign(boids, i);
                BoidCohesion(boids, i);
                BoidAvoid(boids, i, mouseAvoid);
                BoidNormalSpeed(boid);

                boid->visualSpeed.x = boid->visualSpeed.x*0.95f + boid->speed.x*0.05f;
                boid->visualSpeed.y = boid->visualSpeed.y*0.95f + boid->speed.y*0.05f;
                // boid->direction = atan2f(boid->visualSpeed.x, -boid->visualSpeed.y);

                boid->pos = Vector2Add(boid->pos, boid->speed);
            }
        }
        
        BeginDrawing();
            ClearBackground(RAYWHITE);

            for (size_t i = 0; i < BOIDS_COUNT; i++) {
                Boid *boid = &boids[i];
                DrawBoid(boid);
                if (showVisibleCircles)
                    DrawCircleV(boid->pos, BOID_VISIBLE_RADIUS, (Color){0, 121, 241, 10}); // Draw boid's visible area
                // DrawText(TextFormat("%d", i), boid->pos.x, boid->pos.y, 20, BLACK);
            }

            // for (size_t i = 0; i < BOIDS_COUNT; i++) {
            //     Boid *boid = &boids[i];
            //     DrawText(TextFormat("%d: %.3f %.3f %.3f", i, boid->speed.x, boid->speed.y, Vector2Length(boid->speed)), 10, 10 + i*30, 20, BLACK);
            // }

            if (mouseAvoid)
                DrawCircleV(GetMousePosition(), BOID_MOUSE_AVOID_RADIUS, (Color){230, 41, 55, 10});

            if (pause)
                DrawText("Paused", GetScreenWidth() - 85, 10, 20, BLACK);

            DrawFPS(10, 10);
        EndDrawing();
    }

    RL_FREE(boids);
    CloseWindow();
    
    return 0;
}
