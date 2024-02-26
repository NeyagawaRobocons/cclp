#include <rclcpp/rclcpp.hpp>
#include <raylib.h>
#include <raymath.h>
#include <iostream>


struct Line{
    Vector2 from;
    Vector2 to;
};

std::vector<Vector2> points_from_line(Line line, unsigned int point_num){
    if(point_num < 2) return std::vector<Vector2>{};
    std::vector<Vector2> result;
    result.resize(point_num);
    auto sub = Vector2Subtract(line.to, line.from);
    auto vec = Vector2Divide(sub, {(float)point_num-1, (float)point_num-1});
    result[0] = line.from;
    for(int i = 1; i < point_num; i++){
        result[i] = Vector2Add(line.from, Vector2Multiply(vec, {(float)i, (float)i}));
    }
    return result;
}

void move_points(std::vector<Vector2> &points, Matrix mat){
    for(Vector2 &p : points){
        p = Vector2Transform(p, mat);
    }
}

void draw_points(std::vector<Vector2> &points){
    for(Vector2 p : points){
        DrawCircleV(p, 4.0f, BLUE);
    }
}

void draw_points_scale(std::vector<Vector2> &points, float scale, Vector2 origin={0,0}){
    for(Vector2 p : points){
        DrawCircleV(Vector2Add(Vector2Multiply(p, {scale, scale}), origin), 4.0f, BLUE);
    }
}
void draw_line_scale(Line line, float scale, Vector2 origin={0,0}){
        DrawSplineSegmentLinear(
            Vector2Add(Vector2Multiply(line.from, {scale, scale}), origin),
            Vector2Add(Vector2Multiply(line.to, {scale, scale}), origin),
            4.0f, GRAY);
}

Matrix tf2d_from_vec3(Vector3 vec){
    Matrix result = MatrixRotateZ(vec.z);
    result = MatrixMultiply(MatrixTranslate(vec.x, vec.y, 0), result);
    return result;
}

int main()
{
    int screenWidth = 1280;
    int screenHeight = 720;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "cord correction display");
    SetTargetFPS(60);
    screenWidth = GetScreenWidth();
    screenHeight = GetScreenHeight();

    std::cout << "window start" << std::endl;

    Line line = {{0, 0}, {1, 1}};
    auto points = points_from_line(line, 10);
    auto tf = tf2d_from_vec3({0,0,PI/2});
    move_points(points, tf);
    std::cout << "size: " << points.size() << std::endl;
    for(auto p : points){
        std::cout << "x: " << p.x << ", y: " << p.y << std::endl;
    }

    while (!WindowShouldClose()){
        screenWidth = GetScreenWidth();
        screenHeight = GetScreenHeight();

        BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawSplineSegmentLinear(line.from, line.to, 4.0f, GRAY);
            draw_line_scale(line, 500, {(float)screenWidth/2, (float)screenHeight/2});
            draw_points_scale(points, 500, {(float)screenWidth/2, (float)screenHeight/2});
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
