#pragma once
#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <random>
#include <vector>

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
    for(auto i = 1; i < point_num; i++){
        result[i] = Vector2Add(line.from, Vector2Multiply(vec, {(float)i, (float)i}));
    }
    return result;
}

void move_points(std::vector<Vector2> &points, Matrix mat){
    for(Vector2 &p : points){
        p = Vector2Transform(p, mat);
    }
}


void draw_points_scale(std::vector<Vector2> &points, float scale, Vector2 origin={0,0}, Color color=GRAY){
    for(Vector2 p : points){
        DrawCircleV(Vector2Add(Vector2Multiply(p, {scale, scale}), origin), 4.0f, color);
    }
}
void draw_line_scale(Line line, float scale, Vector2 origin={0,0}){
        DrawSplineSegmentLinear(
            Vector2Add(Vector2Multiply(line.from, {scale, scale}), origin),
            Vector2Add(Vector2Multiply(line.to, {scale, scale}), origin),
            4.0f, GRAY);
}
void draw_lines_scale(std::vector<Line> &lines, float scale, Vector2 origin={0,0}){
    for(Line l : lines){
        draw_line_scale(l, scale, origin);
    }
}

Matrix tf2d_from_vec3(Vector3 vec){
    Matrix result = MatrixRotateZ(vec.z);
    result = MatrixMultiply(MatrixTranslate(vec.x, vec.y, 0), result);
    return result;
}

void noise_points(std::vector<Vector2> &points, float noise){
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::normal_distribution<float> dist(0, noise);
    for(Vector2 &p : points){
        p = Vector2Add(p, {dist(engine), dist(engine)});
    }
}

float distance_point_to_line(Vector2 point, Line line){
    if(Vector2DotProduct(Vector2Subtract(line.to, line.from), Vector2Subtract(point, line.from)) < 0) return Vector2Distance(point, line.from);
    else if(Vector2DotProduct(Vector2Subtract(line.from, line.to), Vector2Subtract(point, line.to)) < 0) return Vector2Distance(point, line.to);
    else {
        float len_projection = Vector2DotProduct(Vector2Normalize(Vector2Subtract(line.to, line.from)),Vector2Subtract(point, line.from));
        return Vector2Length(Vector2Subtract(Vector2Subtract(point, line.from),Vector2Multiply(Vector2Normalize(Vector2Subtract(line.to, line.from)), {len_projection, len_projection})));
    }
}

float ave_distance_points_to_line(std::vector<Vector2> &points, Line line){
    float sum = 0;
    for(Vector2 p : points){
        sum += distance_point_to_line(p, line);
    }
    return sum / points.size();
}
float ave_distance_points_to_lines(std::vector<Vector2> &points, std::vector<Line> lines){
    float sum = 0;
    for(Vector2 p : points){
        float min_distance = INFINITY;
        for(Line line : lines){
            float distance = distance_point_to_line(p, line);
            if(distance < min_distance) min_distance = distance;
        }
        sum += min_distance;
    }
    return sum / lines.size();
}
std::vector<Vector2> Vector2Transform(std::vector<Vector2> &points, Matrix mat){
    std::vector<Vector2> result;
    result.resize(points.size());
    for(unsigned int i = 0; i < points.size(); i++){
        result[i] = Vector2Transform(points[i], mat);
    }
    return result;
}

Vector3 grad_ave_distance_points_to_line(std::vector<Vector2> &points, Line line, float delta){
    auto p_dx = Vector2Transform(points, tf2d_from_vec3({delta/2,0,0}));
    auto p_m_dx = Vector2Transform(points, tf2d_from_vec3({-delta/2,0,0}));
    auto p_dy = Vector2Transform(points, tf2d_from_vec3({0,delta/2,0}));
    auto p_m_dy = Vector2Transform(points, tf2d_from_vec3({0,-delta/2,0}));
    auto p_dz = Vector2Transform(points, tf2d_from_vec3({0,0,delta/2}));
    auto p_m_dz = Vector2Transform(points, tf2d_from_vec3({0,0,-delta/2}));
    float dx = ave_distance_points_to_line(p_dx, line) - ave_distance_points_to_line(p_m_dx, line);
    float dy = ave_distance_points_to_line(p_dy, line) - ave_distance_points_to_line(p_m_dy, line);
    float dz = ave_distance_points_to_line(p_dz, line) - ave_distance_points_to_line(p_m_dz, line);
    return {dx, dy, dz};
}

Vector3 grad_ave_distance_points_to_lines(std::vector<Vector2> &points, std::vector<Line> lines, float delta){
    auto p_dx = Vector2Transform(points, tf2d_from_vec3({delta/2,0,0}));
    auto p_m_dx = Vector2Transform(points, tf2d_from_vec3({-delta/2,0,0}));
    auto p_dy = Vector2Transform(points, tf2d_from_vec3({0,delta/2,0}));
    auto p_m_dy = Vector2Transform(points, tf2d_from_vec3({0,-delta/2,0}));
    auto p_dz = Vector2Transform(points, tf2d_from_vec3({0,0,delta/2}));
    auto p_m_dz = Vector2Transform(points, tf2d_from_vec3({0,0,-delta/2}));
    float dx = ave_distance_points_to_lines(p_dx, lines) - ave_distance_points_to_lines(p_m_dx, lines);
    float dy = ave_distance_points_to_lines(p_dy, lines) - ave_distance_points_to_lines(p_m_dy, lines);
    float dz = ave_distance_points_to_lines(p_dz, lines) - ave_distance_points_to_lines(p_m_dz, lines);
    return {dx, dy, dz};
}