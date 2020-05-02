// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>

bool needprint =  false;
bool bSecond = false;


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// if v0v1 cross v0p is the same side of v1p cross v1v2
static bool sameSide(Vector3f p, Vector3f v0, Vector3f v1, Vector3f v2)
{
    Vector3f v0v1 = v1 - v0;
    Vector3f v0p = p - v0;
    Vector3f cross1 = v0v1.cross(v0p);
    Vector3f v1p = p - v1;
    Vector3f v1v2 = v2 - v1;
    Vector3f cross2 = v1v2.cross(v1p);
    return cross1.dot(cross2) >= 0;
}


static bool coordInsideTriangle(float x, float y, const Eigen::Vector3f* _v)
{
    Vector3f p{x + 0.5, y + 0.5, 0};
    Vector3f v0 = {_v[0].x(), _v[0].y(), 0};
    Vector3f v1 = {_v[1].x(), _v[1].y(), 0};
    Vector3f v2 = {_v[2].x(), _v[2].y(), 0};
    return sameSide(p, v0, v1, v2) && sameSide(p, v1, v2, v0) && sameSide(p, v2, v0, v1);
}


static bool insideTriangle(int x, int y, const Eigen::Vector3f* _v)
{  
    return coordInsideTriangle(float(x), float(y), _v); 
    // Vector3f p{x + 0.5, y + 0.5, 0};
    // Vector3f v0 = {_v[0].x(), _v[0].y(), 0};
    // Vector3f v1 = {_v[1].x(), _v[1].y(), 0};
    // Vector3f v2 = {_v[2].x(), _v[2].y(), 0};
    // return sameSide(p, v0, v1, v2) && sameSide(p, v1, v2, v0) && sameSide(p, v2, v0, v1);
}

static std::vector<bool> subInsideTriangle(int x, int y, const Eigen::Vector3f* _v)
{
    return std::vector{coordInsideTriangle(x + 0.25, y+0.75, _v), coordInsideTriangle(x+0.75, y+0.75, _v),
        coordInsideTriangle(x+0.25, y+0.25, _v), coordInsideTriangle(x+0.75, y+0.25, _v)};
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;

    bSecond = false;
    for (auto& i : ind)
    {
        
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        //rasterize_triangle(t);
        rasterize_triangle_msaa(t);
        bSecond = true;
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float minx = std::min(t.v[0].x(), std::min(t.v[1].x(), t.v[2].x()));
    float maxx = std::max(t.v[0].x(), std::max(t.v[1].x(), t.v[2].x()));
    float miny = std::min(t.v[0].y(), std::min(t.v[1].y(), t.v[2].y()));
    float maxy = std::max(t.v[0].y(), std::max(t.v[1].y(), t.v[2].y()));

    // std::cout << "Triangle v0 :" << t.v[0] << std::endl;
    // std::cout << "Triangle v1 :" << t.v[1] << std::endl;
    // std::cout << "Triangle v2 :" << t.v[2] << std::endl;
    bool first = true;
    for(int x = int(minx); x <= int(maxx) + 1; x++)
    {
        for(int y = int(miny); y <= int(maxy) + 1; y++)
        {
            // if(x == 350 && y == 500)
            //     needprint = true;
            // else
            //     needprint = false;


            if(insideTriangle(x, y, t.v))
            {
                // std::cout << "pixel x " << x << " y " << y << " is in triangle" <<std::endl;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(first)
                    std::cout << "z_interpolated " << z_interpolated << std::endl;
                first = false;
                auto ind = (height-1-y)*width + x;
                if(depth_buf[ind] > z_interpolated)
                {
                    depth_buf[ind] = z_interpolated;
                    set_pixel({x,y,0}, t.getColor());
                }
            }
        }
    }

}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle_msaa(const Triangle& t) {
    auto v = t.toVector4();
    float minx = std::min(t.v[0].x(), std::min(t.v[1].x(), t.v[2].x()));
    float maxx = std::max(t.v[0].x(), std::max(t.v[1].x(), t.v[2].x()));
    float miny = std::min(t.v[0].y(), std::min(t.v[1].y(), t.v[2].y()));
    float maxy = std::max(t.v[0].y(), std::max(t.v[1].y(), t.v[2].y()));

    // std::cout << "Triangle v0 :" << t.v[0] << std::endl;
    // std::cout << "Triangle v1 :" << t.v[1] << std::endl;
    // std::cout << "Triangle v2 :" << t.v[2] << std::endl;
    for(int x = int(minx); x <= int(maxx) + 1; x++)
    {
        for(int y = int(miny); y <= int(maxy) + 1; y++)
        {
            if(x == 350 && y == 392 && bSecond)
                needprint = true;
            else
                needprint = false;
            auto b_subPixelsInTriangle = subInsideTriangle(x, y, t.v);
            if(b_subPixelsInTriangle[0] || b_subPixelsInTriangle[1] || b_subPixelsInTriangle[2] || b_subPixelsInTriangle[3])
            {
                // any pixel inside trianngle
                Eigen::Vector3f color{0, 0, 0};
                for(int pixelindex = 0; pixelindex < 4; pixelindex++)
                {
                    if(b_subPixelsInTriangle[pixelindex])
                    {
                        int i_x = pixelindex % 2; // x
                        int i_y = pixelindex / 2; // y

                        //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        auto[alpha, beta, gamma] = computeBarycentric2D(x + i_x * 0.5, y + (1-i_y) * 0.5, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        auto ind = (height - 1 - y) * 4 * width + i_y * width * 2 + x * 2 + i_x;

                        if(needprint)
                        {
                            std::cout << "i_x " << i_x << " i_y " << i_y << std::endl;
                            std::cout << "ind " << ind << " depth_buf[ind] " << depth_buf[ind] << " z_interpolated " << z_interpolated << std::endl;
                        }
                        if(depth_buf[ind] > z_interpolated)
                        {
                            depth_buf[ind] = z_interpolated;
                            color += t.getColor();
                            if(needprint)
                            {
                                std::cout << "color is " << color << std::endl;
                            }
                        }


                    }
                }
                if(color.x() != 0 || color.y() != 0 || color.z() != 0)
                    set_pixel({x,y,0}, color / 4);
            }
        }
    }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on