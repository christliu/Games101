#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle);

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
   // Eigen::AngleAxisd rotate(MY_PI * rotation_angle / 180, Eigen::Vector3f(0, 0, 1));
   // model = rotate.matrix();
    float angle = MY_PI * rotation_angle / 180;
    model << cos(angle), -sin(angle), 0, 0, 
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Vector3f axis = {1, 0, 0};
    return get_rotation(axis, rotation_angle);
   // return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    zNear = -zNear;
    zFar = -zFar;
    float height = 2 * tan(eye_fov / 2 * MY_PI / 180) * (-zNear);
    float width = aspect_ratio * height;
    Eigen::Matrix4f orth_scale, orth_translate;

    std::cout << "height " << height << " width " << width << std::endl;
    orth_scale << 2.0 / width, 0, 0, 0,
        0, 2.0/height, 0, 0,
        0, 0, 2.0/(zNear - zFar), 0,
        0,0,0,1;
    orth_translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -(zNear + zFar) / 2.0,
        0, 0, 0, 1;

    Eigen::Matrix4f Orth;
    Orth << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zFar + zNear, -zNear*zFar,
        0, 0, 1, 0;
    //projection = orth_scale * orth_translate;
    projection = orth_scale * orth_translate * Orth;

    return projection;
}


// Eigen::Matrix4f frustum(float l, float r, float b, float t, float n, float f)
// {
//     Eigen::Matrix4f mat;
//     mat << 2.0f * n / (r-l), 0, 0, 0,
//         0, 2.0f * n / (t - b), 0, 0,
//         (r+l)/(r-l), (t+b)/(t-b), -(f+n)/(f-n), -1.0f,
//         0, 0, -2.0 * (f * n) / (f - n), 0;
//     return mat;
// }

// Eigen::Matrix4f get_projection_matrix2(float eye_fov, float aspect_ratio,
//                                       float zNear, float zFar)
// {
//     float height = zNear * tan(eye_fov * MY_PI / 360.0);
//     float width = height * aspect_ratio;
//     return frustum(-width, width, -height, height, zNear, zFar);
// }


Eigen::Matrix4f get_projection_matrix(float l, float r, float b, float t, float near, float far)
{
    Eigen::Matrix4f orh_projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orth_translate, orth_scale;
    orth_translate << 1, 0, 0, -(r + l) / 2.0,
        0, 1, 0, -(b + t) / 2.0,
        0, 0, 1, -(near + far) / 2.0,
        0, 0, 0, 1;
    orth_scale << 2.0 / (r - l), 0, 0, 0,
        0, 2.0 / (t - b), 0, 0,
        0, 0, 2.0 / (far - near), 0,
        0, 0, 0, 1;
    orh_projection = orth_scale * orth_translate;
    return orh_projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f rotateWIthZ;
    angle = angle * MY_PI / 360;
    rotateWIthZ << cos(angle), -sin(angle), 0, 0,
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Vector3f newX = {axis.y(), -axis.x(), 0};
    if(newX.x() == 0 && newX.y() == 0)
    {
        // axis same with z.
        return rotateWIthZ;
    }
    Eigen::Vector3f newY = {axis.z() * axis.x(), axis.y() * axis.z(), -axis.x() * axis.x() - axis.y() * axis.y()};

    Eigen::Matrix4f rotateToZ;
    rotateToZ << newX.x(), newY.x(), axis.x(), 0,
        newX.y(), newY.y(), axis.y(), 0,
        newX.z(), newY.z(), axis.z(), 0,
        0, 0, 0, 1;

    return rotateToZ.inverse() * rotateWIthZ * rotateToZ;

}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    bool isOrth = false;
    std::cout << "argc " << argc << std::endl;
    if(argc == 2)
    {
        if(std::stof(argv[1]) == 1)
            isOrth = true;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        if(isOrth)
            r.set_projection(get_projection_matrix(-5, 5, -5, 5, 0.1, 50));
        else
            r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        if(isOrth)
            r.set_projection(get_projection_matrix(-5, 5, -5, 5, 0.1, 50));
        else
            r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
