//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u_idx = std::min(static_cast<int>(u_img), width - 1);
        int v_idx = std::min(static_cast<int>(v_img), height - 1);

        auto color = image_data.at<cv::Vec3b>(v_idx, u_idx);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getBiColor(float u, float v) {

        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

		const int offset[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

        cv::Vec3b color = cv::Vec3b();

        for (int i = 0; i < 4; i++) {
			auto u_offset = u_img + offset[i][0];
			auto v_offset = v_img + offset[i][1];
            
            u_offset = std::min(static_cast<int>(u_img), width - 1);
            v_offset = std::min(static_cast<int>(v_img), height - 1);
            u_offset = std::max(static_cast<int>(u_img), 0);
            v_offset = std::max(static_cast<int>(v_img), 0);

			color += image_data.at<cv::Vec3b>(v_offset, u_offset);
        }
        
        color = color / 4.0f;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
