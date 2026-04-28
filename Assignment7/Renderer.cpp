//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>
#include <atomic>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 32;
    std::cout << "SPP: " << spp << "\n";

    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0)
        num_threads = 4;
    std::cout << "Using " << num_threads << " threads.\n";

    std::atomic<int> next_row(0);
    std::mutex progress_mutex;

    auto worker = [&]() {
        while (true) {
            int j = next_row.fetch_add(1, std::memory_order_relaxed);
            if (j >= scene.height) break;

            for (uint32_t i = 0; i < scene.width; ++i) {
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));

                for (int k = 0; k < spp; k++) {
                    framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }

            std::lock_guard<std::mutex> lock(progress_mutex);
            UpdateProgress(j / (float)scene.height);
        }
    };

    std::vector<std::thread> threads;
    for (unsigned int t = 0; t < num_threads; t++) {
        threads.emplace_back(worker);
    }

    for (auto& t : threads) {
        t.join();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
