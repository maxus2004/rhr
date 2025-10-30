// test_ld19_raylib.cpp
// Visualizer for ld19::LD19Gatherer with:
// - --port PATH
// - --baud N
// - --dots N
// - --print (enable periodic printing of full array)
// - --update-delay S (seconds between prints, default 1.0)
// - --window WxH (e.g. 1024x800)
// - --max-distance M (meters; draws & prints only <= M; used to compute zoom)
// - --min-distance M (meters; ignores points closer than this threshold)

#include "ld19_gatherer.hpp"
// #include "raylib.h"

#include <cmath>
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <mutex>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud = 230400;
    bool print_points = false;
    double update_delay_s = 1.0; // default: print once per second
    size_t dots = 3600; // default bins (3600 -> 0.1 deg resolution)
    int winw = 800, winh = 800;
    double max_distance_m = 6.0; // default max distance (meters)
    double min_distance_m = 0.0; // default min distance (meters) - new flag

    // Parse args
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = argv[++i];
        else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) baud = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--print") == 0) print_points = true;
        else if (std::strcmp(argv[i], "--update-delay") == 0 && i + 1 < argc) update_delay_s = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--dots") == 0 && i + 1 < argc) {
            unsigned long v = std::strtoul(argv[++i], nullptr, 10);
            if (v == 0) { std::cerr << "Invalid --dots value; must be > 0\n"; return 1; }
            dots = static_cast<size_t>(v);
        }
        else if (std::strcmp(argv[i], "--window") == 0 && i + 1 < argc) {
            std::string w = argv[++i];
            auto x = w.find('x');
            if (x != std::string::npos) {
                winw = std::atoi(w.substr(0, x).c_str());
                winh = std::atoi(w.substr(x + 1).c_str());
                if (winw <= 0) winw = 800;
                if (winh <= 0) winh = 800;
            } else {
                std::cerr << "Invalid --window format, expected WxH (e.g. 1024x800)\n";
                return 1;
            }
        }
        else if (std::strcmp(argv[i], "--max-distance") == 0 && i + 1 < argc) {
            max_distance_m = std::atof(argv[++i]);
            if (!(max_distance_m > 0.0)) { std::cerr << "Invalid --max-distance, must be > 0\n"; return 1; }
        }
        else if (std::strcmp(argv[i], "--min-distance") == 0 && i + 1 < argc) {
            min_distance_m = std::atof(argv[++i]);
            if (min_distance_m < 0.0) { std::cerr << "Invalid --min-distance, must be >= 0\n"; return 1; }
        }
        else {
            std::cerr << "Unknown arg: " << argv[i] << "\n";
        }
    }

    if (update_delay_s <= 0.0) update_delay_s = 0.01;
    if (min_distance_m >= max_distance_m) {
        std::cerr << "Warning: --min-distance >= --max-distance; no points will be shown.\n";
    }

    // Prepare gatherer
    ld19::LD19Gatherer g(port, baud, dots);
    std::cerr << "Starting LD19 gatherer on " << port << " @ " << baud << " with " << dots << " bins...\n";
    if (!g.start()) {
        std::cerr << "Failed to open serial port " << port << "\n";
        return 1;
    }

    // Compute pixels-per-meter so max_distance_m fits in the window radius with a small margin
    const float margin = 8.0f;
    const float max_draw_r = std::min(winw, winh) / 2.0f - margin;
    float pixels_per_meter = (max_distance_m > 0.0) ? (max_draw_r / static_cast<float>(max_distance_m)) : (max_draw_r / 6.0f);
    if (pixels_per_meter <= 0.0f) pixels_per_meter = 1.0f;

    // Snapshot buffer published by printer thread and consumed by renderer.
    std::vector<ld19::LD19Gatherer::RecentPoint> display_points;
    std::mutex display_mtx;

    std::atomic<bool> running{ true };

    // Printer thread: prints full array every update_delay_s and publishes snapshot for drawing.
    std::thread printer_thread;
    if (print_points) {
        printer_thread = std::thread([&]() {
            using clock = std::chrono::steady_clock;
            auto next = clock::now() + std::chrono::duration<double>(update_delay_s);
            while (running.load()) {
                std::this_thread::sleep_until(next);
                next += std::chrono::duration<double>(update_delay_s);

                // Get full scan (may return smaller vectors in some conditions)
                std::vector<float> ranges;
                std::vector<uint8_t> intens;
                g.getCurrentScanCopy(ranges, intens);

                // Prepare snapshot points for rendering (only points inside [min,max] are kept)
                std::vector<ld19::LD19Gatherer::RecentPoint> snapshot_pts;
                snapshot_pts.reserve(dots);

                // Print header with timestamp + parameters
                auto t = std::chrono::system_clock::now();
                std::time_t tt = std::chrono::system_clock::to_time_t(t);
                std::tm tm = *std::localtime(&tt);
                char timebuf[64];
                std::strftime(timebuf, sizeof(timebuf), "%F %T", &tm);
                std::cout << "=== LD19 Scan @ " << timebuf
                          << " (requested_bins=" << dots
                          << ", returned_ranges=" << ranges.size()
                          << ", returned_intens=" << intens.size()
                          << ", min_distance=" << min_distance_m
                          << ", max_distance=" << max_distance_m << ") ===\n";
                std::cout << std::fixed << std::setprecision(6);

                // Print exactly 'dots' lines; treat missing entries or out-of-range as zero.
                for (size_t i = 0; i < dots; ++i) {
                    double angle = 360.0 * double(i) / double(dots);
                    float range_val = 0.0f;
                    uint8_t inten_val = 0;
                    if (i < ranges.size()) range_val = ranges[i];
                    if (i < intens.size()) inten_val = intens[i];

                    // Apply min/max filters: if outside range, treat as zero
                    if (!(range_val >= static_cast<float>(min_distance_m) && range_val <= static_cast<float>(max_distance_m))) {
                        range_val = 0.0f;
                        inten_val = 0;
                    }

                    std::cout << "bin[" << i << "] angle=" << angle << " deg"
                              << " range=" << range_val << " m"
                              << " inten=" << int(inten_val) << "\n";

                    if (range_val > 0.0f) {
                        ld19::LD19Gatherer::RecentPoint rp;
                        rp.angle_deg = static_cast<float>(angle);
                        rp.range_m = range_val;
                        rp.intensity = inten_val;
                        rp.age_ms = 0;
                        snapshot_pts.push_back(rp);
                    }
                }

                std::cout << "=== end scan ===\n" << std::flush;

                // Publish snapshot for rendering (fast swap under lock)
                {
                    std::lock_guard<std::mutex> lk(display_mtx);
                    display_points.swap(snapshot_pts);
                }
            }
        });
    }

    while(true){
        std::this_thread::yield();
    }

    // // Renderer (main thread): init window and run Raylib loop
    // InitWindow(winw, winh, "LD19 Visualizer");
    // SetTargetFPS(60);

    // // Main render loop: draws either display_points (if printing enabled) or live recent updates
    // while (!WindowShouldClose()) {
    //     BeginDrawing();
    //     ClearBackground(BLACK);

    //     Vector2 center = { winw / 2.0f, winh / 2.0f };

    //     if (print_points) {
    //         std::vector<ld19::LD19Gatherer::RecentPoint> pts_copy;
    //         {
    //             std::lock_guard<std::mutex> lk(display_mtx);
    //             pts_copy = display_points; // small copy
    //         }
    //         for (const auto &p : pts_copy) {
    //             if (p.range_m <= 0.0f) continue;
    //             if (p.range_m < static_cast<float>(min_distance_m) || p.range_m > static_cast<float>(max_distance_m)) continue;
    //             float angle_rad = p.angle_deg * (float)(M_PI / 180.0);
    //             float x = center.x + sinf(angle_rad) * p.range_m * pixels_per_meter;
    //             float y = center.y - cosf(angle_rad) * p.range_m * pixels_per_meter;
    //             Color c = {
    //                 static_cast<unsigned char>(255),
    //                 static_cast<unsigned char>(std::max(0, 255 - int(p.intensity))),
    //                 static_cast<unsigned char>(0),
    //                 static_cast<unsigned char>(255)
    //             };
    //             DrawPixelV({ x, y }, c);
    //         }
    //     } else {
    //         std::vector<ld19::LD19Gatherer::RecentPoint> live_pts;
    //         g.getRecentUpdates(live_pts);
    //         for (const auto &p : live_pts) {
    //             if (p.range_m <= 0.0f) continue;
    //             if (p.range_m < static_cast<float>(min_distance_m) || p.range_m > static_cast<float>(max_distance_m)) continue;
    //             float angle_rad = p.angle_deg * (float)(M_PI / 180.0);
    //             float x = center.x + sinf(angle_rad) * p.range_m * pixels_per_meter;
    //             float y = center.y - cosf(angle_rad) * p.range_m * pixels_per_meter;
    //             Color c = {
    //                 static_cast<unsigned char>(255),
    //                 static_cast<unsigned char>(std::max(0, 255 - int(p.intensity))),
    //                 static_cast<unsigned char>(0),
    //                 static_cast<unsigned char>(255)
    //             };
    //             DrawPixelV({ x, y }, c);
    //         }
    //     }

    //     EndDrawing();
    // }

    // Shutdown
    running.store(false);
    if (printer_thread.joinable()) printer_thread.join();

    g.stop();
    // CloseWindow();

    std::cerr << "Visualizer closed. Lidar stopped.\n";
    return 0;
}