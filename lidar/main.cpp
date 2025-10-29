// test_ld19_raylib.cpp
#include "ld19_gatherer.hpp"
#include "raylib.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <mutex>

// meters -> pixels
static float RANGE_SCALE = 200.0f;

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud = 230400;
    bool print_points = false;
    double update_delay_s = 1.0; // default: print once per second
    size_t dots = 3600; // default bins (3600 -> 0.1 deg resolution)

    // Parse args
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = argv[++i];
        else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) baud = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--print") == 0) print_points = true;
        else if (std::strcmp(argv[i], "--update-delay") == 0 && i + 1 < argc) update_delay_s = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--dots") == 0 && i + 1 < argc) {
            unsigned long v = std::strtoul(argv[++i], nullptr, 10);
            if (v == 0) {
                std::cerr << "Invalid --dots value; must be > 0\n";
                return 1;
            }
            dots = static_cast<size_t>(v);
        }
        else std::cerr << "Unknown arg: " << argv[i] << "\n";
    }

    if (update_delay_s <= 0.0) update_delay_s = 0.01; // clamp to small positive

    // Construct gatherer with requested number of angular bins
    ld19::LD19Gatherer g(port, baud, dots);
    std::cout << "Starting LD19 gatherer on " << port << " @ " << baud << " with " << dots << " bins...\n";
    if (!g.start()) {
        std::cerr << "Failed to open serial port " << port << "\n";
        return 1;
    }

    const int screenWidth = 800;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "LD19 Visualizer");
    SetTargetFPS(60);

    std::cout << "Press ESC in the window to exit.\n";
    if (print_points) {
        std::cout << "[Printing enabled: outputting full array of " << dots << " bins every " << update_delay_s << " s]\n";
    }

    // For printing snapshots -> these are the points we will draw when --print is enabled.
    std::vector<ld19::LD19Gatherer::RecentPoint> display_points;
    std::mutex display_mtx;

    // timer for printing
    auto last_print = std::chrono::steady_clock::now();

    while (!WindowShouldClose()) {
        // If not printing, draw live recent updates; if printing, draw the last printed snapshot.
        std::vector<ld19::LD19Gatherer::RecentPoint> live_pts;
        if (!print_points) {
            // fetch recent updates for live drawing
            g.getRecentUpdates(live_pts);
        }

        // maybe time to print the whole current scan and capture it for display
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last_print;
        if (print_points && dt.count() >= update_delay_s) {
            last_print = now;

            std::vector<float> ranges;
            std::vector<uint8_t> intens;
            g.getCurrentScanCopy(ranges, intens);
            size_t bins = ranges.size();

            // print header with timestamp
            auto t = std::chrono::system_clock::now();
            std::time_t tt = std::chrono::system_clock::to_time_t(t);
            std::tm tm = *std::localtime(&tt);
            char timebuf[64];
            std::strftime(timebuf, sizeof(timebuf), "%F %T", &tm);
            std::cout << "=== LD19 Scan @ " << timebuf << " (bins=" << bins << ") ===\n";

            // print every bin (including zero entries) so output has exactly `dots` lines
            std::cout << std::fixed << std::setprecision(3);

            // prepare the snapshot we'll draw (only non-zero bins are turned into points)
            std::vector<ld19::LD19Gatherer::RecentPoint> snapshot_pts;
            snapshot_pts.reserve(bins);

            for (size_t i = 0; i < bins; ++i) {
                double angle = 360.0 * double(i) / double(bins);
                std::cout << "bin[" << i << "] angle=" << angle << " deg"
                          << " range=" << ranges[i] << " m"
                          << " inten=" << int(intens[i]) << "\n";
                if (ranges[i] > 0.0f) {
                    // create RecentPoint (age_ms unknown here, set 0)
                    ld19::LD19Gatherer::RecentPoint rp;
                    rp.angle_deg = static_cast<float>(angle);
                    rp.range_m = ranges[i];
                    rp.intensity = intens[i];
                    rp.age_ms = 0;
                    snapshot_pts.push_back(rp);
                }
            }
            std::cout << "=== end scan ===\n";

            // publish snapshot for drawing (thread-safe)
            {
                std::lock_guard<std::mutex> lk(display_mtx);
                display_points.swap(snapshot_pts);
            }
        }

        // render
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw origin and crosshair
        Vector2 center = { screenWidth / 2.0f, screenHeight / 2.0f };
        DrawCircleLines(center.x, center.y, 4, GREEN);
        DrawLine(center.x - 400, center.y, center.x + 400, center.y, DARKGRAY);
        DrawLine(center.x, center.y - 400, center.x, center.y + 400, DARKGRAY);

        if (print_points) {
            // draw the last printed snapshot (display_points)
            std::vector<ld19::LD19Gatherer::RecentPoint> pts_copy;
            {
                std::lock_guard<std::mutex> lk(display_mtx);
                pts_copy = display_points;
            }
            for (const auto &p : pts_copy) {
                if (p.range_m <= 0.01f) continue;
                float angle_rad = p.angle_deg * (PI / 180.0f);
                float x = center.x + sinf(angle_rad) * p.range_m * RANGE_SCALE;
                float y = center.y - cosf(angle_rad) * p.range_m * RANGE_SCALE;
                Color c = { static_cast<unsigned char>(255),
                            static_cast<unsigned char>(std::max(0, 255 - int(p.intensity))),
                            static_cast<unsigned char>(0),
                            static_cast<unsigned char>(255) };
                DrawPixelV({ x, y }, c);
            }
        } else {
            // draw live recent updates
            for (const auto &p : live_pts) {
                if (p.range_m <= 0.01f) continue;
                float angle_rad = p.angle_deg * (PI / 180.0f);
                float x = center.x + sinf(angle_rad) * p.range_m * RANGE_SCALE;
                float y = center.y - cosf(angle_rad) * p.range_m * RANGE_SCALE;
                Color c = { static_cast<unsigned char>(255),
                            static_cast<unsigned char>(std::max(0, 255 - int(p.intensity))),
                            static_cast<unsigned char>(0),
                            static_cast<unsigned char>(255) };
                DrawPixelV({ x, y }, c);
            }
        }

        // HUD
        DrawText("Press ESC to quit", 10, 10, 20, GRAY);
        if (print_points) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "Printing full array (%zu bins) every %.2f s", dots, update_delay_s);
            DrawText(buf, 10, 35, 20, GRAY);
        }

        EndDrawing();
    }

    g.stop();
    CloseWindow();
    std::cout << "Visualizer closed. Lidar stopped.\n";
    return 0;
}