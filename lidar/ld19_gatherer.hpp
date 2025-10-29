// ld19_gatherer.hpp
// Public header for LD19 gatherer (implementation in ld19_gatherer.cpp)

#ifndef LD19_GATHERER_HPP
#define LD19_GATHERER_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <cstddef>
#include <memory>

namespace ld19 {

// Lightweight headless LD19 gatherer API (header)
class LD19Gatherer {
public:
    // A recent point (angle in degrees, range in meters, intensity 0-255, age in ms)
    struct RecentPoint { float angle_deg; float range_m; uint8_t intensity; uint32_t age_ms; };

    // ctor: serial port path, baud (default 230400), bins (angular bins, default 3600 -> 0.1deg),
    // recent window ms (default 200), serial timeout sec
    LD19Gatherer(const std::string& port, int baud = 230400, size_t bins = 3600, uint32_t recent_window_ms = 200, double ser_timeout_s = 0.02);

    // non-copyable
    LD19Gatherer(const LD19Gatherer&) = delete;
    LD19Gatherer& operator=(const LD19Gatherer&) = delete;

    ~LD19Gatherer();

    // start background reader thread; returns true if serial port opened
    bool start();

    // stop background thread and close port
    void stop();

    // thread-safe snapshot of current scan. ranges_m and intens will be resized to bins()
    void getCurrentScanCopy(std::vector<float>& ranges_m, std::vector<uint8_t>& intens);

    // thread-safe recent updates (points within recent_window_ms at call time)
    void getRecentUpdates(std::vector<RecentPoint>& out);

    // number of angular bins
    size_t bins() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_; // pimpl to hide implementation details
};

} // namespace ld19

#endif // LD19_GATHERER_HPP
