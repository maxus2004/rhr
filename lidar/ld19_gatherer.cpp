// File: ld19_gatherer.cpp
// Implementation for ld19_gatherer.hpp

#include "ld19_gatherer.hpp"

#include <vector>
#include <deque>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <optional>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <iostream>

namespace ld19 {

// ---- LD19 frame / CRC constants (private to cpp) ----
static constexpr uint8_t HEADER = 0x54;
static constexpr uint8_t VERLEN = 0x2C;
static constexpr size_t FRAME_LEN = 47;
static constexpr int POINTS_PER_PACK = 12;
static constexpr int BYTES_PER_POINT = 3;

// LD19 CRC8 table
static const uint8_t LD19_CRC_TABLE[256] = {
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

static inline uint8_t crc8_ld19(const std::vector<uint8_t>& data){
    uint8_t crc = 0;
    for(auto b : data) crc = LD19_CRC_TABLE[(crc ^ b) & 0xFF];
    return crc;
}

static inline bool checksum_ok_ld19(const std::vector<uint8_t>& frame){
    if(frame.size() < 1) return false;
    std::vector<uint8_t> body(frame.begin(), frame.end() - 1);
    uint8_t tail = frame.back();
    return crc8_ld19(body) == tail;
}

static inline uint16_t le16(const uint8_t* p){ return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }

// Minimal POSIX serial port helper
class SerialPort {
public:
    SerialPort(): fd_(-1), timeout_s_(0.02){}
    ~SerialPort(){ closePort(); }
    bool openPort(const std::string& path, int baud, double timeout_s){
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(fd_ < 0){ perror("open"); return false; }
        struct termios tio; memset(&tio, 0, sizeof(tio));
        if(tcgetattr(fd_, &tio) != 0){ perror("tcgetattr"); close(fd_); fd_=-1; return false; }
        cfmakeraw(&tio);
        speed_t speed = B115200;
        switch(baud){
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B115200; break;
        }
        cfsetspeed(&tio, speed);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if(tcsetattr(fd_, TCSANOW, &tio) != 0){ perror("tcsetattr"); close(fd_); fd_=-1; return false; }
        timeout_s_ = timeout_s;
        return true;
    }
    void closePort(){ if(fd_ >= 0){ close(fd_); fd_ = -1; } }
    ssize_t readToBuffer(std::vector<uint8_t>& buf, size_t max_bytes){
        if(fd_ < 0) return -1;
        fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
        struct timeval tv; tv.tv_sec = static_cast<long>(timeout_s_);
        tv.tv_usec = static_cast<long>((timeout_s_ - floor(timeout_s_)) * 1e6);
        int rv = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if(rv <= 0) return 0;
        size_t toread = std::min<size_t>(max_bytes, 512);
        std::vector<uint8_t> tmp(toread);
        ssize_t r = ::read(fd_, tmp.data(), toread);
        if(r > 0){ buf.insert(buf.end(), tmp.begin(), tmp.begin() + r); return r; }
        return r;
    }
    bool readExact(uint8_t* out, size_t n){
        if(fd_ < 0) return false;
        size_t got = 0;
        auto start = std::chrono::steady_clock::now();
        while(got < n){
            fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
            struct timeval tv; tv.tv_sec = static_cast<long>(timeout_s_);
            tv.tv_usec = static_cast<long>((timeout_s_ - floor(timeout_s_)) * 1e6);
            int rv = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if(rv <= 0) return false;
            ssize_t r = ::read(fd_, out + got, n - got);
            if(r <= 0) return false;
            got += r;
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration<double>(now - start).count() > timeout_s_ * 5.0) return false;
        }
        return true;
    }
private:
    int fd_;
    double timeout_s_;
};

// Parsed structures
struct Point { double ang_deg; double r_m; uint8_t inten; };
struct ParsedFrame { uint16_t speed; double start_deg; double end_deg; uint16_t timestamp_ms; std::vector<Point> points; };

static std::optional<ParsedFrame> parse_frame_ld19(const std::vector<uint8_t>& frame){
    if(frame.size() != FRAME_LEN) return std::nullopt;
    if(frame[0] != HEADER || frame[1] != VERLEN) return std::nullopt;
    uint16_t speed = le16(&frame[2]);
    uint16_t start_angle = le16(&frame[4]);
    std::vector<std::pair<uint16_t,uint8_t>> pts;
    size_t off = 6;
    for(int i = 0; i < POINTS_PER_PACK; ++i){
        if(off + BYTES_PER_POINT > frame.size()) break;
        uint16_t dist_mm = le16(&frame[off]);
        uint8_t inten = frame[off + 2];
        pts.emplace_back(dist_mm, inten);
        off += BYTES_PER_POINT;
    }
    uint16_t end_angle = 0, ts_ms = 0;
    if(42 + 4 <= frame.size()){
        end_angle = le16(&frame[42]);
        ts_ms = le16(&frame[44]);
    }
    double start_deg = double(start_angle % 36000) / 100.0;
    double end_deg = double(end_angle % 36000) / 100.0;
    double angle_diff = fmod((end_deg - start_deg) + 360.0, 360.0);
    double step = (POINTS_PER_PACK > 1) ? (angle_diff / (POINTS_PER_PACK - 1)) : 0.0;
    std::vector<Point> decoded;
    for(size_t i = 0; i < pts.size(); ++i){
        double ang_deg = fmod(start_deg + double(i) * step, 360.0);
        uint16_t dist_mm = pts[i].first;
        uint8_t inten = pts[i].second;
        if(dist_mm == 0 || dist_mm == 0xFFFF) continue;
        decoded.push_back({ang_deg, double(dist_mm) / 1000.0, inten});
    }
    return ParsedFrame{speed, start_deg, end_deg, ts_ms, decoded};
}

// buffered search for LD19 frames in stream
static std::optional<std::vector<uint8_t>> sync_and_read_frame_buffered_ld19(SerialPort& ser, double timeout_s){
    auto start = std::chrono::steady_clock::now();
    std::vector<uint8_t> buf;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < timeout_s){
        ssize_t r = ser.readToBuffer(buf, 128);
        if(r < 0) return std::nullopt;
        if(r == 0) continue;
        size_t i = 0;
        while(i + 1 <= buf.size() - 2){
            if(buf[i] == HEADER && buf[i+1] == VERLEN){
                if(buf.size() - i < FRAME_LEN) break;
                std::vector<uint8_t> frame(buf.begin() + i, buf.begin() + i + FRAME_LEN);
                if(checksum_ok_ld19(frame)){
                    buf.erase(buf.begin(), buf.begin() + i + FRAME_LEN);
                    return frame;
                }
                ++i;
                continue;
            }
            ++i;
        }
        if(buf.size() > 4096){ buf.erase(buf.begin(), buf.begin() + 2048); }
    }
    return std::nullopt;
}

// Implementation details hidden in Impl
struct LD19Gatherer::Impl {
    Impl(const std::string& port, int baud, size_t bins, uint32_t recent_window_ms, double ser_timeout_s)
    : port(port), baud(baud), bins(bins), recent_window_ms(recent_window_ms), ser_timeout_s(ser_timeout_s), running(false)
    {
        current_ranges.assign(bins, 0.0f);
        current_intens.assign(bins, 0);
        last_update_ts.assign(bins, 0);
    }

    ~Impl(){ stop(); }

    bool start(){
        if(running.load()) return true;
        if(!ser.openPort(port, baud, ser_timeout_s)) return false;
        running.store(true);
        worker = std::thread(&Impl::workerLoop, this);
        return true;
    }

    void stop(){
        if(!running.load()) return;
        running.store(false);
        if(worker.joinable()) worker.join();
        ser.closePort();
    }

    void getCurrentScanCopy(std::vector<float>& ranges_m, std::vector<uint8_t>& intens){
        std::lock_guard<std::mutex> lk(mtx);
        ranges_m = current_ranges;
        intens = current_intens;
    }

    void getRecentUpdates(std::vector<RecentPoint>& out){
        std::lock_guard<std::mutex> lk(mtx);
        out.clear();
        uint64_t now_ms = timestamp_ms_now();
        for(const auto &ri : recent_deque){
            uint64_t age = (now_ms > ri.ts_ms) ? (now_ms - ri.ts_ms) : 0;
            if(age <= recent_window_ms){
                out.push_back({ri.angle_deg, ri.range_m, ri.intensity, static_cast<uint32_t>(age)});
            }
        }
    }

    size_t angle_to_index(double ang_deg){
        double a = fmod(ang_deg, 360.0);
        if(a < 0) a += 360.0;
        double step = 360.0 / double(bins);
        size_t idx = static_cast<size_t>(std::round(a / step)) % bins;
        return idx;
    }

    void purge_old_recent_locked(uint64_t now_ms){
        while(!recent_deque.empty()){
            uint64_t age = (now_ms > recent_deque.front().ts_ms) ? (now_ms - recent_deque.front().ts_ms) : 0;
            if(age > recent_window_ms) recent_deque.pop_front(); else break;
        }
    }

    void workerLoop(){
        while(running.load()){
            auto frame_raw = sync_and_read_frame_buffered_ld19(ser, ser_timeout_s);
            if(!frame_raw.has_value()) continue;
            if(!checksum_ok_ld19(frame_raw.value())) continue;
            auto parsed = parse_frame_ld19(frame_raw.value());
            if(!parsed.has_value()) continue;
            auto &pf = parsed.value();
            uint64_t now_ms = timestamp_ms_now();
            std::vector<std::pair<size_t, RecentInternal>> local_updates;
            for(const auto &p : pf.points){
                size_t idx = angle_to_index(p.ang_deg);
                RecentInternal ri{static_cast<float>(p.ang_deg), static_cast<float>(p.r_m), p.inten, now_ms};
                local_updates.emplace_back(idx, ri);
            }
            {
                std::lock_guard<std::mutex> lk(mtx);
                for(auto &u : local_updates){
                    size_t idx = u.first; const RecentInternal &ri = u.second;
                    current_ranges[idx] = ri.range_m;
                    current_intens[idx] = ri.intensity;
                    last_update_ts[idx] = ri.ts_ms;
                    recent_deque.push_back(ri);
                }
                purge_old_recent_locked(now_ms);
                const size_t MAX_RECENT = std::max<size_t>(10000, bins * 4);
                while(recent_deque.size() > MAX_RECENT) recent_deque.pop_front();
            }
        }
    }

    // utility
    static inline uint64_t timestamp_ms_now(){
        using namespace std::chrono;
        return (uint64_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    struct RecentInternal { float angle_deg; float range_m; uint8_t intensity; uint64_t ts_ms; };

    // configuration
    std::string port;
    int baud;
    size_t bins;
    uint32_t recent_window_ms;
    double ser_timeout_s;

    // serial and thread
    SerialPort ser;
    std::atomic<bool> running;
    std::thread worker;

    // data (protected by mtx)
    std::vector<float> current_ranges;
    std::vector<uint8_t> current_intens;
    std::vector<uint64_t> last_update_ts;
    std::deque<RecentInternal> recent_deque;

    std::mutex mtx;
};

// LD19Gatherer public methods
LD19Gatherer::LD19Gatherer(const std::string& port, int baud, size_t bins, uint32_t recent_window_ms, double ser_timeout_s)
: impl_(new Impl(port, baud, bins, recent_window_ms, ser_timeout_s)){}

LD19Gatherer::~LD19Gatherer(){ stop(); }

bool LD19Gatherer::start(){ return impl_ ? impl_->start() : false; }

void LD19Gatherer::stop(){ if(impl_) impl_->stop(); }

void LD19Gatherer::getCurrentScanCopy(std::vector<float>& ranges_m, std::vector<uint8_t>& intens){ if(impl_) impl_->getCurrentScanCopy(ranges_m, intens); }

void LD19Gatherer::getRecentUpdates(std::vector<RecentPoint>& out){ if(impl_) impl_->getRecentUpdates(out); }

size_t LD19Gatherer::bins() const { return impl_ ? impl_->bins : 0; }

} // namespace ld19
