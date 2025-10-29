#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <thread>
#include <chrono>
#include "madgwick_filter.h"

#define SPI_SPEED 1000000

// LSM6DSO registerss
#define WHO_AM_I        0x0F
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define OUTX_L_G        0x22
#define OUTX_L_A        0x28
#define STATUS_REG      0x1E

float calibration_time = 20;

float gyro_scale = 0.00122173; // rad/s/LSB
float accel_scale = 0.000244;  // g/LSB
float dt = 1/1666.0;

float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float yaw = 0;

uint8_t SPI_write(int fd, uint8_t reg, uint8_t *data, size_t len) {
    struct spi_ioc_transfer tr = {};
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];

    tx[0] = reg;
    memcpy(&tx[1], data, len);

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len + 1;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return 0;
}

uint8_t SPI_read(int fd, uint8_t reg, uint8_t *data, size_t len) {
    struct spi_ioc_transfer tr = {};
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];

    tx[0] = reg | 0x80;

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len + 1;
    tr.speed_hz = SPI_SPEED;
    tr.bits_per_word = 8;

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    memcpy(data, &rx[1], len);
    return 0;
}

bool IMU_data_available(int fd){
    uint8_t reg_value;
    SPI_read(fd, STATUS_REG, &reg_value, 1);
    return reg_value & 0b00000010 != 0;
}

void IMU_wait_and_get_data(int fd, float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
    while(!IMU_data_available(fd)){
        std::this_thread::yield();
    }

    uint8_t buffer[12];
    SPI_read(fd, OUTX_L_G, buffer, 12);
    *gx = (int16_t)(buffer[1] << 8 | buffer[0])*gyro_scale+gx_offset;
    *gy = (int16_t)(buffer[3] << 8 | buffer[2])*gyro_scale+gy_offset;
    *gz = (int16_t)(buffer[5] << 8 | buffer[4])*gyro_scale+gz_offset;
    *ax = (int16_t)(buffer[7] << 8 | buffer[6])*accel_scale;
    *ay = (int16_t)(buffer[9] << 8 | buffer[8])*accel_scale;
    *az = (int16_t)(buffer[11] << 8 | buffer[10])*accel_scale;
}

void IMU_calibrate(int fd){
    gx_offset = 0;
    gy_offset = 0;
    gz_offset = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint32_t data_count = 0;
    auto start = std::chrono::system_clock::now();
    float prev_print_time = 0;
    float time = 0;
    while(std::chrono::system_clock::now()-start < std::chrono::duration<float>(calibration_time)){
        float ax, ay, az, gx, gy, gz;
        IMU_wait_and_get_data(fd, &ax, &ay, &az, &gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        data_count++;
    }
    q_est.q1 = 1;
    q_est.q2 = 0;
    q_est.q3 = 0;
    q_est.q4 = 0;
    dt = calibration_time/data_count;
    gx_offset = -gx_sum/data_count;
    gy_offset = -gy_sum/data_count;
    gz_offset = -gz_sum/data_count;
    std::cout << "sample rate: " << data_count/calibration_time << " samples/sec, dt=" << dt << std::endl;
    std::cout << "gx_offset: " << gx_offset << " rad/sec" << std::endl;
    std::cout << "gy_offset: " << gy_offset << " rad/sec" << std::endl;
    std::cout << "gz_offset: " << gz_offset << " rad/sec" << std::endl;
}

int main() {
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open SPI device" << std::endl;
        return 1;
    }

    uint8_t spi_mode = SPI_MODE_0;
    ioctl(fd, SPI_IOC_WR_MODE, &spi_mode);
    uint32_t spi_speed = SPI_SPEED;
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

    // Check WHO_AM_I register
    uint8_t whoami = 0;
    SPI_read(fd, WHO_AM_I, &whoami, 1);
    std::cout << "WHO_AM_I = 0x" << std::hex << int(whoami) << std::dec << std::endl;

    if (whoami != 0x6B && whoami != 0x69) {
        std::cerr << "Device not detected!" << std::endl;
        return 1;
    }

    // Configure accelerometer and gyro
    uint8_t a = 0x01;
    SPI_write(fd,0x12,&a,1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t ctrl1_xl = 0b10001100;
    uint8_t ctrl2_g  = 0b10001100;
    SPI_write(fd, CTRL1_XL, &ctrl1_xl, 1);
    SPI_write(fd, CTRL2_G,  &ctrl2_g,  1);

    std::cout << "calibrating..." << std::endl;
    IMU_calibrate(fd);

    // Read loop
    float prev_print_time = 0;
    float time = 0;
    while (true) {
        float ax, ay, az, gx, gy, gz;
        IMU_wait_and_get_data(fd, &ax, &ay, &az, &gx, &gy, &gz);

        imu_filter(ax, ay, az, gx, gy, gz);

        float roll,pitch,yaw;
        eulerAngles(q_est,&roll,&pitch,&yaw);
        
        time+=dt;
        if(time>prev_print_time+0.02){
            prev_print_time+=0.02;
            std::cout << std::endl;
            std::cout << "yaw:   " << yaw << std::endl;
            std::cout << "pitch: " << pitch << std::endl;
            std::cout << "roll:  " << roll << std::endl;
        }
    }

    close(fd);
    return 0;
}
