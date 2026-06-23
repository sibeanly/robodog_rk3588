// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file damiao_imu_driver.hpp
 * @brief Damiao DM-IMU-L1 IMU driver (USB CDC-ACM, 80-byte composite frame).
 * @details Implements the IMUDriver interface for the Damiao DM-IMU-L1 six-axis
 *          IMU module. Communicates over a USB CDC virtual serial port
 *          (/dev/dm_imu @ 921600, 8N1). The uplink is an 80-byte composite frame
 *          made of four sub-packets:
 *            offset  0-18 : acceleration (19B, reg=0x01, 3x float32, m/s^2)
 *            offset 19-37 : angular velocity (19B, reg=0x02, 3x float32, deg/s)
 *            offset 38-56 : euler angles (19B, reg=0x03, 3x float32, deg)
 *            offset 57-79 : quaternion (23B, reg=0x04, 4x float32, w/x/y/z)
 *          Each sub-packet is independently framed (header 0x55 0xAA, slave 0x01,
 *          CRC16-CCITT over the first 16 bytes, tail 0x0A) so a byte-stream
 *          sliding-window parser is used (ROS2 dm_serial approach).
 *
 *          Unit conversions applied to satisfy the base-class contract:
 *            - gyro   : deg/s -> rad/s  (x PI/180)  -> ang_vel_
 *            - quat   : stored as (w,x,y,z) directly              -> quat_
 *            - accel  : m/s^2 stored directly                     -> lin_acc_
 *            - temperature is NOT present in the 80-byte frame (returns 0.0)
 *
 *          The driver mirrors the hipnuc driver's threading model: the
 *          IMUSerialPort abstraction owns the background RX thread and delivers
 *          raw bytes via serial_rx_cbk(); parsing and field updates happen there
 *          under a shared_mutex.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "imu_driver.hpp"
#include "protocol/serial/serial_port.hpp"

// deg/s -> rad/s
constexpr double DAMIAO_DEG_TO_RAD = 0.017453292519943295769236907684886;

class DamiaoImuDriver : public IMUDriver {
   public:
    DamiaoImuDriver(uint16_t imu_id, const std::string& interface_type, const std::string& interface, const int baudrate = 0);
    ~DamiaoImuDriver();

    void serial_rx_cbk(const uint8_t* data, size_t length);

    std::vector<float> get_ang_vel() override;
    std::vector<float> get_quat() override;
    std::vector<float> get_lin_acc() override;
    float get_temperature() override;

   private:
    int baudrate_;
    std::string interface_type_;
    std::string interface_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSerialPort> serial_;

    // Parsed sensor data (SI units). Guarded by imu_mutex_.
    float quat_data_[4];       // w, x, y, z
    float ang_vel_data_[3];    // x, y, z (rad/s)
    float lin_acc_data_[3];    // x, y, z (m/s^2)
    float temperature_data_;   // deg C (not present in 80-byte frame -> stays 0.0)

    // Byte-stream receive buffer for sliding-window frame sync. Guarded by imu_mutex_.
    std::vector<uint8_t> rx_buf_;

    // Statistics. Guarded by imu_mutex_.
    uint32_t frame_count_ = 0;
    uint32_t crc_errors_ = 0;

    // Downlink configuration.
    void send_command(const uint8_t* cmd, size_t len, int repeat = 5);
    void configure_imu();

    // CRC16-CCITT (matches official Damiao Get_CRC16: poly 0x1021, init 0xFFFF,
    // table-driven with crc = (crc << 1) ^ table[(crc>>8) ^ byte]).
    static uint16_t compute_crc16(const uint8_t* data, int len);

    // Parse one sub-packet starting at a 0x55 0xAA header. Return true and update
    // the corresponding sensor field on success. The caller guarantees that at
    // least NORMAL_PACKET_LEN / EXTENDED_PACKET_LEN bytes are available.
    bool parse_normal_subpacket(const uint8_t* data);
    bool parse_quat_subpacket(const uint8_t* data);

    // Sliding-window parser over rx_buf_. Called under imu_mutex_.
    void parse_buffer();
};
