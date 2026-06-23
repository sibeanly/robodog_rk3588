// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file damiao_imu_driver.cpp
 * @brief Damiao DM-IMU-L1 IMU driver implementation.
 */

#include "damiao_imu_driver.hpp"

#include <algorithm>
#include <stdexcept>

// ── Protocol constants (verified against official ROS1 driver & STM32 imu.c) ──
namespace {
constexpr uint8_t FRAME_HEADER = 0x55;
constexpr uint8_t FRAME_TAG = 0xAA;
constexpr uint8_t FRAME_TAIL = 0x0A;
constexpr uint8_t DEFAULT_SLAVE = 0x01;

constexpr int NORMAL_PACKET_LEN = 19;    // accel / gyro / euler
constexpr int EXTENDED_PACKET_LEN = 23;  // quaternion
constexpr int CRC16_COVER = 16;          // CRC computed over the first 16 bytes

constexpr uint8_t REG_ACCEL = 0x01;
constexpr uint8_t REG_GYRO = 0x02;
constexpr uint8_t REG_EULER = 0x03;
constexpr uint8_t REG_QUAT = 0x04;

constexpr size_t MAX_BUF_SIZE = 4096;  // rx_buf_ cap (≈51 frames)

// CRC16-CCITT lookup table (poly 0x1021). Identical to the official Damiao
// bsp_crc.cpp CRC16_table and to the dm-imu-viewer references.
const uint16_t kCRC16Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};
}  // namespace

// ═══════════════════════════════════════════════════════════
// Construction / destruction
// ═══════════════════════════════════════════════════════════

DamiaoImuDriver::DamiaoImuDriver(uint16_t imu_id, const std::string& interface_type, const std::string& interface, const int baudrate)
    : IMUDriver(), interface_type_(interface_type), interface_(interface) {
    imu_id_ = imu_id;

    // Identity quaternion (w=1) until the first frame arrives.
    quat_data_[0] = 1.0f;
    quat_data_[1] = quat_data_[2] = quat_data_[3] = 0.0f;
    ang_vel_data_[0] = ang_vel_data_[1] = ang_vel_data_[2] = 0.0f;
    lin_acc_data_[0] = lin_acc_data_[1] = lin_acc_data_[2] = 0.0f;
    temperature_data_ = 0.0f;

    if (interface_type_ != "serial") {
        throw std::runtime_error("Damiao driver only supports the SERIAL interface (USB CDC-ACM)");
    }

    baudrate_ = baudrate;
    serial_ = IMUSerialPort::open(interface_, baudrate_);
    IMUSerialPort::SerialCbkFunc cbk =
        std::bind(&DamiaoImuDriver::serial_rx_cbk, this, std::placeholders::_1, std::placeholders::_2);
    serial_->set_serial_callback(cbk);

    // Push the downlink configuration (rate + channels). Quat is ON by default
    // on the module, so it is intentionally left untouched (no turn-off command).
    configure_imu();
}

DamiaoImuDriver::~DamiaoImuDriver() {
    if (serial_) {
        serial_->close();
    }
}

// ═══════════════════════════════════════════════════════════
// Downlink configuration
// ═══════════════════════════════════════════════════════════

void DamiaoImuDriver::send_command(const uint8_t* cmd, size_t len, int repeat) {
    if (!serial_) return;
    for (int i = 0; i < repeat; ++i) {
        serial_->write(cmd, len);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DamiaoImuDriver::configure_imu() {
    // Command byte sequences (verified against official ROS1 imu_driver.cpp).
    static const uint8_t CMD_ENTER[]   = {0xAA, 0x06, 0x01, 0x0D};  // enter config mode
    static const uint8_t CMD_ACC_ON[]  = {0xAA, 0x01, 0x14, 0x0D};  // enable accel channel
    static const uint8_t CMD_GYRO_ON[] = {0xAA, 0x01, 0x15, 0x0D};  // enable gyro channel
    static const uint8_t CMD_EUL_ON[]  = {0xAA, 0x01, 0x16, 0x0D};  // enable euler channel
    static const uint8_t CMD_QUAT_ON[] = {0xAA, 0x01, 0x17, 0x0D};  // enable quat channel (param = 0x13 + reg)
    static const uint8_t CMD_1000HZ[]  = {0xAA, 0x02, 0x01, 0x00, 0x0D};  // 1000Hz output
    static const uint8_t CMD_SAVE[]    = {0xAA, 0x03, 0x01, 0x0D};  // save to flash
    static const uint8_t CMD_EXIT[]    = {0xAA, 0x06, 0x00, 0x0D};  // exit config mode

    if (logger_) logger_->info("Damiao IMU [{}]: configuring (1000Hz, accel+gyro+euler+quat)...", interface_);

    send_command(CMD_ENTER, sizeof(CMD_ENTER));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_ACC_ON, sizeof(CMD_ACC_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_GYRO_ON, sizeof(CMD_GYRO_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_EUL_ON, sizeof(CMD_EUL_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Explicitly enable the quaternion channel. The module only documents the
    // quat-OFF command (AA 01 07 0D); the enable param follows the channel
    // pattern 0x13 + reg_id (accel 0x14, gyro 0x15, euler 0x16, quat 0x17),
    // confirmed by observing reg=0x04 sub-packets appear in the stream.
    send_command(CMD_QUAT_ON, sizeof(CMD_QUAT_ON));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_1000HZ, sizeof(CMD_1000HZ));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_SAVE, sizeof(CMD_SAVE));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_command(CMD_EXIT, sizeof(CMD_EXIT));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let the stream restart

    // Discard any stale bytes buffered during the config session.
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    rx_buf_.clear();
    frame_count_ = 0;
    crc_errors_ = 0;
    if (logger_) logger_->info("Damiao IMU [{}]: configuration complete", interface_);
}

// ═══════════════════════════════════════════════════════════
// CRC16-CCITT (official Damiao Get_CRC16)
// ═══════════════════════════════════════════════════════════

uint16_t DamiaoImuDriver::compute_crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        uint8_t index = static_cast<uint8_t>((crc >> 8) ^ data[i]);
        crc = static_cast<uint16_t>((crc << 1) ^ kCRC16Table[index]);
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════
// Sub-packet parsing
// ═══════════════════════════════════════════════════════════

bool DamiaoImuDriver::parse_normal_subpacket(const uint8_t* data) {
    if (data[0] != FRAME_HEADER || data[1] != FRAME_TAG || data[2] != DEFAULT_SLAVE) return false;
    if (data[NORMAL_PACKET_LEN - 1] != FRAME_TAIL) return false;

    uint8_t reg = data[3];

    uint16_t wire_crc = static_cast<uint16_t>(data[16]) | (static_cast<uint16_t>(data[17]) << 8);
    // Mode 1: CRC over bytes[0..15] (with header) — official STM32/ROS1 default.
    uint16_t calc = compute_crc16(data, CRC16_COVER);
    if (calc != wire_crc) {
        // Mode 2: CRC over bytes[2..15] (without header) — official ROS2 fallback.
        calc = compute_crc16(data + 2, CRC16_COVER - 2);
        if (calc != wire_crc) return false;
    }

    // Valid framed sub-packet. Update the fields we expose for the data
    // channels; other regs (e.g. the 0x05/0x07 status packets the module emits
    // when the quat channel is enabled) are valid frames we simply consume to
    // keep the byte stream synchronised.
    if (reg == REG_ACCEL) {
        float values[3];
        std::memcpy(values, data + 4, sizeof(values));  // 3x float32, little-endian
        lin_acc_data_[0] = values[0];                   // m/s^2
        lin_acc_data_[1] = values[1];
        lin_acc_data_[2] = values[2];
    } else if (reg == REG_GYRO) {
        float values[3];
        std::memcpy(values, data + 4, sizeof(values));
        ang_vel_data_[0] = static_cast<float>(values[0] * DAMIAO_DEG_TO_RAD);  // deg/s -> rad/s
        ang_vel_data_[1] = static_cast<float>(values[1] * DAMIAO_DEG_TO_RAD);
        ang_vel_data_[2] = static_cast<float>(values[2] * DAMIAO_DEG_TO_RAD);
    }
    // REG_EULER (0x03) and any other reg: valid frame, accepted and ignored.
    return true;
}

bool DamiaoImuDriver::parse_quat_subpacket(const uint8_t* data) {
    if (data[0] != FRAME_HEADER || data[1] != FRAME_TAG || data[2] != DEFAULT_SLAVE) return false;
    if (data[3] != REG_QUAT) return false;
    if (data[EXTENDED_PACKET_LEN - 1] != FRAME_TAIL) return false;

    // The 23-byte quaternion sub-packet carries 4 floats, so its CRC covers
    // bytes[0..19] (header + slave + reg + 4 floats) — i.e. everything up to
    // the wire CRC at [20..21]. This differs from the 19-byte sub-packets,
    // whose CRC covers only [0..15]. (Verified empirically against the module:
    // the dm-imu-viewer references use 16 for both and never exercise the quat
    // path because their default config disables quat.)
    constexpr int QUAT_CRC_COVER = 20;
    uint16_t wire_crc = static_cast<uint16_t>(data[20]) | (static_cast<uint16_t>(data[21]) << 8);
    uint16_t calc = compute_crc16(data, QUAT_CRC_COVER);
    if (calc != wire_crc) {
        calc = compute_crc16(data + 2, QUAT_CRC_COVER - 2);
        if (calc != wire_crc) return false;
    }

    float q[4];
    std::memcpy(q, data + 4, sizeof(q));  // w, x, y, z (little-endian float32)
    quat_data_[0] = q[0];
    quat_data_[1] = q[1];
    quat_data_[2] = q[2];
    quat_data_[3] = q[3];
    return true;
}

// ═══════════════════════════════════════════════════════════
// Byte-stream sliding-window frame synchronizer
// ═══════════════════════════════════════════════════════════

void DamiaoImuDriver::parse_buffer() {
    while (!rx_buf_.empty()) {
        // Search for the 0x55 0xAA header.
        size_t j = rx_buf_.size();
        for (size_t k = 0; k + 1 < rx_buf_.size(); ++k) {
            if (rx_buf_[k] == FRAME_HEADER && rx_buf_[k + 1] == FRAME_TAG) {
                j = k;
                break;
            }
        }
        if (j >= rx_buf_.size()) {
            // No header: keep the last byte in case a header straddles chunks.
            if (rx_buf_.size() > 1) {
                uint8_t last = rx_buf_.back();
                rx_buf_.clear();
                rx_buf_.push_back(last);
            }
            return;
        }

        size_t remaining = rx_buf_.size() - j;
        if (remaining < static_cast<size_t>(NORMAL_PACKET_LEN)) {
            // Not enough data yet; keep from the header onward.
            if (j > 0) rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + j);
            return;
        }

        uint8_t reg = rx_buf_[j + 3];
        bool ok = false;
        int consumed;
        if (reg == REG_QUAT) {
            if (remaining < static_cast<size_t>(EXTENDED_PACKET_LEN)) {
                if (j > 0) rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + j);
                return;
            }
            ok = parse_quat_subpacket(rx_buf_.data() + j);
            consumed = EXTENDED_PACKET_LEN;
        } else {
            ok = parse_normal_subpacket(rx_buf_.data() + j);
            consumed = NORMAL_PACKET_LEN;
        }

        if (!ok) {
            // Drop through this header byte and resync.
            rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + j + 1);
            ++crc_errors_;
            continue;
        }

        ++frame_count_;
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + j + consumed);
    }
}

// ═══════════════════════════════════════════════════════════
// RX callback (runs on the IMUSerialPort background thread)
// ═══════════════════════════════════════════════════════════

void DamiaoImuDriver::serial_rx_cbk(const uint8_t* data, size_t length) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    rx_buf_.insert(rx_buf_.end(), data, data + length);
    if (rx_buf_.size() > MAX_BUF_SIZE) {
        // Drop the oldest bytes to cap memory (keep the most recent tail).
        rx_buf_.erase(rx_buf_.begin(),
                      rx_buf_.begin() + (rx_buf_.size() - MAX_BUF_SIZE));
    }
    parse_buffer();
}

// ═══════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════

std::vector<float> DamiaoImuDriver::get_ang_vel() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {ang_vel_data_[0], ang_vel_data_[1], ang_vel_data_[2]};
}

std::vector<float> DamiaoImuDriver::get_quat() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {quat_data_[0], quat_data_[1], quat_data_[2], quat_data_[3]};
}

std::vector<float> DamiaoImuDriver::get_lin_acc() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {lin_acc_data_[0], lin_acc_data_[1], lin_acc_data_[2]};
}

float DamiaoImuDriver::get_temperature() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    // Temperature is not present in the standard 80-byte uplink frame.
    return temperature_data_;
}
