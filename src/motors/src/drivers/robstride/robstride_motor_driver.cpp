// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi
// Copyright (C) 2026 wentywenty
//
// RobStride03 motor driver implementation. Protocol ported from the
// EDULITE_A3 robstride_can_driver reference (encode/decode, extended-ID layout,
// comm-type dispatch), adapted to the MotorDriver interface and the
// MotorsCAN SocketCAN abstraction.

#include "robstride_motor_driver.hpp"

#include <linux/can.h>
#include <cstring>

// RS03 limit table (per confirmed spec: P=+/-4pi, V=+/-20, T=+/-60,
// KP=0..5000, KD=0..100).
RS03_Limit_Param rs03_limit_param[RS03_Num_Of_Motor] = {
    {4.0f * static_cast<float>(M_PI), 20.0f, 60.0f, 5000.0f, 100.0f},  // RS03_MODEL
};

// Default gains for the pos/spd convenience wrappers. Real-time control goes
// through motor_mit_cmd (used by the inference node), so these only apply when
// the legacy non-MIT command paths are exercised.
static constexpr float RS03_POS_KP_DEFAULT = 30.0f;
static constexpr float RS03_POS_KD_DEFAULT = 3.0f;
static constexpr float RS03_SPD_KD_DEFAULT = 5.0f;

RobStrideMotorDriver::RobStrideMotorDriver(uint16_t motor_id, const std::string& interface_type,
                                           const std::string& can_interface, RS03_Motor_Model motor_model,
                                           double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "can") {
        throw std::runtime_error("RobStride03 driver only supports classic CAN (CAN 2.0) interface");
    }
    motor_id_ = motor_id;
    master_id_ = 0x00;       // fixed host address (no master_id_offset for RobStride)
    motor_sign_ = 1.0f;
    limit_param_ = rs03_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    comm_type_ = CommType::CAN;
    can_ = MotorsCAN::get(can_interface);

    // RX status frames (comm 0x02) carry the source motor_id in ExtID bits
    // 15..8 and the host/master_id in bits 7..0. The default MotorsCAN key
    // extractor returns the low 16 bits of can_id, so we key our callback on
    // (motor_id<<8)|master_id — this demuxes by motor_id without disturbing
    // co-resident standard-ID motors and without touching the shared extractor.
    CanCbkFunc can_callback = std::bind(&RobStrideMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    cbk_key_ = static_cast<CanCbkId>((static_cast<uint16_t>(motor_id_ & 0xFF) << 8) | master_id_);
    can_->add_can_callback(can_callback, cbk_key_);
}

RobStrideMotorDriver::~RobStrideMotorDriver() {
    can_->remove_can_callback(cbk_key_);
}

uint32_t RobStrideMotorDriver::build_ext_can_id(uint8_t comm_type, uint16_t data_area2, uint8_t target_id) {
    // 29-bit ID layout:
    //   bits 28..24: comm_type (5 bits)
    //   bits 23..8 : data_area2 (16 bits) — host CAN_ID, or torque raw for MIT
    //   bits 7..0  : target motor CAN_ID
    uint32_t id = 0;
    id |= (static_cast<uint32_t>(comm_type) & 0x1F) << 24;
    id |= (static_cast<uint32_t>(data_area2) & 0xFFFF) << 8;
    id |= static_cast<uint32_t>(target_id) & 0xFF;
    return id | CAN_EFF_FLAG;
}

uint16_t RobStrideMotorDriver::float_to_uint16(float x, float x_min, float x_max) {
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    float span = x_max - x_min;
    uint32_t raw = static_cast<uint32_t>((x - x_min) * 65535.0f / span + 0.5f);
    if (raw > 65535) raw = 65535;
    return static_cast<uint16_t>(raw);
}

float RobStrideMotorDriver::uint16_to_float(uint16_t x_int, float x_min, float x_max) {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span / 65535.0f + x_min;
}

void RobStrideMotorDriver::send_comm_frame(uint8_t comm_type, uint8_t target_id, const uint8_t* data, uint8_t len) {
    can_frame tx_frame{};
    tx_frame.can_id = build_ext_can_id(comm_type, master_id_, target_id);
    tx_frame.can_dlc = (len > 8) ? 8 : len;
    if (data && tx_frame.can_dlc > 0) {
        std::memcpy(tx_frame.data, data, tx_frame.can_dlc);
    }
    can_->transmit(tx_frame);
    response_count_++;
}

void RobStrideMotorDriver::send_mit_control(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    // Apply zero offset (subtract on TX, like dm) and direction sign.
    f_p = (f_p - static_cast<float>(motor_zero_offset_)) * motor_sign_;
    f_v = f_v * motor_sign_;
    f_t = f_t * motor_sign_;

    f_p  = limit(f_p,  -limit_param_.PosMax, limit_param_.PosMax);
    f_v  = limit(f_v,  -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f,                  limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f,                  limit_param_.OKdMax);
    f_t  = limit(f_t,  -limit_param_.TauMax,  limit_param_.TauMax);

    uint16_t p_raw  = float_to_uint16(f_p,  -limit_param_.PosMax, limit_param_.PosMax);
    uint16_t v_raw  = float_to_uint16(f_v,  -limit_param_.SpdMax, limit_param_.SpdMax);
    uint16_t kp_raw = float_to_uint16(f_kp, 0.0f,                 limit_param_.OKpMax);
    uint16_t kd_raw = float_to_uint16(f_kd, 0.0f,                 limit_param_.OKdMax);
    uint16_t t_raw  = float_to_uint16(f_t,  -limit_param_.TauMax,  limit_param_.TauMax);

    // Comm 0x01: torque feedforward rides in ExtID bits 23..8; data carries
    // p/v/kp/kd as big-endian uint16 pairs (matches EDULITE_A3 sendMotionControl).
    can_frame tx_frame{};
    tx_frame.can_id = build_ext_can_id(RS03_COMM_MIT_CMD, t_raw, static_cast<uint8_t>(motor_id_ & 0xFF));
    tx_frame.can_dlc = 8;
    tx_frame.data[0] = (p_raw >> 8) & 0xFF;
    tx_frame.data[1] = p_raw & 0xFF;
    tx_frame.data[2] = (v_raw >> 8) & 0xFF;
    tx_frame.data[3] = v_raw & 0xFF;
    tx_frame.data[4] = (kp_raw >> 8) & 0xFF;
    tx_frame.data[5] = kp_raw & 0xFF;
    tx_frame.data[6] = (kd_raw >> 8) & 0xFF;
    tx_frame.data[7] = kd_raw & 0xFF;

    can_->transmit(tx_frame);
    response_count_++;
}

void RobStrideMotorDriver::lock_motor() {
    // Enable motor (comm 0x03).
    uint8_t data[8] = {0};
    send_comm_frame(RS03_COMM_ENABLE, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
}

void RobStrideMotorDriver::unlock_motor() {
    // Disable motor (comm 0x04), no fault clear.
    uint8_t data[8] = {0};
    send_comm_frame(RS03_COMM_DISABLE, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
}

uint8_t RobStrideMotorDriver::init_motor() {
    // Mirror the dm init sequence: disable -> set MIT mode -> enable -> refresh.
    unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    lock_motor();
    Timer::sleep_for(normal_sleep_time);
    refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    return error_id_;
}

void RobStrideMotorDriver::deinit_motor() {
    unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool RobStrideMotorDriver::write_motor_flash() {
    // Comm 0x16: save current parameters to Flash.
    uint8_t data[8] = {0};
    data[0] = 1;
    send_comm_frame(RS03_COMM_SAVE, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
    Timer::sleep_for(setup_sleep_time);
    return true;
}

bool RobStrideMotorDriver::set_motor_zero() {
    // Comm 0x06: set current position as mechanical zero (data[0]=1).
    uint8_t data[8] = {0};
    data[0] = 1;
    send_comm_frame(RS03_COMM_SET_ZERO, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
    Timer::sleep_for(setup_sleep_time);
    refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    logger_->info("motor_id: {0}\tposition: {1}", motor_id_, get_motor_pos());
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
}

void RobStrideMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    // Only comm 0x02 (status) frames are meaningful here; they are already
    // demuxed to us by the callback key.
    uint32_t can_id = rx_frame.can_id & CAN_EFF_MASK;
    uint8_t comm_type = (can_id >> 24) & 0x1F;
    if (comm_type != RS03_COMM_STATUS) {
        return;
    }

    // ExtID bits 23..22 = mode state, bits 21..16 = fault code.
    uint8_t fault_code = (can_id >> 16) & 0x3F;
    if (fault_code != 0) {
        error_id_ = fault_code;
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\tfault: 0x{2:x}",
                           can_interface_, motor_id_, static_cast<uint32_t>(error_id_));
        }
    } else {
        error_id_ = 0;
    }

    // Data is big-endian uint16 pairs: pos, vel, torque, temp(*0.1 °C).
    uint16_t pos_raw    = (static_cast<uint16_t>(rx_frame.data[0]) << 8) | rx_frame.data[1];
    uint16_t vel_raw    = (static_cast<uint16_t>(rx_frame.data[2]) << 8) | rx_frame.data[3];
    uint16_t torque_raw = (static_cast<uint16_t>(rx_frame.data[4]) << 8) | rx_frame.data[5];
    uint16_t temp_raw   = (static_cast<uint16_t>(rx_frame.data[6]) << 8) | rx_frame.data[7];

    float raw_pos = uint16_to_float(pos_raw,    -limit_param_.PosMax, limit_param_.PosMax);
    float raw_vel = uint16_to_float(vel_raw,    -limit_param_.SpdMax, limit_param_.SpdMax);
    float raw_tq  = uint16_to_float(torque_raw, -limit_param_.TauMax,  limit_param_.TauMax);

    // Apply sign on raw telemetry, then add zero offset to position (mirror dm,
    // which adds the offset on RX and subtracts on TX).
    motor_pos_        = raw_pos * motor_sign_ + static_cast<float>(motor_zero_offset_);
    motor_spd_        = raw_vel * motor_sign_;
    motor_current_    = raw_tq * motor_sign_;
    motor_temperature_ = static_cast<float>(temp_raw) * 0.1f;
}

void RobStrideMotorDriver::get_motor_param(uint8_t param_cmd) {
    // Comm 0x11: read parameter by index (data[0..1] = index, little-endian).
    uint8_t data[8] = {0};
    data[0] = param_cmd & 0xFF;
    data[1] = (param_cmd >> 8) & 0xFF;
    send_comm_frame(RS03_COMM_READ_PARAM, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
}

void RobStrideMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    // RS03 has no dedicated position frame; position control is realised through
    // the MIT frame with position + velocity feed-forward and default gains.
    (void)ignore_limit;
    send_mit_control(pos, spd, RS03_POS_KP_DEFAULT, RS03_POS_KD_DEFAULT, 0.0f);
}

void RobStrideMotorDriver::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    // Pure velocity control via MIT: kp=0 so position target is ignored, kd
    // provides the velocity gain, zero feed-forward torque.
    send_mit_control(0.0f, spd, 0.0f, RS03_SPD_KD_DEFAULT, 0.0f);
}

void RobStrideMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    send_mit_control(f_p, f_v, f_kp, f_kd, f_t);
}

void RobStrideMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    // RS03's motion-control (MIT) mode is implied by sending comm 0x01 frames;
    // there is no separate mode-register command to issue (unlike dm/xyn). We
    // only track the requested mode locally so the pos/spd/mit command paths
    // can follow the same first-call-sets-mode convention as the dm driver.
    if (motor_control_mode_ == motor_control_mode) {
        return;
    }
    motor_control_mode_ = motor_control_mode;
}

void RobStrideMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    // Comm 0x07: change the motor CAN ID. Targeted at old_id; new id in data[0].
    uint8_t data[8] = {0};
    data[0] = new_id;
    send_comm_frame(RS03_COMM_SET_ID, old_id, data, 8);
    Timer::sleep_for(setup_sleep_time);
}

void RobStrideMotorDriver::reset_motor_id() {
    RobStrideMotorDriver::set_motor_id(static_cast<uint8_t>(motor_id_ & 0xFF), 0x01);
}

void RobStrideMotorDriver::refresh_motor_status() {
    // RS03 emits comm 0x02 status in response to any comm 0x01 control frame.
    // Solicit a fresh status by sending a zero-torque MIT frame (kp=kd=t=0 ->
    // no applied force) which the motor acks with its current telemetry.
    send_mit_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void RobStrideMotorDriver::clear_motor_error() {
    // Comm 0x04 with data[0]=1 clears the active fault / protection state.
    uint8_t data[8] = {0};
    data[0] = 1;
    send_comm_frame(RS03_COMM_DISABLE, static_cast<uint8_t>(motor_id_ & 0xFF), data, 8);
}
