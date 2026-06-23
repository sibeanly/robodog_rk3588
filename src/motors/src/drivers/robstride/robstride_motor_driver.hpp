// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi
// Copyright (C) 2026 wentywenty
//
// RobStride03 motor driver (MotorDriver interface).
// Implements the RobStride MIT-style control protocol over classic SocketCAN
// (CAN 2.0, 1 Mbps, 29-bit extended IDs). Protocol ported from the EDULITE_A3
// robstride_can_driver reference implementation.

#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can_iso.hpp"
#include "utils.hpp"

// RobStride private-protocol communication types (ExtID bits 28..24).
enum RS03_CommType {
    RS03_COMM_MIT_CMD    = 0x01,  // Motion control (MIT) command -> status reply
    RS03_COMM_STATUS     = 0x02,  // Motor status feedback (RX only)
    RS03_COMM_ENABLE     = 0x03,  // Enable motor
    RS03_COMM_DISABLE    = 0x04,  // Disable motor (data[0]=1 clears fault)
    RS03_COMM_SET_ZERO   = 0x06,  // Set current position as mechanical zero
    RS03_COMM_SET_ID     = 0x07,  // Change motor CAN ID
    RS03_COMM_READ_PARAM = 0x11,  // Read parameter by index
    RS03_COMM_WRITE_PARAM= 0x12,  // Write parameter (volatile)
    RS03_COMM_SAVE       = 0x16,  // Save parameters to Flash
};

enum RS03_Motor_Model {
    RS03_MODEL,
    RS03_Num_Of_Motor
};

// RS03 mechanical/limits per the confirmed protocol spec.
//   P = +/- 4*pi rad, V = +/- 20 rad/s (mevius2 derate),
//   KP = 0..5000, KD = 0..100, T = +/- 60 Nm.
typedef struct {
    float PosMax;   ///< Maximum position limit (rad)
    float SpdMax;   ///< Maximum velocity limit (rad/s)
    float TauMax;   ///< Maximum torque limit (N·m)
    float OKpMax;   ///< Maximum position gain (Kp)
    float OKdMax;   ///< Maximum velocity gain (Kd)
} RS03_Limit_Param;

class RobStrideMotorDriver : public MotorDriver {
   public:
    RobStrideMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                         RS03_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~RobStrideMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;
    virtual void get_motor_param(uint8_t param_cmd) override;

    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override {}
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override {
        return response_count_;
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    virtual void reset_motor_id() override;
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    // Fixed host/master CAN address. Unlike Damiao there is no master_id_offset;
    // the host address is constant for all motors on the bus.
    uint8_t master_id_{0x00};
    // Direction sign (+1 / -1). Applied as a multiplier to position, velocity
    // and torque on both TX (after zero-offset subtraction) and RX. Defaults to
    // +1 so behaviour is unchanged unless explicitly configured.
    float motor_sign_{1.0f};

    std::atomic<int> response_count_{0};
    RS03_Motor_Model motor_model_;
    RS03_Limit_Param limit_param_;

    // Callback key registered with the CAN backend (low 16 bits of the RX
    // ExtID = (motor_id<<8)|master_id). Stored so the destructor can deregister.
    CanCbkId cbk_key_{0};

    // RX handler — invoked by MotorsCAN for status frames addressed to us.
    virtual void can_rx_cbk(const can_frame& rx_frame);

    // Build a 29-bit extended CAN ID (with CAN_EFF_FLAG) from a comm type, the
    // 16-bit data-area-2 field (host addr or torque raw), and a target motor id.
    static uint32_t build_ext_can_id(uint8_t comm_type, uint16_t data_area2, uint8_t target_id);

    // Encode/decode helpers: linear map float <-> uint16 over [x_min, x_max].
    static uint16_t float_to_uint16(float x, float x_min, float x_max);
    static float uint16_to_float(uint16_t x_int, float x_min, float x_max);

    // Core MIT control frame (comm 0x01). Applies zero-offset + sign, clamps to
    // limits, packs big-endian and transmits. Shared by motor_mit_cmd and the
    // pos/spd convenience wrappers.
    void send_mit_control(float f_p, float f_v, float f_kp, float f_kd, float f_t);

    // Send a control-style frame (comm types that carry master_id in data-area-2
    // and target motor_id in bits 7..0). `data` may be nullptr when len==0.
    void send_comm_frame(uint8_t comm_type, uint8_t target_id, const uint8_t* data, uint8_t len);

    std::shared_ptr<MotorsCAN> can_;
};
