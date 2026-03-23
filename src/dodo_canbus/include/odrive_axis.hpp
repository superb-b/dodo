#pragma once

#include <cstdint>

#include "can_port.hpp"
#include "odrive_protocol.hpp"
#include "odrive_enums.h"

class ODriveAxis
{
public:
    ODriveAxis(CanPort::SharedPtr can, uint8_t node_id);

    void estop();
    void clear_errors();

    void set_axis_state(ODriveAxisState state);
    void set_closed_loop();
    void set_idle();

    void set_controller_mode(ODriveControlMode control_mode,
                             ODriveInputMode input_mode);

    void set_input_torque(float torque);
    void set_calibration();
    void set_input_vel(float vel, float torque_ff = 0.0f);
    void set_input_pos(float pos, int16_t vel_ff = 0, int16_t torque_ff = 0);

    uint8_t node_id() const { return node_id_; }

private:
    void send(uint32_t can_id, const uint8_t* data, uint8_t len);

private:
    CanPort::SharedPtr can_;
    uint8_t node_id_;
};

inline ODriveAxis::ODriveAxis(CanPort::SharedPtr can, uint8_t node_id)
: can_(std::move(can)), node_id_(node_id)
{
}

inline void ODriveAxis::send(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (!can_)
    {
        return;
    }
    can_->send(can_id, data, len);
}

inline void ODriveAxis::estop()
{
    uint8_t data[8] = {};
    send(odrive::make_can_id(node_id_, odrive::CmdId::Estop), data, 0);
}

inline void ODriveAxis::clear_errors()
{
    uint8_t data[8] = {};
    data[0] = 0;
    send(odrive::make_can_id(node_id_, odrive::CmdId::ClearErrors), data, 1);
}

inline void ODriveAxis::set_axis_state(ODriveAxisState state)
{
    uint8_t data[8] = {};
    odrive::write_le<uint32_t>(static_cast<uint32_t>(state), data);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetAxisState), data, 4);
}

inline void ODriveAxis::set_calibration()
{
    uint8_t data[8] = {};
    odrive::write_le<uint32_t>(static_cast<uint32_t>(AXIS_STATE_FULL_CALIBRATION_SEQUENCE), data);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetAxisState), data, 4);
}

inline void ODriveAxis::set_closed_loop()
{
    set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

inline void ODriveAxis::set_idle()
{
    set_axis_state(AXIS_STATE_IDLE);
}

inline void ODriveAxis::set_controller_mode(ODriveControlMode control_mode,
                                            ODriveInputMode input_mode)
{
    uint8_t data[8] = {};
    odrive::write_le<uint32_t>(static_cast<uint32_t>(control_mode), data);
    odrive::write_le<uint32_t>(static_cast<uint32_t>(input_mode), data + 4);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetControllerMode), data, 8);
}

inline void ODriveAxis::set_input_torque(float torque)
{
    uint8_t data[8] = {};
    odrive::write_le<float>(torque, data);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetInputTorque), data, 4);
}

inline void ODriveAxis::set_input_vel(float vel, float torque_ff)
{
    uint8_t data[8] = {};
    odrive::write_le<float>(vel, data);
    odrive::write_le<float>(torque_ff, data + 4);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetInputVel), data, 8);
}

inline void ODriveAxis::set_input_pos(float pos, int16_t vel_ff, int16_t torque_ff)
{
    uint8_t data[8] = {};
    odrive::write_le<float>(pos, data);
    odrive::write_le<int16_t>(vel_ff, data + 4);
    odrive::write_le<int16_t>(torque_ff, data + 6);
    send(odrive::make_can_id(node_id_, odrive::CmdId::SetInputPos), data, 8);
}