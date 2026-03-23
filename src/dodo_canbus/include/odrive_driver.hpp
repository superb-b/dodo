#ifndef ODRIVE_DRIVER_HPP
#define ODRIVE_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <chrono>

#include "odrive_protocol.hpp"
#include "odrive_enums.h"

class ODriveDriver
{
public:
    struct EncoderState
    {
        float pos{0.0f};
        float vel{0.0f};
        bool valid{false};
    };

    struct HeartbeatState
    {
        uint32_t active_errors{0};
        ODriveAxisState axis_state{AXIS_STATE_UNDEFINED};
        ODriveProcedureResult procedure_result{PROCEDURE_RESULT_SUCCESS};
        bool trajectory_done{false};
        bool valid{false};
    };

    struct ErrorState
    {
        uint32_t active_errors{0};
        uint32_t disarm_reason{0};
        bool valid{false};
    };

    struct IqState
    {
        float iq_setpoint{0.0f};
        float iq_measured{0.0f};
        bool valid{false};
    };

    struct TempState
    {
        float fet_temperature{0.0f};
        float motor_temperature{0.0f};
        bool valid{false};
    };

    struct BusState
    {
        float bus_voltage{0.0f};
        float bus_current{0.0f};
        bool valid{false};
    };

    struct TorqueState
    {
        float torque_target{0.0f};
        float torque_estimate{0.0f};
        bool valid{false};
    };

    ODriveDriver() = default;

    void process_frame(uint32_t id, const uint8_t* data, uint8_t len)
    {
        const auto cmd = odrive::extract_cmd(id);
        const uint8_t axis = odrive::extract_node_id(id);

        switch (cmd)
        {
            case odrive::CmdId::Heartbeat:
            {
                if (len < 8) return;

                HeartbeatState state;
                state.active_errors   = odrive::read_le<uint32_t>(data + 0);
                state.axis_state      = static_cast<ODriveAxisState>(odrive::read_le<uint8_t>(data + 4));
                state.procedure_result= static_cast<ODriveProcedureResult>(odrive::read_le<uint8_t>(data + 5));
                state.trajectory_done = odrive::read_le<bool>(data + 6);
                state.valid = true;

                heartbeats_[axis] = state;
                return;
            }

            case odrive::CmdId::GetError:
            {
                if (len < 8) return;

                ErrorState state;
                state.active_errors = odrive::read_le<uint32_t>(data + 0);
                state.disarm_reason = odrive::read_le<uint32_t>(data + 4);
                state.valid = true;

                errors_[axis] = state;
                return;
            }

            case odrive::CmdId::GetEncoderEstimates:
            {
                if (len < 8) return;

                EncoderState state;
                state.pos = odrive::read_le<float>(data + 0);
                state.vel = odrive::read_le<float>(data + 4);
                state.valid = true;

                encoders_[axis] = state;
                return;
            }

            case odrive::CmdId::GetIq:
            {
                if (len < 8) return;

                IqState state;
                state.iq_setpoint = odrive::read_le<float>(data + 0);
                state.iq_measured = odrive::read_le<float>(data + 4);
                state.valid = true;

                iqs_[axis] = state;
                return;
            }

            case odrive::CmdId::GetTemp:
            {
                if (len < 8) return;

                TempState state;
                state.fet_temperature   = odrive::read_le<float>(data + 0);
                state.motor_temperature = odrive::read_le<float>(data + 4);
                state.valid = true;

                temps_[axis] = state;
                
                return;
            }

            case odrive::CmdId::GetBusVoltageCurrent:
            {
                if (len < 8) return;

                BusState state;
                state.bus_voltage = odrive::read_le<float>(data + 0);
                state.bus_current = odrive::read_le<float>(data + 4);
                state.valid = true;

                bus_states_[axis] = state;
                return;
            }

            case odrive::CmdId::GetTorques:
            {
                if (len < 8) return;

                TorqueState state;
                state.torque_target   = odrive::read_le<float>(data + 0);
                state.torque_estimate = odrive::read_le<float>(data + 4);
                state.valid = true;

                torques_[axis] = state;
                
                return;
            }

            default:
                return;
        }
    }

    EncoderState get_encoder(uint8_t axis) const
    {
        auto it = encoders_.find(axis);
        if (it != encoders_.end())
        {
            return it->second;
        }
        return {};
    }

    HeartbeatState get_heartbeat(uint8_t axis) const
    {
        auto it = heartbeats_.find(axis);
        if (it != heartbeats_.end())
        {
            return it->second;
        }
        return {};
    }

    ErrorState get_error(uint8_t axis) const
    {
        auto it = errors_.find(axis);
        if (it != errors_.end())
        {
            return it->second;
        }
        return {};
    }

    IqState get_iq(uint8_t axis) const
    {
        auto it = iqs_.find(axis);
        if (it != iqs_.end())
        {
            return it->second;
        }
        return {};
    }

    TempState get_temp(uint8_t axis) const
    {
        auto it = temps_.find(axis);
        if (it != temps_.end())
        {
            return it->second;
        }
        return {};
    }

    BusState get_bus(uint8_t axis) const
    {
        auto it = bus_states_.find(axis);
        if (it != bus_states_.end())
        {
            return it->second;
        }
        return {};
    }

    TorqueState get_torque(uint8_t axis) const
    {
        auto it = torques_.find(axis);
        if (it != torques_.end())
        {
            return it->second;
        }
        return {};
    }

private:
    std::unordered_map<uint8_t, EncoderState> encoders_;
    std::unordered_map<uint8_t, HeartbeatState> heartbeats_;
    std::unordered_map<uint8_t, ErrorState> errors_;
    std::unordered_map<uint8_t, IqState> iqs_;
    std::unordered_map<uint8_t, TempState> temps_;
    std::unordered_map<uint8_t, BusState> bus_states_;
    std::unordered_map<uint8_t, TorqueState> torques_;
};

#endif