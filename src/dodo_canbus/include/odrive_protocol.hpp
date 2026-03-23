#pragma once
#include <cstdint>
#include <cstring>
#include <type_traits>
#include "odrive_enums.h"

namespace odrive
{

enum class CmdId : uint32_t
{
    Heartbeat             = 0x001,
    Estop                 = 0x002,
    GetError              = 0x003,
    SetAxisState          = 0x007,
    GetEncoderEstimates   = 0x009,
    SetControllerMode     = 0x00B,
    SetInputPos           = 0x00C,
    SetInputVel           = 0x00D,
    SetInputTorque        = 0x00E,
    GetIq                 = 0x014,
    GetTemp               = 0x015,
    GetBusVoltageCurrent  = 0x017,
    ClearErrors           = 0x018,
    GetTorques            = 0x01C,
};

inline constexpr CmdId extract_cmd(uint32_t can_id)
{
    return static_cast<CmdId>(can_id & 0x1F);
}

inline constexpr uint32_t make_can_id(uint8_t node_id, CmdId cmd)
{
    return (static_cast<uint32_t>(node_id) << 5) |
           (static_cast<uint32_t>(cmd) & 0x1F);
}

inline constexpr uint8_t extract_node_id(uint32_t can_id)
{
    return static_cast<uint8_t>((can_id >> 5) & 0x3F);
}

inline constexpr uint8_t extract_cmd_id(uint32_t can_id)
{
    return static_cast<uint8_t>(can_id & 0x1F);
}

template<typename T>
inline void write_le(T value, uint8_t* dst)
{
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    std::memcpy(dst, &value, sizeof(T));
}

template<typename T>
inline T read_le(const uint8_t* src)
{
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    T value{};
    std::memcpy(&value, src, sizeof(T));
    return value;
}

} // namespace odrive