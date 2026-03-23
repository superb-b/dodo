#ifndef CAN_DISPATCHER_HPP
#define CAN_DISPATCHER_HPP

#include "can_port.hpp"
#include <vector>
#include <functional>
#include <cstring>

struct CANFrame
{
    uint32_t id{0};
    uint8_t data[8]{};
    uint8_t len{0};
};

class CANDispatcher
{
public:
    using Callback = std::function<void(const CANFrame&)>;

    explicit CANDispatcher(CanPort::SharedPtr can)
        : can_(std::move(can))
    {
    }

    void register_callback(Callback cb)
    {
        callbacks_.push_back(std::move(cb));
    }

    std::size_t poll(std::size_t max_frames = 512)
    {
        std::size_t count = 0;

        while (count < max_frames)
        {
            uint32_t id = 0;
            uint8_t data[8] = {};
            uint8_t len = 0;

            const ssize_t ret = can_->recv(id, data, len);
            if (ret <= 0)
            {
                break;
            }

            CANFrame frame;
            frame.id = id;
            frame.len = len;
            std::memcpy(frame.data, data, len);

            for (auto &cb : callbacks_)
            {
                cb(frame);
            }

            ++count;
        }

        return count;
    }

private:
    CanPort::SharedPtr can_;
    std::vector<Callback> callbacks_;
};

#endif