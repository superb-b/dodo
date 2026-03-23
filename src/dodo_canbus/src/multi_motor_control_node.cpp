#include "multi_motor_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

MultiMotorControlNode::MultiMotorControlNode()
: Node("multi_motor_control_node")
{
    declare_parameters_custom();
    load_parameters();
    validate_parameters();
    param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MultiMotorControlNode::on_parameters_changed, this, std::placeholders::_1));


    init_buses();
    setup_dispatchers();
    init_motors();
    enable_all_motors();


    cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/motor_commands",
        10,
        std::bind(&MultiMotorControlNode::command_callback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/motor_states",
        10);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / update_rate_));

    timer_ = this->create_wall_timer(
        period,
        std::bind(&MultiMotorControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "multi_motor_control_node started.");
}


MultiMotorControlNode::~MultiMotorControlNode()
{
    shutdown_motors();
}

void MultiMotorControlNode::declare_parameters_custom()
{
    this->declare_parameter<std::vector<std::string>>(
        "can_interfaces",
        std::vector<std::string>{"can0", "can1"});

    this->declare_parameter<double>("update_rate", 100.0);
    this->declare_parameter<bool>("interpolation_enabled", true);
    this->declare_parameter<double>("default_interp_duration", 1.0);

    this->declare_parameter<std::string>("motor_ids_can0", "[1, 2, 5, 6]");
    this->declare_parameter<std::string>("motor_ids_can1", "[3, 4, 7, 8]");

    this->declare_parameter<std::string>("motor_types_can0",
        "['DM4340', 'DM4340', 'DM4340', 'DM4340']");

    this->declare_parameter<std::string>("motor_types_can1",
        "['ODRIVE', 'ODRIVE', 'ODRIVE', 'ODRIVE']");

    this->declare_parameter<std::string>("joint_limit_keys", "[]");
    this->declare_parameter<std::string>("joint_q_mins", "[]");
    this->declare_parameter<std::string>("joint_q_maxs", "[]");
    this->declare_parameter<double>("default_limit_margin", 0.02);
}

void MultiMotorControlNode::load_parameters()
{
    can_interfaces_      = this->get_parameter("can_interfaces").as_string_array();
    update_rate_         = this->get_parameter("update_rate").as_double();
    interpolation_enabled_ = this->get_parameter("interpolation_enabled").as_bool();
    default_interp_duration_ = this->get_parameter("default_interp_duration").as_double();
    motor_ids_can0_raw_   = this->get_parameter("motor_ids_can0").as_string();
    motor_ids_can1_raw_   = this->get_parameter("motor_ids_can1").as_string();
    motor_types_can0_raw_ = this->get_parameter("motor_types_can0").as_string();
    motor_types_can1_raw_ = this->get_parameter("motor_types_can1").as_string();

    motor_ids_can0_   = parse_int_array_string(motor_ids_can0_raw_);
    motor_ids_can1_   = parse_int_array_string(motor_ids_can1_raw_);
    motor_types_can0_ = parse_string_array_string(motor_types_can0_raw_);
    motor_types_can1_ = parse_string_array_string(motor_types_can1_raw_);

    joint_limit_keys_raw_ = this->get_parameter("joint_limit_keys").as_string();
    joint_q_mins_raw_     = this->get_parameter("joint_q_mins").as_string();
    joint_q_maxs_raw_     = this->get_parameter("joint_q_maxs").as_string();
    default_limit_margin_ = this->get_parameter("default_limit_margin").as_double();

    load_joint_limits();
}

void MultiMotorControlNode::validate_parameters()
{
    if (can_interfaces_.empty())
    {
        throw std::runtime_error("Parameter 'can_interfaces' is empty.");
    }

    if (update_rate_ <= 0.0)
    {
        throw std::runtime_error("Parameter 'update_rate' must be > 0.");
    }

    if (motor_ids_can0_.size() != motor_types_can0_.size())
    {
        throw std::runtime_error("motor_ids_can0 size != motor_types_can0 size");
    }

    if (motor_ids_can1_.size() != motor_types_can1_.size())
    {
        throw std::runtime_error("motor_ids_can1 size != motor_types_can1 size");
    }

    for (const auto &t : motor_types_can0_)
    {
        if (t != "DM4340" && t != "ODRIVE")
        {
            throw std::runtime_error("Unsupported motor type in can0: " + t);
        }
    }

    for (const auto &t : motor_types_can1_)
    {
        if (t != "DM4340" && t != "ODRIVE")
        {
            throw std::runtime_error("Unsupported motor type in can1: " + t);
        }
    }

    if (default_limit_margin_ < 0.0)
    {
        throw std::runtime_error("Parameter 'default_limit_margin' must be >= 0.");
    }
}

void MultiMotorControlNode::init_buses()
{
    for (const auto &ifname : can_interfaces_)
    {
        BusContext bus;
        bus.ifname = ifname;
        bus.can = std::make_shared<CanPort>(ifname);
        bus.dispatcher = std::make_shared<CANDispatcher>(bus.can);
        bus.dm_ctrl = std::make_shared<damiao_can::Motor_Control>(bus.can);
        bus.odrive_driver = std::make_shared<ODriveDriver>();

        buses_.emplace(ifname, std::move(bus));

        RCLCPP_INFO(this->get_logger(), "Initialized bus: %s", ifname.c_str());
    }
}

void MultiMotorControlNode::setup_dispatchers()
{
    for (auto &[ifname, bus] : buses_)
    {
        bus.dispatcher->register_callback(
            [this, ifname](const CANFrame &frame)
            {
                auto &ctx = buses_.at(ifname);
                ctx.dm_ctrl->process_frame(frame.id, frame.data, frame.len);
            });

        bus.dispatcher->register_callback(
            [this, ifname](const CANFrame &frame)
            {
                auto &ctx = buses_.at(ifname);
                ctx.odrive_driver->process_frame(frame.id, frame.data, frame.len);
            });
    }
}

void MultiMotorControlNode::init_motors()
{
    auto add_bus_motors =
        [this](const std::string &ifname,
               const std::vector<int64_t> &ids,
               const std::vector<std::string> &types)
    {
        if (buses_.find(ifname) == buses_.end())
        {
            return;
        }

        auto &bus = buses_.at(ifname);

        for (size_t i = 0; i < ids.size(); ++i)
        {
            const int id = static_cast<int>(ids[i]);
            const auto &type = types[i];
            const std::string key = make_key(ifname, id);

            if (type == "DM4340")
            {
                auto motor = std::make_shared<damiao_can::Motor>(
                    damiao_can::DM4340,
                    static_cast<uint32_t>(id),
                    0x01);

                bus.dm_ctrl->addMotor(motor.get());

                DmMotorHandle handle;
                handle.can_if = ifname;
                handle.motor_id = id;
                handle.motor = motor;

                dm_motors_[key] = handle;

                RCLCPP_INFO(this->get_logger(),
                            "Added DM motor: %s (id=%d)",
                            ifname.c_str(), id);
            }
            else if (type == "ODRIVE")
            {
                auto axis = std::make_shared<ODriveAxis>(
                    bus.can,
                    static_cast<uint8_t>(id));

                OdriveMotorHandle handle;
                handle.can_if = ifname;
                handle.motor_id = id;
                handle.axis = axis;

                odrive_motors_[key] = handle;
                handle.axis->set_idle();

                OdriveCommand cmd;
                cmd.control_mode = CONTROL_MODE_VOLTAGE_CONTROL;
                cmd.input_mode = INPUT_MODE_PASSTHROUGH;
                cmd.pos = 0.0f;
                cmd.vel = 0.0f;
                cmd.torque = 0.0f;

                cmd.pos_input_enabled = false;
                cmd.vel_input_enabled = false;
                cmd.torque_input_enabled = false;

                cmd.mode_initialized = false;

                cmd.calibration_required = true;
                cmd.calibration_requested = false;
                cmd.calibrated = false;
                cmd.calibration_failed = false;

                cmd.active = false;
                cmd.closed_loop_requested = false;
                cmd.calibration_request_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

                odrive_cmds_[key] = cmd;

                RCLCPP_INFO(this->get_logger(),
                            "Added ODrive axis: %s (axis=%d, calibration_required=%d)",
                            ifname.c_str(), id, cmd.calibration_required ? 1 : 0);
            }
        }
    };

    if (buses_.count("can0"))
    {
        add_bus_motors("can0", motor_ids_can0_, motor_types_can0_);
    }

    if (buses_.count("can1"))
    {
        add_bus_motors("can1", motor_ids_can1_, motor_types_can1_);
    }
}

void MultiMotorControlNode::enable_all_motors()
{
    for (auto &[key, handle] : dm_motors_)
    {
        auto &bus = buses_.at(handle.can_if);
        bus.dm_ctrl->enable(*handle.motor);

        RCLCPP_INFO(this->get_logger(),
                    "Sent enable command to DM motor %s",
                    key.c_str());
    }

    for (auto &[key, handle] : odrive_motors_)
    {
        handle.axis->clear_errors();

        auto &cmd = odrive_cmds_[key];
        cmd.mode_initialized = false;
        cmd.calibration_requested = false;
        cmd.calibrated = false;
        cmd.calibration_failed = false;
        cmd.active = false;
        cmd.closed_loop_requested = false;
        cmd.calibration_request_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        RCLCPP_INFO(this->get_logger(),
                    "ODrive axis %s waiting for calibration before enable",
                    key.c_str());
    }

    enabled_ = true;
}

void MultiMotorControlNode::control_loop()
{
    // 1. last round feedback
    for (auto &[ifname, bus] : buses_)
    {
        (void)ifname;
        bus.dispatcher->poll(512);
    }

    // 2. DM refresh_motor_status
    for (auto &[key, handle] : dm_motors_)
    {
        auto &bus = buses_.at(handle.can_if);
        bus.dm_ctrl->refresh_motor_status(*handle.motor);
    }

/*     // 2.5 (debug use) DM homing
    if (dm_homing_active_)
    {
        bool all_dm_homed = true;

        for (auto &[key, handle] : dm_motors_)
        {
            auto &bus = buses_.at(handle.can_if);

            const float q  = handle.motor->Get_Position();
            const float dq = handle.motor->Get_Velocity();

            const float pos_err = static_cast<float>(dm_home_target_) - q;

            // 持续发 MIT 命令拉回 0
            bus.dm_ctrl->control_mit(
                *handle.motor,
                static_cast<float>(dm_home_kp_),
                static_cast<float>(dm_home_kd_),
                static_cast<float>(dm_home_target_),
                0.0f,
                0.0f);

            // 判断是否到位
            if (std::fabs(pos_err) > dm_home_pos_tol_ ||
                std::fabs(dq) > dm_home_vel_tol_)
            {
                all_dm_homed = false;
            }
        }

        if (all_dm_homed)
        {
            dm_homing_active_ = false;
            RCLCPP_INFO(this->get_logger(), "All DM motors homed to zero.");
        }
    } */

    // 2, damiao
    const double dt = 1.0 / update_rate_;

    for (auto &[key, handle] : dm_motors_)
    {
        if (!has_active_interpolation(key))
        {
            continue;
        }

        auto &bus = buses_.at(handle.can_if);

        auto interp = update_joint_interpolation(key, dt);
        double q_send = interp.q_cmd;
        const double dq_send = interp.dq_cmd;

        q_send = clamp_position_command(key, q_send);

        const double q_actual = static_cast<double>(handle.motor->Get_Position());
        if (is_position_out_of_bounds(key, q_actual))
        {
            stop_motor_safely(key);
            continue;
        }

        // DM MIT Control，simple PD + feedforward, to be tuned
        const float kp = 10.0f;
        const float kd = 0.8f;
        const float tau_ff = 0.0f;

        bus.dm_ctrl->control_mit(*handle.motor,
                                 kp, kd,
                                 static_cast<float>(q_send),
                                 static_cast<float>(dq_send),
                                 tau_ff);
    }


     // 3. ODrive state machine and control
    for (auto &[key, handle] : odrive_motors_)
    {
        auto cmd_it = odrive_cmds_.find(key);
        if (cmd_it == odrive_cmds_.end())
        {
            continue;
        }

        auto &cmd = cmd_it->second;
        auto &bus = buses_.at(handle.can_if);
        const auto axis_id = static_cast<uint8_t>(handle.motor_id);

        auto heartbeat = bus.odrive_driver->get_heartbeat(axis_id);

        // calibration
        update_odrive_calibration(key, handle, cmd, heartbeat);

        // if failed or not calibrated, skip the rest
        if (!cmd.calibrated || cmd.calibration_failed)
        {
            continue;
        }

        // set odrive axis state
        set_odrive_axis_command_mode(key, handle, cmd, heartbeat);

        // it is only availalbe to send commands when in closed loop control(and it must be calibrated)
        if (!heartbeat.valid ||
            heartbeat.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            continue;
        }

        if (cmd.pos_input_enabled)
        {
            double q_send = cmd.pos;
            int16_t vel_ff_send = cmd.vel_ff;

            if (has_active_interpolation(key))
            {
                auto interp = update_joint_interpolation(key, dt);
                q_send = interp.q_cmd;
                vel_ff_send = static_cast<int16_t>(interp.dq_cmd);
            }
            else{
                q_send = cmd.pos;
                vel_ff_send = cmd.vel_ff;
            }
            q_send = clamp_position_command(key, q_send);

            const double q_actual = static_cast<double>(bus.odrive_driver->get_encoder(axis_id).pos);
            if (is_position_out_of_bounds(key, q_actual))
            {
                stop_motor_safely(key);
                continue;
            }

            handle.axis->set_input_pos(static_cast<float>(q_send), vel_ff_send, cmd.torque_ff);
        }
        else if (cmd.vel_input_enabled)
        {
            const double q_actual = static_cast<double>(bus.odrive_driver->get_encoder(axis_id).pos);
            if (is_position_out_of_bounds(key, q_actual))
            {
                stop_motor_safely(key);
                continue;
            }
            handle.axis->set_input_vel(cmd.vel, cmd.torque);
        }
        else if (cmd.torque_input_enabled)
        {
            handle.axis->set_input_torque(cmd.torque);
        }
    }

    // 4. feedback after control
    for (auto &[ifname, bus] : buses_)
    {
        (void)ifname;
        bus.dispatcher->poll(512);
    } 

    // 5. output to ROS topic
    publish_log_states();
}

void MultiMotorControlNode::publish_log_states()
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    // DM state
    for (auto &[key, handle] : dm_motors_)
    {
        const float q   = handle.motor->Get_Position();
        const float dq  = handle.motor->Get_Velocity();
        const float tau = handle.motor->Get_tau();

        msg.name.push_back(key);
        msg.position.push_back(static_cast<double>(q));
        msg.velocity.push_back(static_cast<double>(dq));
        msg.effort.push_back(static_cast<double>(tau));

        RCLCPP_DEBUG(
            this->get_logger(),
            "[DM] %s q=%.6f dq=%.6f tau=%.6f",
            key.c_str(), q, dq, tau);
    }

    // ODrive state
    for (auto &[key, handle] : odrive_motors_)
    {
        auto &bus = buses_.at(handle.can_if);
        const auto axis_id = static_cast<uint8_t>(handle.motor_id);

        auto encoder   = bus.odrive_driver->get_encoder(axis_id);
        auto heartbeat = bus.odrive_driver->get_heartbeat(axis_id);
        auto torque    = bus.odrive_driver->get_torque(axis_id);

        msg.name.push_back(key);
        msg.position.push_back(static_cast<double>(encoder.pos));
        msg.velocity.push_back(static_cast<double>(encoder.vel));
        msg.effort.push_back(static_cast<double>(torque.torque_estimate));

        RCLCPP_DEBUG(
            this->get_logger(),
            "[ODRIVE] %s q=%.6f"
            "  dq=%.6f"
            "  tau: %.6f"
            "  torque_target: %.6f\n"
            "  active_errors: %u\n"
            "  axis_state: %d\n"
            "  procedure_result: %d\n"
            "  trajectory_done_flag: %s",
            key.c_str(),
            encoder.pos,
            encoder.vel,
            torque.torque_estimate,
            torque.torque_target,
            heartbeat.active_errors,
            static_cast<int>(heartbeat.axis_state),
            static_cast<int>(heartbeat.procedure_result),
            heartbeat.trajectory_done ? "true" : "false");
    }

    state_pub_->publish(msg);
}

void MultiMotorControlNode::shutdown_motors()
{

    RCLCPP_INFO(this->get_logger(), "Shutting down motors...");

    if (timer_)
    {
        timer_->cancel();
    }

    // ODrive stop commands and then idle
    for (auto &[key, handle] : odrive_motors_)
    {
        try
        {
            auto cmd_it = odrive_cmds_.find(key);
            if (cmd_it != odrive_cmds_.end())
            {
                cmd_it->second.active = false;
                cmd_it->second.closed_loop_requested = false;
                cmd_it->second.mode_initialized = false;
            }

            handle.axis->set_idle();

            RCLCPP_INFO(this->get_logger(),
                        "ODrive set idle: %s",
                        key.c_str());
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set ODrive idle: %s",
                        key.c_str());
        }
    }

    for (auto &[ifname, bus] : buses_)
    {
        (void)ifname;
        bus.dispatcher->poll(512);
    }

    // DM disable
    for (auto &[key, handle] : dm_motors_)
    {
        try
        {
            auto &bus = buses_.at(handle.can_if);
            bus.dm_ctrl->disable(*handle.motor);

            RCLCPP_INFO(this->get_logger(),
                        "Disabled DM motor: %s",
                        key.c_str());
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to disable DM motor: %s",
                        key.c_str());
        }
    }

    for (auto &[ifname, bus] : buses_)
    {
        (void)ifname;
        bus.dispatcher->poll();
    }

    enabled_ = false;
}

void MultiMotorControlNode::update_odrive_calibration(
    const std::string &key,
    OdriveMotorHandle &handle,
    OdriveCommand &cmd,
    const ODriveDriver::HeartbeatState &hb)
{
    if (!cmd.calibration_required || cmd.calibrated || cmd.calibration_failed)
    {
        return;
    }

    if (!hb.valid)
    {
        return;
    }

    const auto now = this->now();

    // calibration request
    if (!cmd.calibration_requested)
    {
        handle.axis->clear_errors();
        handle.axis->set_calibration();

        cmd.calibration_requested = true;
        cmd.calibration_request_time = now;
        cmd.mode_initialized = false;
        cmd.closed_loop_requested = false;
        cmd.active = false;

        RCLCPP_INFO(this->get_logger(),
                    "Requested ODrive calibration: %s",
                    key.c_str());
        return;
    }

    if ((now - cmd.calibration_request_time).seconds() < 1.0)
    {
        return;
    }

    // wait IDLE
    if (hb.axis_state != AXIS_STATE_IDLE)
    {
        return;
    }

    // IDLE check results
    if (hb.active_errors == 0 &&
        hb.procedure_result == PROCEDURE_RESULT_SUCCESS)
    {
        cmd.calibrated = true;
        cmd.active = true; 
        cmd.mode_initialized = false;
        cmd.closed_loop_requested = false;

        RCLCPP_INFO(this->get_logger(),
                    "ODrive calibration completed successfully: %s",
                    key.c_str());
    }
    else
    {
        cmd.calibration_failed = true;
        cmd.active = false;

        RCLCPP_ERROR(this->get_logger(),
                     "ODrive calibration failed: %s, active_errors=%u, procedure_result=%d",
                     key.c_str(),
                     hb.active_errors,
                     static_cast<int>(hb.procedure_result));
    }
}

void MultiMotorControlNode::set_odrive_axis_command_mode(
    const std::string &key,
    OdriveMotorHandle &handle,
    OdriveCommand &cmd,
    const ODriveDriver::HeartbeatState &hb)
{
    (void)key;

    if (!hb.valid)
    {
        return;
    }

    // not calibrated，not allowed to enable(firmware will reject closed loop command if not calibrated, but we can still set idle)
    if (!cmd.calibrated)
    {
        return;
    }

    // inactive -> idle
    if (!cmd.active)
    {
        handle.axis->set_idle();
        return;
    }

    // clear_errors
    if (hb.active_errors != 0)
    {
        handle.axis->clear_errors();
        cmd.mode_initialized = false;
        cmd.closed_loop_requested = false;
        return;
    }

    // no input -> idle
    if (!cmd.pos_input_enabled &&
        !cmd.vel_input_enabled &&
        !cmd.torque_input_enabled)
    {
        handle.axis->set_idle();
        return;
    }

    // select control mode
    ODriveControlMode desired_mode = CONTROL_MODE_VELOCITY_CONTROL;

    if (cmd.pos_input_enabled)
    {
        desired_mode = CONTROL_MODE_POSITION_CONTROL;
    }
    else if (cmd.vel_input_enabled)
    {
        desired_mode = CONTROL_MODE_VELOCITY_CONTROL;
    }
    else if (cmd.torque_input_enabled)
    {
        desired_mode = CONTROL_MODE_TORQUE_CONTROL;
    }

    if (!cmd.mode_initialized || cmd.control_mode != desired_mode)
    {
        cmd.control_mode = desired_mode;

        handle.axis->set_controller_mode(cmd.control_mode, INPUT_MODE_PASSTHROUGH);
        handle.axis->clear_errors();

        cmd.mode_initialized = true;
        cmd.closed_loop_requested = false;
    }

    if (!cmd.closed_loop_requested &&
        hb.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
        handle.axis->set_closed_loop();
        cmd.closed_loop_requested = true;
        return;
    }
}

void MultiMotorControlNode::set_odrive_velocity(const std::string &can_if,
                                                int id,
                                                float vel,
                                                float torque_ff)                                               
{
    const std::string key = make_key(can_if, id);
    auto it = odrive_cmds_.find(key);
    if (it == odrive_cmds_.end())
    {
        RCLCPP_WARN(this->get_logger(),
                    "ODrive target not found: %s",
                    key.c_str());
        return;
    }

    auto &cmd = it->second;
    cmd.pos_input_enabled = false;
    cmd.vel_input_enabled = true;
    cmd.torque_input_enabled = false;

    cmd.input_mode = INPUT_MODE_PASSTHROUGH;
    cmd.mode_initialized = false;
    cmd.closed_loop_requested = false;

    cmd.vel = vel;
    cmd.torque = torque_ff;
}

void MultiMotorControlNode::set_odrive_position(const std::string &can_if,
                                                int id,
                                                float pos,
                                                int16_t vel_ff,
                                                int16_t torque_ff)
{
    const std::string key = make_key(can_if, id);
    auto it = odrive_cmds_.find(key);
    if (it == odrive_cmds_.end())
    {
        RCLCPP_WARN(this->get_logger(),
                    "ODrive target not found: %s",
                    key.c_str());
        return;
    }

    auto &cmd = it->second;
    cmd.pos_input_enabled = true;
    cmd.vel_input_enabled = false;
    cmd.torque_input_enabled = false;

    cmd.input_mode = INPUT_MODE_PASSTHROUGH;
    cmd.mode_initialized = false;
    cmd.closed_loop_requested = false;

    cmd.pos = pos;
    cmd.vel_ff = vel_ff;
    cmd.torque_ff = torque_ff;
}

void MultiMotorControlNode::set_odrive_torque(const std::string &can_if,
                                              int id,
                                              float torque)
{
    const std::string key = make_key(can_if, id);
    auto it = odrive_cmds_.find(key);
    if (it == odrive_cmds_.end())
    {
        RCLCPP_WARN(this->get_logger(),
                    "ODrive target not found: %s",
                    key.c_str());
        return;
    }

    auto &cmd = it->second;
    cmd.pos_input_enabled = false;
    cmd.vel_input_enabled = false;
    cmd.torque_input_enabled = true;

    cmd.input_mode = INPUT_MODE_PASSTHROUGH;
    cmd.mode_initialized = false;
    cmd.closed_loop_requested = false;

    cmd.torque = torque;
}

std::string MultiMotorControlNode::make_key(const std::string &can_if, int id) const
{
    return can_if + ":" + std::to_string(id);
}

std::vector<int64_t> MultiMotorControlNode::parse_int_array_string(const std::string &text)
{
    std::vector<int64_t> result;
    std::string cleaned;

    for (char c : text)
    {
        if (c != '[' && c != ']' && c != ' ')
        {
            cleaned.push_back(c);
        }
    }

    if (cleaned.empty())
    {
        return result;
    }

    std::stringstream ss(cleaned);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            result.push_back(std::stoll(item));
        }
    }

    return result;
}

std::vector<std::string> MultiMotorControlNode::parse_string_array_string(const std::string &text)
{
    std::vector<std::string> result;
    std::string cleaned;

    for (char c : text)
    {
        if (c != '[' && c != ']' && c != ' ' && c != '\'' && c != '"')
        {
            cleaned.push_back(c);
        }
    }

    if (cleaned.empty())
    {
        return result;
    }

    std::stringstream ss(cleaned);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            result.push_back(item);
        }
    }

    return result;
}

void MultiMotorControlNode::command_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const size_t n = msg->name.size();

    if ((!msg->position.empty() && msg->position.size() != n) ||
        (!msg->velocity.empty() && msg->velocity.size() != n) ||
        (!msg->effort.empty() && msg->effort.size() != n))
    {
        RCLCPP_WARN(this->get_logger(),
                    "JointState command size mismatch.");
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        std::string can_if;
        int id = -1;
        if (!parse_joint_key(msg->name[i], can_if, id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid joint name format: %s",
                        msg->name[i].c_str());
            continue;
        }

        const bool has_pos =
            !msg->position.empty() && std::isfinite(msg->position[i]);
        const bool has_vel =
            !msg->velocity.empty() && std::isfinite(msg->velocity[i]);
        const bool has_eff =
            !msg->effort.empty() && std::isfinite(msg->effort[i]);

        const std::string key = make_key(can_if, id);

        // ---------- ODrive ----------
        auto odrive_it = odrive_motors_.find(key);
        if (odrive_it != odrive_motors_.end())
        {
            auto &bus = buses_.at(can_if);
            const uint8_t axis_id = static_cast<uint8_t>(id);
            auto encoder = bus.odrive_driver->get_encoder(axis_id);

            if (has_pos)
            {
                const double q_current = static_cast<double>(encoder.pos);
                const double q_target_raw = msg->position[i];
                const double q_target = clamp_position_command(key, q_target_raw);
                const double duration = default_interp_duration_;

                auto &cmd = odrive_cmds_[key];
                cmd.pos_input_enabled = true;
                cmd.vel_input_enabled = false;
                cmd.torque_input_enabled = false;
                cmd.input_mode = INPUT_MODE_PASSTHROUGH;
                cmd.mode_initialized = false;
                cmd.closed_loop_requested = false;
                cmd.pos = static_cast<float>(q_target);
                if(interpolation_enabled_)
                {
                    start_joint_interpolation(key, q_current, q_target, duration);
                }
                else{
                    auto &cmd = odrive_cmds_[key];
                    cmd.pos = static_cast<float>(q_target);
                }
            }
            else if (has_vel)
            {
                const double q_current = static_cast<double>(encoder.pos);
                const double v_cmd = clamp_velocity_command(key, q_current, msg->velocity[i]);
                const float torque_ff = has_eff ? static_cast<float>(msg->effort[i]) : 0.0f;
                set_odrive_velocity(can_if, id, static_cast<float>(v_cmd), torque_ff);
            }
            else if (has_eff)
            {
                set_odrive_torque(can_if, id, static_cast<float>(msg->effort[i]));
            }
            else
            {
                // 如果没有任何有效输入，则切换到 idle 模式
                auto &cmd = odrive_cmds_[key];
                cmd.pos_input_enabled = false;
                cmd.vel_input_enabled = false;
                cmd.torque_input_enabled = false;
                cmd.active = false;
                cmd.mode_initialized = false;
                cmd.closed_loop_requested = false;

                RCLCPP_INFO(this->get_logger(),
                            "Set ODrive %s to idle (no valid command input)",
                            key.c_str());
            }

            continue;
        }

        // ---------- DM ----------
        auto dm_it = dm_motors_.find(key);
        if (dm_it != dm_motors_.end())
        {
            auto &handle = dm_it->second;
            auto &bus = buses_.at(handle.can_if);

            if (has_pos)
            {
                const double q_current = static_cast<double>(handle.motor->Get_Position());
                const double q_target_raw = msg->position[i];
                const double q_target = clamp_position_command(key, q_target_raw);
                const double duration = default_interp_duration_;
                if(interpolation_enabled_)
                {
                    start_joint_interpolation(key, q_current, q_target, duration);
                }
                else
                {
                    const float kp = 10.0f;
                    const float kd = 0.8f;
                    bus.dm_ctrl->control_mit(*handle.motor, kp, kd, q_target, 0.0f, 0.0f);
                }
            }
            else if (has_vel)
            {
                const double q_current = static_cast<double>(handle.motor->Get_Position());
                const double v_cmd = clamp_velocity_command(key, q_current, msg->velocity[i]);
                bus.dm_ctrl->control_vel(*handle.motor, static_cast<float>(v_cmd));
            }
            else if (has_eff)
            {
                const float q = handle.motor->Get_Position();
                const float dq = handle.motor->Get_Velocity();
                float tau = static_cast<float>(msg->effort[i]);

                double q_min_soft = 0.0;
                double q_max_soft = 0.0;
                if (get_soft_position_bounds(key, q_min_soft, q_max_soft))
                {
                    if ((q >= q_max_soft && tau > 0.0f) ||
                        (q <= q_min_soft && tau < 0.0f))
                    {
                        RCLCPP_WARN(this->get_logger(),
                                    "Torque command blocked at position limit for %s: q=%.6f, tau=%.6f",
                                    key.c_str(), static_cast<double>(q), static_cast<double>(tau));
                        tau = 0.0f;
                    }
                }

                const float kp = 0.0f;
                const float kd = 0.0f;
                bus.dm_ctrl->control_mit(*handle.motor, kp, kd, q, dq, tau);
            }
            else
            {
                bus.dm_ctrl->disable(*handle.motor);

                RCLCPP_INFO(this->get_logger(),
                            "Disabled DM motor %s (no valid command input)",
                            key.c_str());
            }

            continue;
        }

        RCLCPP_WARN(this->get_logger(),
                    "Command target not found: %s",
                    key.c_str());
    }
}

void MultiMotorControlNode::start_joint_interpolation(const std::string &key,
                                                      double q_start,
                                                      double q_target,
                                                      double duration)
{
    JointInterpolation interp;
    interp.active = true;
    interp.q_start = clamp_position_command(key, q_start);
    interp.q_target = clamp_position_command(key, q_target);
    interp.duration = std::max(0.001, duration);
    interp.elapsed = 0.0;
    interp.q_cmd = interp.q_start;
    interp.dq_cmd = 0.0;

    joint_interpolators_[key] = interp;

    RCLCPP_INFO(this->get_logger(),
                "Start cubic interpolation: %s q_start=%.6f q_target=%.6f duration=%.3f",
                key.c_str(), interp.q_start, interp.q_target, duration);
}

MultiMotorControlNode::JointInterpolation
MultiMotorControlNode::update_joint_interpolation(const std::string &key, double dt)
{
    auto it = joint_interpolators_.find(key);
    if (it == joint_interpolators_.end())
    {
        return {};
    }

    auto &interp = it->second;

    if (!interp.active)
    {
        return interp;
    }

    interp.elapsed += dt;

    if (interp.elapsed >= interp.duration)
    {
        interp.elapsed = interp.duration;
        interp.q_cmd = interp.q_target;
        interp.dq_cmd = 0.0;
        interp.active = false;
        return interp;
    }

    const double T = interp.duration;
    const double s = interp.elapsed / T;
    const double dq = interp.q_target - interp.q_start;

    // 三次多项式
    interp.q_cmd = interp.q_start + dq * (3.0 * s * s - 2.0 * s * s * s);
    interp.dq_cmd = dq * (6.0 * s - 6.0 * s * s) / T;

    return interp;
}

bool MultiMotorControlNode::has_active_interpolation(const std::string &key) const
{
    auto it = joint_interpolators_.find(key);
    if (it == joint_interpolators_.end())
    {
        return false;
    }
    return it->second.active;
}

rcl_interfaces::msg::SetParametersResult
MultiMotorControlNode::on_parameters_changed(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &p : params)
    {
        if (p.get_name() == "interpolation_enabled")
        {
            interpolation_enabled_ = p.as_bool();
            RCLCPP_INFO(this->get_logger(),
                        "interpolation_enabled set to %s",
                        interpolation_enabled_ ? "true" : "false");
        }
        else if (p.get_name() == "default_interp_duration")
        {
            default_interp_duration_ = p.as_double();
            RCLCPP_INFO(this->get_logger(),
                        "default_interp_duration set to %.3f",
                        default_interp_duration_);
        }
    }

    return result;
}

std::vector<double> MultiMotorControlNode::parse_double_array_string(const std::string &text)
{
    std::vector<double> result;
    std::string cleaned;

    for (char c : text)
    {
        if (c != '[' && c != ']' && c != ' ')
        {
            cleaned.push_back(c);
        }
    }

    if (cleaned.empty())
    {
        return result;
    }

    std::stringstream ss(cleaned);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            result.push_back(std::stod(item));
        }
    }

    return result;
}

void MultiMotorControlNode::load_joint_limits()
{
    joint_limits_.clear();

    const auto keys  = parse_string_array_string(joint_limit_keys_raw_);
    const auto qmins = parse_double_array_string(joint_q_mins_raw_);
    const auto qmaxs = parse_double_array_string(joint_q_maxs_raw_);

    if (keys.size() != qmins.size() || keys.size() != qmaxs.size())
    {
        throw std::runtime_error(
            "joint_limit_keys, joint_q_mins, joint_q_maxs size mismatch");
    }

    for (size_t i = 0; i < keys.size(); ++i)
    {
        if (qmins[i] >= qmaxs[i])
        {
            throw std::runtime_error("Invalid joint limit for " + keys[i] + ": q_min >= q_max");
        }

        JointLimits lim;
        lim.enabled = true;
        lim.q_min = qmins[i];
        lim.q_max = qmaxs[i];
        lim.margin = default_limit_margin_;

        joint_limits_[keys[i]] = lim;

        RCLCPP_INFO(this->get_logger(),
                    "Loaded soft limit for %s: [%.6f, %.6f], margin=%.6f",
                    keys[i].c_str(), lim.q_min, lim.q_max, lim.margin);
    }
}

bool MultiMotorControlNode::get_soft_position_bounds(const std::string &key,
                                                     double &q_min_soft,
                                                     double &q_max_soft) const
{
    auto it = joint_limits_.find(key);
    if (it == joint_limits_.end() || !it->second.enabled)
    {
        return false;
    }

    q_min_soft = it->second.q_min + it->second.margin;
    q_max_soft = it->second.q_max - it->second.margin;

    if (q_min_soft > q_max_soft)
    {
        q_min_soft = it->second.q_min;
        q_max_soft = it->second.q_max;
    }

    return true;
}

double MultiMotorControlNode::clamp_position_command(const std::string &key, double q_cmd) const
{
    double q_min_soft = 0.0;
    double q_max_soft = 0.0;

    if (!get_soft_position_bounds(key, q_min_soft, q_max_soft))
    {
        return q_cmd;
    }

    const double q_clamped = std::clamp(q_cmd, q_min_soft, q_max_soft);

    if (q_clamped != q_cmd)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Position command clamped for %s: raw=%.6f -> clamped=%.6f, bounds=[%.6f, %.6f]",
                    key.c_str(), q_cmd, q_clamped, q_min_soft, q_max_soft);
    }

    return q_clamped;
}

double MultiMotorControlNode::clamp_velocity_command(const std::string &key,
                                                     double q_current,
                                                     double v_cmd) const
{
    double q_min_soft = 0.0;
    double q_max_soft = 0.0;

    if (!get_soft_position_bounds(key, q_min_soft, q_max_soft))
    {
        return v_cmd;
    }

    if (q_current >= q_max_soft && v_cmd > 0.0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Velocity command blocked at upper limit for %s: q=%.6f, v=%.6f",
                    key.c_str(), q_current, v_cmd);
        return 0.0;
    }

    if (q_current <= q_min_soft && v_cmd < 0.0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Velocity command blocked at lower limit for %s: q=%.6f, v=%.6f",
                    key.c_str(), q_current, v_cmd);
        return 0.0;
    }

    return v_cmd;
}

bool MultiMotorControlNode::is_position_out_of_bounds(const std::string &key, double q_current) const
{
    auto it = joint_limits_.find(key);
    if (it == joint_limits_.end() || !it->second.enabled)
    {
        return false;
    }

    return (q_current < it->second.q_min || q_current > it->second.q_max);
}

void MultiMotorControlNode::stop_motor_safely(const std::string &key)
{
    auto odrive_it = odrive_motors_.find(key);
    if (odrive_it != odrive_motors_.end())
    {
        try
        {
            auto &handle = odrive_it->second;
            auto &cmd = odrive_cmds_[key];
            cmd.pos_input_enabled = false;
            cmd.vel_input_enabled = false;
            cmd.torque_input_enabled = false;
            cmd.active = false;
            cmd.mode_initialized = false;
            cmd.closed_loop_requested = false;
            handle.axis->set_idle();

            RCLCPP_ERROR(this->get_logger(),
                         "ODrive stopped due to limit violation: %s",
                         key.c_str());
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to stop ODrive safely: %s",
                        key.c_str());
        }
        return;
    }

    auto dm_it = dm_motors_.find(key);
    if (dm_it != dm_motors_.end())
    {
        try
        {
            auto &handle = dm_it->second;
            auto &bus = buses_.at(handle.can_if);
            bus.dm_ctrl->disable(*handle.motor);

            RCLCPP_ERROR(this->get_logger(),
                         "DM motor disabled due to limit violation: %s",
                         key.c_str());
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to disable DM safely: %s",
                        key.c_str());
        }
        return;
    }
}

bool MultiMotorControlNode::parse_joint_key(const std::string &name,
                                            std::string &can_if,
                                            int &id) const
{
    const auto pos = name.find(':');
    if (pos == std::string::npos)
    {
        return false;
    }

    can_if = name.substr(0, pos);

    try
    {
        id = std::stoi(name.substr(pos + 1));
    }
    catch (...)
    {
        return false;
    }

    return true;
}