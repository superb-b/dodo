#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>


#include "can_port.hpp"
#include "can_dispatcher.hpp"
#include "damiao_can.hpp"
#include "odrive_driver.hpp"
#include "odrive_axis.hpp"
#include "odrive_enums.h"


struct BusContext
{
    std::string ifname;
    CanPort::SharedPtr can;
    std::shared_ptr<CANDispatcher> dispatcher;
    std::shared_ptr<damiao_can::Motor_Control> dm_ctrl;
    std::shared_ptr<ODriveDriver> odrive_driver;
};

struct DmMotorHandle
{
    std::string can_if;
    int motor_id;
    std::shared_ptr<damiao_can::Motor> motor;
};

struct OdriveMotorHandle
{
    std::string can_if;
    int motor_id;
    std::shared_ptr<ODriveAxis> axis;
};

// Odrive State and Command
struct OdriveCommand
{
    ODriveControlMode control_mode{CONTROL_MODE_VELOCITY_CONTROL};
    ODriveInputMode input_mode{INPUT_MODE_PASSTHROUGH};

    float pos{0.0f};
    float vel{0.0f};
    float torque{0.0f};

    int16_t vel_ff{0};
    int16_t torque_ff{0};

    bool pos_input_enabled{false};
    bool vel_input_enabled{false};
    bool torque_input_enabled{false};

    bool mode_initialized{false};

    bool calibration_required{true};
    bool calibration_requested{false};
    bool calibrated{false};
    bool calibration_failed{false};

    bool active{false};
    bool closed_loop_requested{false};

    rclcpp::Time calibration_request_time;
};



struct JointLimits
{
    bool enabled = false;
    double q_min = 0.0;
    double q_max = 0.0;
    double margin = 0.02;
};

class MultiMotorControlNode : public rclcpp::Node
{
public:
    MultiMotorControlNode();
    ~MultiMotorControlNode() override;

private:

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> &params);


    void declare_parameters_custom();
    void load_parameters();
    void validate_parameters();

    void init_buses();
    void setup_dispatchers();
    void init_motors();
    void enable_all_motors();
    void control_loop();
    void publish_log_states();
    void shutdown_motors();

    std::string make_key(const std::string &can_if, int id) const;
    std::vector<int64_t> parse_int_array_string(const std::string &text);
    std::vector<std::string> parse_string_array_string(const std::string &text);

    // ODrive
    void set_odrive_axis_command_mode(const std::string& ,OdriveMotorHandle&,OdriveCommand&,const ODriveDriver::HeartbeatState&);
    void update_odrive_calibration(const std::string &,OdriveMotorHandle &,OdriveCommand &,const ODriveDriver::HeartbeatState &);
    void set_odrive_velocity(const std::string &can_if, int id, float vel, float torque_ff = 0.0f);
    void set_odrive_position(const std::string &can_if, int id, float pos, int16_t vel_ff = 0, int16_t torque_ff = 0);
    void set_odrive_torque(const std::string &can_if, int id, float torque);

    //node lifecycle

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool parse_joint_key(const std::string &name, std::string &can_if, int &id) const;

    bool interpolation_enabled_{true};
    double default_interp_duration_{1.0};
        struct JointInterpolation
    {
        bool active{false};

        double q_start{0.0};
        double q_target{0.0};

        double duration{1.0};
        double elapsed{0.0};

        double q_cmd{0.0};
        double dq_cmd{0.0};
    };

    std::vector<double> parse_double_array_string(const std::string &text);
    void load_joint_limits();
    double clamp_position_command(const std::string &key, double q_cmd) const;
    bool get_soft_position_bounds(const std::string &key, double &q_min_soft, double &q_max_soft) const;
    double clamp_velocity_command(const std::string &key, double q_current, double v_cmd) const;
    bool is_position_out_of_bounds(const std::string &key, double q_current) const;
    void stop_motor_safely(const std::string &key);
    
    
    void start_joint_interpolation(const std::string &key,
                                   double q_start,
                                   double q_target,
                                   double duration);

    JointInterpolation update_joint_interpolation(const std::string &key, double dt);
    bool has_active_interpolation(const std::string &key) const;

private:
    double update_rate_{100.0};
    std::vector<std::string> can_interfaces_;

    std::string motor_ids_can0_raw_;
    std::string motor_ids_can1_raw_;
    std::string motor_types_can0_raw_;
    std::string motor_types_can1_raw_;

    std::vector<int64_t> motor_ids_can0_;
    std::vector<int64_t> motor_ids_can1_;
    std::vector<std::string> motor_types_can0_;
    std::vector<std::string> motor_types_can1_;

    std::unordered_map<std::string, BusContext> buses_;
    std::unordered_map<std::string, DmMotorHandle> dm_motors_;
    std::unordered_map<std::string, JointLimits> joint_limits_;
    std::unordered_map<std::string, OdriveMotorHandle> odrive_motors_;
    std::unordered_map<std::string, OdriveCommand> odrive_cmds_;
    std::unordered_map<std::string, JointInterpolation> joint_interpolators_;

    std::string joint_limit_keys_raw_;
    std::string joint_q_mins_raw_;
    std::string joint_q_maxs_raw_;
    double default_limit_margin_ = 0.02;

    rclcpp::TimerBase::SharedPtr timer_;
    bool enabled_{false};
};