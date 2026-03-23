from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===== Launch Arguments =====
    can_interfaces_arg = DeclareLaunchArgument(
        'can_interfaces',
        default_value="['can0']",
        description='List of CAN interfaces to use (e.g., ["can0"])'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='100',
        description='Control loop update rate (Hz)'
    )

    # 左腿 CAN 电机 ID
    motor_ids_can0_arg = DeclareLaunchArgument(
        'motor_ids_can0',
        default_value='[1, 2, 5, 6]',
        description='Motor IDs on CAN0 (left leg)'
    )

    # 右腿 CAN 电机 ID
    motor_ids_can1_arg = DeclareLaunchArgument(
        'motor_ids_can1',
        default_value='[3, 4, 7, 8]',
        description='Motor IDs on CAN1 (right leg)'
    )

    # 电机类型（可选）
    motor_types_can0_arg = DeclareLaunchArgument(
        'motor_types_can0',
        default_value="['DM4340', 'DM4340', 'DM4340', 'DM4340']",
        description='Motor types for CAN0'
    )

    motor_types_can1_arg = DeclareLaunchArgument(
        'motor_types_can1',
        default_value="['ODRIVE', 'ODRIVE', 'ODRIVE', 'ODRIVE']",
        description='Motor types for CAN1'
    )
    joint_limit_keys= DeclareLaunchArgument(
        'joint_limit_keys',
        default_value="['can0:1', 'can0:2', 'can0:5', 'can0:6', 'can1:3', 'can1:4', 'can1:7', 'can1:8']",
        description='List of joint limits'
    )

    joint_q_mins = DeclareLaunchArgument(
        'joint_q_mins',
        default_value="[-0.05, 0          , 0.15507, -1.32, -0.617107, -5.542013, -4.7, -1.82]",
        description='Minimum joint positions'
    )

    joint_q_maxs = DeclareLaunchArgument(
        'joint_q_maxs',
        default_value="[0.097849, 1.613833, 0.215724, -0.18, 9, 2.072518, 3.238402, 6]",
        description='Maximum joint positions'
    )

    # ===== 启动节点 =====
    multi_motor_node = Node(
        package='dodo_canbus',
        executable='canbus_node',  # 或 multi_motor_control_node
        name='multi_motor_control_node',
        output='screen',
        parameters=[{
            'can_interfaces': ParameterValue(LaunchConfiguration('can_interfaces'), value_type=str),
            'motor_ids_can0': ParameterValue(LaunchConfiguration('motor_ids_can0'), value_type=str),
            'motor_ids_can1': ParameterValue(LaunchConfiguration('motor_ids_can1'), value_type=str),
            'motor_types_can0': ParameterValue(LaunchConfiguration('motor_types_can0'), value_type=str),
            'motor_types_can1': ParameterValue(LaunchConfiguration('motor_types_can1'), value_type=str),
            'update_rate': LaunchConfiguration('update_rate'),
            'joint_q_mins': ParameterValue(LaunchConfiguration('joint_q_mins'), value_type=str),
            'joint_q_maxs': ParameterValue(LaunchConfiguration('joint_q_maxs'), value_type=str),
            'joint_limit_keys': ParameterValue(LaunchConfiguration('joint_limit_keys'), value_type=str)
        }]
    )

    # ===== 返回 Launch Description =====
    return LaunchDescription([
        can_interfaces_arg,
        motor_ids_can0_arg,
        motor_ids_can1_arg,
        motor_types_can0_arg,
        motor_types_can1_arg,
        update_rate_arg,
        multi_motor_node,
        joint_q_mins,
        joint_q_maxs,
        joint_limit_keys
    ])
