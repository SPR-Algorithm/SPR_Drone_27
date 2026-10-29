from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')
    show_ui = ParameterValue(LaunchConfiguration('show_ui'), value_type=bool)
    auto_save = ParameterValue(LaunchConfiguration('auto_save'), value_type=bool)
    min_pairs = ParameterValue(LaunchConfiguration('min_pairs'), value_type=int)
    board_cols = ParameterValue(LaunchConfiguration('board_cols'), value_type=int)
    board_rows = ParameterValue(LaunchConfiguration('board_rows'), value_type=int)
    square_size = ParameterValue(
        LaunchConfiguration('square_size'), value_type=float)
    blur_threshold = ParameterValue(
        LaunchConfiguration('blur_threshold'), value_type=float)
    edge_margin = ParameterValue(
        LaunchConfiguration('edge_margin'), value_type=float)
    diversity_translation = ParameterValue(
        LaunchConfiguration('diversity_translation'), value_type=float)
    diversity_rotation_deg = ParameterValue(
        LaunchConfiguration('diversity_rotation_deg'), value_type=float)
    save_cooldown = ParameterValue(
        LaunchConfiguration('save_cooldown'), value_type=float)
    detect_rate_hz = ParameterValue(
        LaunchConfiguration('detect_rate_hz'), value_type=float)
    exhaustive_detection = ParameterValue(
        LaunchConfiguration('exhaustive_detection'), value_type=bool)

    # Hardware is owned exclusively by drone_bringup. This launch only subscribes.
    capture = Node(
        package='calibration',
        executable='capture',
        name='calibration_capture',
        output='screen',
        parameters=[{
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_info_topic': LaunchConfiguration('left_info_topic'),
            'right_info_topic': LaunchConfiguration('right_info_topic'),
            'output_dir': output_dir,
            'board_cols': board_cols,
            'board_rows': board_rows,
            'square_size': square_size,
            'auto_orientation': True,
            'auto_save': auto_save,
            'min_pairs': min_pairs,
            'blur_threshold': blur_threshold,
            'edge_margin': edge_margin,
            'diversity_translation': diversity_translation,
            'diversity_rotation_deg': diversity_rotation_deg,
            'save_cooldown': save_cooldown,
            'detect_rate_hz': detect_rate_hz,
            'exhaustive_detection': exhaustive_detection,
            'show_ui': show_ui,
            'input_timeout_sec': ParameterValue(LaunchConfiguration('input_timeout_sec'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('left_image_topic', default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_info_topic', default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_info_topic', default_value='/camera/infra2/camera_info'),
        DeclareLaunchArgument('input_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'output_dir', default_value='data',
            description='Parent directory for capture sessions'),
        DeclareLaunchArgument(
            'show_ui', default_value='true',
            description='Show the OpenCV preview and coverage windows'),
        DeclareLaunchArgument(
            'auto_save', default_value='true',
            description='Automatically save pairs that pass all gates'),
        DeclareLaunchArgument(
            'min_pairs', default_value='30',
            description='Target number of accepted image pairs'),
        DeclareLaunchArgument(
            'board_cols', default_value='8',
            description='Checkerboard inner corners per row'),
        DeclareLaunchArgument(
            'board_rows', default_value='11',
            description='Checkerboard inner corners per column'),
        DeclareLaunchArgument(
            'square_size', default_value='0.025',
            description='Checkerboard square edge length in meters'),
        DeclareLaunchArgument(
            'blur_threshold', default_value='100.0',
            description='Minimum Laplacian variance of the board ROI'),
        DeclareLaunchArgument(
            'edge_margin', default_value='15.0',
            description='Minimum corner distance from the image edge in pixels'),
        DeclareLaunchArgument(
            'diversity_translation', default_value='0.03',
            description='Minimum board translation difference in meters'),
        DeclareLaunchArgument(
            'diversity_rotation_deg', default_value='8.0',
            description='Minimum board rotation difference in degrees'),
        DeclareLaunchArgument(
            'save_cooldown', default_value='0.5',
            description='Minimum seconds between automatic saves'),
        DeclareLaunchArgument(
            'detect_rate_hz', default_value='10.0',
            description='Maximum board detection rate'),
        DeclareLaunchArgument(
            'exhaustive_detection', default_value='false',
            description='Use slower exhaustive checkerboard detection'),
        capture,
    ])
