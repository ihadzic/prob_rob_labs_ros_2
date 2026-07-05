#!/usr/bin/env python3

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _set_or_create_text(parent, tag, text):
    elem = parent.find(tag)
    if elem is None:
        elem = ET.SubElement(parent, tag)
    elem.text = text


def _find_diff_drive_plugin(model_elem):
    for plugin in model_elem.findall('plugin'):
        name = plugin.get('name', '')
        filename = plugin.get('filename', '')
        if 'DiffDrive' in name or 'diff-drive' in filename:
            return plugin
    return None


def _ensure_camera_noise(sensor_elem, stddev):
    camera_elem = sensor_elem.find('camera')
    if camera_elem is None:
        return
    noise_elem = camera_elem.find('noise')
    if noise_elem is None:
        noise_elem = ET.SubElement(camera_elem, 'noise')
    _set_or_create_text(noise_elem, 'type', 'gaussian')
    _set_or_create_text(noise_elem, 'mean', '0.0')
    _set_or_create_text(noise_elem, 'stddev', f'{stddev:.6f}')


def _scale_sensor_noise(model_elem, sensor_noise_scale):
    for sensor in model_elem.findall('.//sensor'):
        sensor_type = sensor.get('type', '')

        if sensor_type in ['camera', 'wideanglecamera']:
            _ensure_camera_noise(sensor, 0.012 * sensor_noise_scale)

        if sensor_type in ['gpu_lidar', 'lidar']:
            stddev_elem = sensor.find('./lidar/noise/stddev')
            if stddev_elem is not None and stddev_elem.text:
                base = float(stddev_elem.text)
                stddev_elem.text = f'{base * sensor_noise_scale:.6f}'

        if sensor_type == 'imu':
            for stddev_elem in sensor.findall('.//noise/stddev'):
                if stddev_elem.text:
                    base = float(stddev_elem.text)
                    stddev_elem.text = f'{base * sensor_noise_scale:.6f}'


def _set_contact_slip(model_elem, contact_slip):
    for link in model_elem.findall('.//link'):
        if not link.get('name', '').startswith('wheel_'):
            continue
        for collision in link.findall('collision'):
            ode_elem = collision.find('./surface/friction/ode')
            if ode_elem is None:
                continue
            _set_or_create_text(ode_elem, 'slip1', f'{contact_slip:.6f}')
            _set_or_create_text(ode_elem, 'slip2', f'{contact_slip:.6f}')


def _ensure_wheel_slip_plugin(model_elem, wheel_slip, wheel_radius, wheel_normal_force):
    existing = None
    for plugin in model_elem.findall('plugin'):
        if 'WheelSlip' in plugin.get('name', '') or 'wheel-slip' in plugin.get('filename', ''):
            existing = plugin
            break

    if existing is None:
        existing = ET.SubElement(
            model_elem,
            'plugin',
            attrib={
                'filename': 'gz-sim-wheel-slip-system',
                'name': 'gz::sim::systems::WheelSlip',
            },
        )

    for wheel_elem in list(existing.findall('wheel')):
        existing.remove(wheel_elem)

    for wheel_name in ['wheel_left_link', 'wheel_right_link']:
        wheel_elem = ET.SubElement(existing, 'wheel', attrib={'link_name': wheel_name})
        _set_or_create_text(wheel_elem, 'wheel_radius', f'{wheel_radius:.6f}')
        _set_or_create_text(wheel_elem, 'slip_compliance_lateral', f'{wheel_slip:.6f}')
        _set_or_create_text(wheel_elem, 'slip_compliance_longitudinal', f'{wheel_slip:.6f}')
        _set_or_create_text(wheel_elem, 'wheel_normal_force', f'{wheel_normal_force:.3f}')


def _patch_model_sdf(base_sdf_path, use_noisy_dynamics, sensor_noise_scale, wheel_slip, use_actuation_noise):
    tree = ET.parse(base_sdf_path)
    root = tree.getroot()
    model = root.find('model')
    if model is None:
        raise RuntimeError(f'No <model> found in SDF {base_sdf_path}')

    if use_noisy_dynamics:
        _scale_sensor_noise(model, sensor_noise_scale)

        # Keep small ODE slip in collision for continuous low-level wheel skid.
        _set_contact_slip(model, max(0.001, wheel_slip * 0.25))

        # Add dedicated wheel-slip system for physically grounded slipping behavior.
        diff_drive = _find_diff_drive_plugin(model)
        wheel_radius = 0.033
        if diff_drive is not None:
            wr = diff_drive.find('wheel_radius')
            if wr is not None and wr.text:
                wheel_radius = float(wr.text)

        # Approximate per-wheel normal force for TurtleBot3-sized robots.
        _ensure_wheel_slip_plugin(
            model,
            wheel_slip=wheel_slip,
            wheel_radius=wheel_radius,
            wheel_normal_force=18.0,
        )

    diff_drive = _find_diff_drive_plugin(model)
    if diff_drive is not None:
        _set_or_create_text(
            diff_drive,
            'topic',
            'cmd_vel_noisy' if use_actuation_noise else 'cmd_vel',
        )

    tmp = tempfile.NamedTemporaryFile(prefix='tb3_noisy_', suffix='.sdf', delete=False)
    tree.write(tmp.name, encoding='utf-8', xml_declaration=True)
    tmp.close()
    return tmp.name


def _build_bridge_config(model_folder, use_actuation_noise):
    src = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        f'{model_folder}_bridge.yaml',
    )

    with open(src, 'r', encoding='utf-8') as f:
        text = f.read()

    if use_actuation_noise:
        text = text.replace(
            'ros_topic_name: "cmd_vel"\n  gz_topic_name: "cmd_vel"',
            'ros_topic_name: "cmd_vel_noisy"\n  gz_topic_name: "cmd_vel_noisy"',
        )

    tmp = tempfile.NamedTemporaryFile(prefix='tb3_bridge_', suffix='.yaml', delete=False)
    with open(tmp.name, 'w', encoding='utf-8') as f:
        f.write(text)
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']
    model_folder = f'turtlebot3_{turtlebot3_model}'

    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    use_noisy_dynamics = LaunchConfiguration('use_noisy_dynamics').perform(context).lower() == 'true'
    use_actuation_noise = LaunchConfiguration('use_actuation_noise').perform(context).lower() == 'true'
    sensor_noise_scale = float(LaunchConfiguration('sensor_noise_scale').perform(context))
    wheel_slip = float(LaunchConfiguration('wheel_slip').perform(context))
    cmd_noise = float(LaunchConfiguration('cmd_noise').perform(context))

    model_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf',
    )

    patched_sdf = _patch_model_sdf(
        base_sdf_path=model_sdf,
        use_noisy_dynamics=use_noisy_dynamics,
        sensor_noise_scale=sensor_noise_scale,
        wheel_slip=wheel_slip,
        use_actuation_noise=use_actuation_noise,
    )

    bridge_config = _build_bridge_config(
        model_folder=model_folder,
        use_actuation_noise=use_actuation_noise,
    )

    actions = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', turtlebot3_model,
                '-file', patched_sdf,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
        ),
    ]

    if turtlebot3_model != 'burger':
        actions.append(
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=['/camera/image_raw'],
                output='screen',
            )
        )

    if use_actuation_noise:
        actions.append(
            Node(
                package='prob_rob_labs',
                executable='cmd_vel_noise_injector',
                name='cmd_vel_noise_injector',
                parameters=[
                    {'input_topic': '/cmd_vel'},
                    {'output_topic': '/cmd_vel_noisy'},
                    {'publish_rate_hz': 30.0},
                    {'actuation_noise_linear_std': cmd_noise},
                    {'actuation_noise_angular_std': cmd_noise},
                ],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0', description='TurtleBot3 X pose'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='TurtleBot3 Y pose'),
        DeclareLaunchArgument(
            'use_noisy_dynamics',
            default_value='true',
            description='Enable wheel slip + stronger sensor noise',
        ),
        DeclareLaunchArgument(
            'use_actuation_noise',
            default_value='true',
            description='Inject cmd_vel noise and idle jitter to create small in-place vibration',
        ),
        DeclareLaunchArgument(
            'sensor_noise_scale',
            default_value='2.0',
            description='Multiplier for existing IMU/LiDAR noise and camera noise baseline',
        ),
        DeclareLaunchArgument(
            'wheel_slip',
            default_value='0.06',
            description='Wheel slip compliance for Gazebo wheel-slip system',
        ),
        DeclareLaunchArgument(
            'cmd_noise',
            default_value='0.0',
            description='Velocity command noise',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
