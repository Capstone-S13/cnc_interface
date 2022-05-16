from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    baud_rate = DeclareLaunchArgument(
        'baud_rate', default_value=TextSubstitution(text='115200')
    )

    port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACMO'
    )

    acc = DeclareLaunchArgument(
        'acc', default_value=TextSubstitution(text='1000')
    )
    x_max = DeclareLaunchArgument(
        'x_max', default_value=TextSubstitution(text='450')
    )

    y_max = DeclareLaunchArgument(
        'y_max', default_value=TextSubstitution(text='550')
    )

    z_max = DeclareLaunchArgument(
        'z_max', default_value=TextSubstitution(text='100')
    )

    default_speed = DeclareLaunchArgument(
        'default_speed', default_value=TextSubstitution(text='1000')
    )

    x_max_speed = DeclareLaunchArgument(
        'x_max_speed', default_value=TextSubstitution(text='1000')
    )

    y_max_speed = DeclareLaunchArgument(
        'y_max_speed', default_value=TextSubstitution(text='0')
    )

    z_max_speed = DeclareLaunchArgument(
        'z_max_speed', default_value=TextSubstitution(text='1000')
    )

    x_steps_mm = DeclareLaunchArgument(
        'x_steps_mm', default_value=TextSubstitution(text='2134')
    )

    y_steps_mm = DeclareLaunchArgument(
        'y_steps_mm', default_value=TextSubstitution(text='2134')
    )

    z_steps_mm = DeclareLaunchArgument(
        'z_steps_mm', default_value=TextSubstitution(text='2134')
    )


    return LaunchDescription([
        baud_rate,
        port,
        acc,
        x_max,
        y_max,
        z_max,
        default_speed,
        x_max_speed,
        y_max_speed,
        z_max_speed,
        x_steps_mm,
        y_steps_mm,
        z_steps_mm,
        Node(
            package='cnc_interface',
            executable='cnc_interface_node',
            name='cnc_interface_node',
            parameters=[
            #     {
            #     'baud_rate': LaunchConfiguration('baud_rate'),
            #     'port': LaunchConfiguration('port'),
            #     'acceleration': LaunchConfiguration('acc'),
            #     'x_max': LaunchConfiguration('x_max'),
            #     'y_max': LaunchConfiguration('y_max'),
            #     'z_max': LaunchConfiguration('z_max'),
            #     'default_speed': LaunchConfiguration('default_speed'),
            #     'x_max_speed': LaunchConfiguration('x_max_speed'),
            #     'y_max_speed': LaunchConfiguration('y_max_speed'),
            #     'z_max_speed': LaunchConfiguration('z_max_speed'),
            #     'x_steps_mm': LaunchConfiguration('x_steps_mm'),
            #     'y_steps_mm': LaunchConfiguration('y_steps_mm'),
            #     'z_steps_mm': LaunchConfiguration('z_steps_mm')
            # },
            {'port': '/dev/ttyACMO'},
			{'x_max': 1000}]
        ),
    ])