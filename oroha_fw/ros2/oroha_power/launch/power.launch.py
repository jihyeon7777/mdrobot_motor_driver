from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    zero = LaunchConfiguration('zero_on_start')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('zero_on_start', default_value='false',
                              description='시작 시 영점 보정 — 반드시 정지 상태에서만 true'),
        Node(
            package='oroha_power',
            executable='power_node',
            name='oroha_power',
            output='screen',
            parameters=[{
                'port': port,
                'baud': 115200,
                'frame_id': 'oroha_power',
                'auto_start': True,
                'rate': 50,                # P<hz>. 100 은 상시 overrun — 보고서 20260813 §6
                'zero_on_start': zero,
                # ── as-built 상수 (설계 문서 §13.0) ──
                'v_per_lsb': 9.1312e-3,    # LSB_V × DIV_RATIO 11.3310 (2026-08-13 1 점 적합)
                'a_per_lsb': 12.21e-3,     # LSB_V ÷ 66.0 mV/A — ACS37030, 보고서 20260814 §7
                'scale_v': 1.0,            # 교정(C6) 후 갱신
                'scale_gp28': 0.9852,      # 2026-08-14 DMM 15 점 교정
                'scale_gp27': 1.0,         # 미교정
                'sign_gp28': 1,              # T3 에서 확인
                'sign_gp27': 1,
                'design_capacity': 20.0,
                'cell_count': 7,
            }],
        ),
    ])
