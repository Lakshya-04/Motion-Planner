"""ros2 launch sim_overlay coverage_paint.launch.py wall_scene_json:=<json> source:=mujoco|gazebo [feed:=none|gazebo|mujoco]

Starts the paint node (RViz: add a MarkerArray display on /mp_viz/coverage_paint, or mp_rviz profile) and, with feed:=gazebo or
feed:=mujoco, the existing gz_overlay.py / overlay_dump.py reading only that topic."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node

TOPICS = "['/mp_viz/coverage_paint']"


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in (
        ('wall_scene_json', ''), ('source', 'mujoco'), ('feed', 'none'), ('frame_id', ''), ('tcp_frame', 'tcp_link'),
        ('swath_m', '0.14'), ('plan_file', ''), ('mujoco_json', '/tmp/mp_sim_overlay.json'))]
    paint = Node(package='sim_overlay', executable='coverage_paint.py', name='coverage_paint', output='screen',
                 parameters=[{'wall_scene_json': LC('wall_scene_json'), 'source': LC('source'), 'frame_id': LC('frame_id'),
                              'tcp_frame': LC('tcp_frame'), 'swath_m': LC('swath_m'), 'plan_file': LC('plan_file')}])
    gz = Node(package='sim_overlay', executable='gz_overlay.py', name='gz_overlay', output='screen',
              parameters=[{'topics': ['/mp_viz/coverage_paint']}], condition=IfCondition(PythonExpression(["'", LC('feed'), "' == 'gazebo'"])))
    mj = Node(package='sim_overlay', executable='overlay_dump.py', name='overlay_dump', output='screen',
              parameters=[{'topics': ['/mp_viz/coverage_paint'], 'path': LC('mujoco_json')}],
              condition=IfCondition(PythonExpression(["'", LC('feed'), "' == 'mujoco'"])))
    return LaunchDescription(args + [paint, gz, mj])
