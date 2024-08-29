from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml, os

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # description_path = PathJoinSubstitution([
    #     FindPackageShare('interbotix_xsarm_descriptions'),
    #     'urdf',
    #     'wx250s.urdf.xacro',
    # ])

    moveit_config = (
        MoveItConfigsBuilder("wx250s", package_name='interbotix_xsarm_moveit')
        # .robot_description(file_path=description_path)
        # .trajectory_execution(file_path="config/controllers/wx250s_controllers.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .robot_description_semantic(file_path="config/srdf/wx250s.srdf.xacro")
        .joint_limits(file_path='config/joint_limits/wx250s_joint_limits.yaml')
        # .robot_description_kinematics(file_path="config/kinematics.yaml")
        # .to_moveit_configs()
    )

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }
    ompl_planning_pipeline_yaml_file = load_yaml(
        'interbotix_xsarm_moveit', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    # MTC Demo node
    pick_place_demo = Node(
        package="pick_and_place",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            kinematics_config,
            ompl_planning_pipeline_config,
        ],
        remappings=(
            ('/joint_states', f'/wx250s/joint_states'),
            ('/robot_description', f'/wx250s/robot_description'),
        ),
        # prefix=['gdbserver localhost:3000']
        # prefix=['gdb -ex run --args']
    )

    return LaunchDescription([pick_place_demo,])