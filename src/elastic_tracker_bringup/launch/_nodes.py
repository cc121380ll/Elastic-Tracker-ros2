from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate(phase="all"):
    share = Path(get_package_share_directory("elastic_tracker_bringup"))
    config = yaml.safe_load((share / "config/baseline.yaml").read_text())
    def node(pkg, exe, ns, name, remaps=()):
        params = config.get(f"/{ns}/{name}" if ns else f"/{name}", {}).get("ros__parameters", {})
        return Node(package=pkg, executable=exe, namespace=ns, name=name,
                    parameters=[params], remappings=[("~/"+a,b) for a,b in remaps], output="screen")
    actions = [DeclareLaunchArgument("rviz", default_value="false")]
    if phase in ("all", "target"):
        actions.append(Node(package="mockamap", executable="mockamap_node", name="mockamap_node", parameters=[config["/mockamap_node"]["ros__parameters"]], remappings=[("mock_map", "/global_map")], output="screen"))
    for ns in (["target", "drone0"] if phase=="all" else ["target"] if phase=="target" else ["drone0"]):
        actions.extend([
            node("so3_quadrotor", "so3_quadrotor_node", ns, "so3_quadrotor", [(x,x) for x in ["odom","imu","so3cmd","vis"]]),
            node("so3_controller", "so3_controller_node", ns, "so3_controller", [(x,x) for x in ["odom","imu","so3cmd","position_cmd"]]),
            node("mapping", "mapping_node", ns, "mapping", [("global_map","/global_map"),("odom","odom"),("depth","depth"),("gridmap_inflate","gridmap_inflate")]),
            node("odom_visualization", "odom_visualization", ns, "odom_visualization", [("odom","odom")]),
            node("planning", "planning_node", ns, "planning", [("odom","odom"),("gridmap_inflate","gridmap_inflate"),("heartbeat","heartbeat"),("trajectory","trajectory"),("replanState","replanState"),("target","/target_ekf_odom"),("triger","/move_base_simple/goal" if ns=="target" else "/triger"),("land_triger","/land_triger")]),
            node("planning", "traj_server", ns, "traj_server", [(x,x) for x in ["position_cmd","trajectory","heartbeat"]]),
        ])
        if ns=="drone0":
            actions.extend([
                node("local_sensing_node", "pcl_render_node", ns, "pcl_render_node", [("global_map","/global_map"),("odometry","odom"),("depth","depth")]),
                node("target_ekf", "target_ekf_sim_node", ns, "target_ekf_sim_node", [("yolo","/target/odom"),("odom","odom"),("target_odom","/target_ekf_odom")]),
                node("mapping", "mapping_vis_node", ns, "mapping_vis", [("gridmap","gridmap"),("gridmap_inflate","gridmap_inflate")]),
            ])
    actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", str(share/"config/rviz_sim.rviz")], condition=IfCondition(LaunchConfiguration("rviz")), output="screen"))
    return LaunchDescription(actions)
