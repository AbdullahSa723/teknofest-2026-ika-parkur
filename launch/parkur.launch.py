"""
Launch the TEKNOFEST 2026 IKA parkur in Gazebo Sim 8 (Harmonic).

    ros2 launch ika_parkur_gazebo parkur.launch.py
    ros2 launch ika_parkur_gazebo parkur.launch.py paused:=true    # open frozen
    ros2 launch ika_parkur_gazebo parkur.launch.py gui:=false      # headless
    ros2 launch ika_parkur_gazebo parkur.launch.py verbosity:=4    # noisy

Bridges /clock so your ROS 2 nodes can run on simulation time.
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "ika_parkur_gazebo"


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_gazebo(context, *args, **kwargs):
    """Assemble gz_args in Python.

    Doing this with launch substitutions alone needs four conditioned includes
    for the gui x paused matrix, which is unreadable. One OpaqueFunction is
    clearer and the flags are resolved before Gazebo ever sees them.
    """
    pkg_share = get_package_share_directory(PKG)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = LaunchConfiguration("world").perform(context)
    gui = _truthy(LaunchConfiguration("gui").perform(context))
    paused = _truthy(LaunchConfiguration("paused").perform(context))
    verbosity = LaunchConfiguration("verbosity").perform(context)

    world_path = os.path.join(pkg_share, "worlds", world_file)
    if not os.path.isfile(world_path):
        raise RuntimeError(
            f"World not found: {world_path}\n"
            "Did you run generator/generate.py and rebuild the workspace?"
        )

    # Fingerprint the world that is actually about to load. Gazebo reads from
    # the INSTALL directory, not the source tree, so a forgotten `colcon build`
    # silently launches the previous version -- and an empty-looking parkur is
    # very hard to tell from a broken one. Printing the counts makes a stale
    # install obvious in one line.
    text = open(world_path, "r", encoding="utf-8").read()
    models = text.count("<model ")
    links = text.count("<link ")
    stamp = time.strftime("%Y-%m-%d %H:%M:%S",
                          time.localtime(os.path.getmtime(world_path)))
    print("\n" + "=" * 68)
    print(f" world  : {world_path}")
    print(f" built  : {stamp}")
    print(f" content: {models} models, {links} links")
    if models <= 1:
        print(" WARNING: only the ground is present. The install is stale --")
        print("          re-run  ./tools/sync_and_build.sh")
    print("=" * 68 + "\n")

    flags = [f"-v {verbosity}"]
    if not paused:
        flags.append("-r")          # -r starts the simulation immediately
    if not gui:
        flags.append("-s")          # -s is server-only, no rendering window
    gz_args = " ".join(flags + [world_path])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        )
    ]


def _slider_params(pkg_share):
    """Read the kayar engel's motion straight out of generator/config.yaml.

    config.yaml is the single source of truth for S6.8: the WORLD is generated
    from slider.speed / travel (the prismatic joint's limits come from
    `travel`, and generate.py warns if it is too short to clear the road). The
    driver declares its own defaults for those same values, so if the launch
    file does not pass them, the two halves are only in agreement by
    coincidence -- edit slider.travel and the blade's rails move while the node
    keeps commanding the old sweep, with nothing to flag the divergence.
    Passing them from here means one edit, one rebuild, both halves.

    Config lives beside the generator in the install tree
    (share/ika_parkur_gazebo/generator/config.yaml). If it is missing the node
    still starts on its own defaults, because a stale install should not turn
    into a launch failure at the competition -- but it says so loudly.
    """
    path = os.path.join(pkg_share, "generator", "config.yaml")
    try:
        sl = yaml.safe_load(open(path, "r", encoding="utf-8"))["slider"]
    except (OSError, KeyError, yaml.YAMLError) as exc:
        print(f" WARNING: cannot read slider config from {path} ({exc}).\n"
              "          kayar_engel_driver falls back to its own defaults, "
              "which may not match the world.")
        return {}
    return {
        "speed": float(sl["speed"]),
        "travel": float(sl["travel"]),
        "publish_rate": float(sl["publish_rate"]),
    }


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    # Gazebo must be able to find worlds/ and models/. The ament environment
    # hook covers an installed package; setting it here too keeps things working
    # under --symlink-install and when running straight from the source tree.
    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            p for p in (
                os.path.join(pkg_share, "worlds"),
                os.path.join(pkg_share, "models"),
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ) if p
        ),
    )

    # `[` is gz->ROS, `]` is ROS->gz. The slider command has to go ROS->gz or
    # the blade never receives it.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parkur_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/kayar_engel/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/world/ika_parkur/model/kayar_engel/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        output="screen",
    )

    slider_driver = Node(
        package=PKG,
        executable="kayar_engel_driver.py",
        name="kayar_engel_driver",
        condition=IfCondition(LaunchConfiguration("slider")),
        parameters=[{"use_sim_time": True, **_slider_params(pkg_share)}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="ika_parkur.sdf",
                              description="World file inside worlds/"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="false runs Gazebo server-only"),
        DeclareLaunchArgument("paused", default_value="false",
                              description="true opens the world frozen"),
        DeclareLaunchArgument("verbosity", default_value="3",
                              description="Gazebo verbosity, 0-4"),
        DeclareLaunchArgument("slider", default_value="true",
                              description="false leaves the kayar engel still"),
        resource_path,
        OpaqueFunction(function=_launch_gazebo),
        bridge,
        slider_driver,
    ])
