# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml

# This launch file will take a raw pointcloud coming from driver node and do preprocessing per lidar before concatenation
# This includes cropping of ego vehicle, correction of distoration coming from ego motion, and outlier removal

def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", LaunchConfiguration("input_topic")),
                ("output", "self_cropped/pointcloud"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    #nodes.append(
    #    ComposableNode(
    #        package="pointcloud_preprocessor",
    #        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
    #        name="crop_box_filter_mirror",
    #        remappings=[
    #            ("input", "self_cropped/pointcloud"),
    #            ("output", "mirror_cropped/pointcloud"),
    #        ],
    #        parameters=[cropbox_parameters],
    #        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #    )
    #)

    # Uncomment this once driver is updated to insert timestamps for each points
    # Also it requires imu info and vehicle information for distortion correction

    # nodes.append(
    #     ComposableNode(
    #         package="autoware_pointcloud_preprocessor",
    #         plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
    #         name="distortion_corrector_node",
    #         remappings=[
    #             ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
    #             ("~/input/imu", "/sensing/imu/imu_data"),
    #             ("~/input/pointcloud", "self_cropped/pointcloud"),
    #             ("~/output/pointcloud", "rectified/pointcloud_ex"),
    #         ],
    #         parameters=[{
    #             "base_frame": LaunchConfiguration("base_frame"),
    #             "use_imu": False,
    #             "use_3d_distortion_correction": False,
    #             "update_azimuth_and_distance": False,
    #             "has_static_tf_only": True,
    #         }],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )


    # This also requires point cloud fields mentioned in autoware design documentaiton
    # https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/#point-cloud-fields

    # nodes.append(
    #     ComposableNode(
    #         package="pointcloud_preprocessor",
    #         plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
    #         name="ring_outlier_filter",
    #         remappings=[
    #             ("input", "rectified/pointcloud_ex"),
    #             ("output", "outlier_filtered/pointcloud"),
    #         ],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    component_loader = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return [container, component_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )
    add_launch_arg("input_topic", description="Topic coming from Ouster driver")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("container_name", "ouster_composable_node_container", "container name")
    add_launch_arg("frame_id", "ouster", "ouster lidar frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS2 component container communication")
    add_launch_arg("use_pointcloud_container", "false")
    add_launch_arg("container_name", "velodyne_node_container")
    add_launch_arg(
        "distortion_correction_node_param_path",
        "edgar_common_sensor_launch/config/distortion_corrector_node.param.yaml",
        description="path to parameter file of distortion correction node",
    )
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )