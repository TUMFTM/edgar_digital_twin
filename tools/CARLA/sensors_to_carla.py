import argparse
import os
import random
import xml.etree.ElementTree as ET
import yaml

import carla


def sensor_setup_from_urdf(urdf_path, output_yaml=False):
    """Loads the sensor position and orientation from a URDF file.

    Args:
        urdf_path (path): path to the urdf file with the robot joint definitions.
        output_yaml (bool, optional): Optional output of sensor positions to yaml-file. Defaults to False.
    """
    with open(urdf_path, "r") as f:
        urdf_content = f.read()

    # Parsing the URDF content using XML parser
    urdf_parsed = ET.fromstring(urdf_content)

    # Extracting joint names and their corresponding xyz and rpy positions
    joints = {}
    for joint in urdf_parsed.findall(".//joint"):
        joint_name = joint.get("name")
        origin = joint.find("origin")

        # If xyz and rpy attributes are available in origin, extract and store them
        if origin is not None:
            xyz = (
                origin.get("xyz").split() if "xyz" in origin.attrib else [0.0, 0.0, 0.0]
            )
            rpy = (
                origin.get("rpy").split() if "rpy" in origin.attrib else [0.0, 0.0, 0.0]
            )
            joints[joint_name] = {
                "position_xyz": [float(p) for p in xyz],
                "orientation_rpy": [float(r) for r in rpy],
            }

    # For CARLA, the origin of the vehicle coordinate system is on the ground plane (z=0).
    # Therefore,we need to extract the z-coordinate of the rear axle center joint in
    # relation to the middle of the rear axle and adjust all z-coordinates accordingly.
    z_adjustment = joints["rear_axle_center_joint"].get("position_xyz")[2]

    for link_name, attributes in joints.items():
        adj_z = attributes.get("position_xyz")[2] + z_adjustment
        joints[link_name]["position_xyz"][2] = adj_z

    # Remove base link joint, rack joint
    remove_keys = [
        "rear_axle_center_ground_joint",
        "rack_rear_center_joint",
        "rear_axle_center_joint",
    ]
    for key in remove_keys:
        joints.pop(key, None)

    # Categorizing the entries
    sorted_joints = {"cameras": {}, "radars": {}, "lidars": {}, "others": {}}
    for joint_name, attributes in joints.items():
        if "camera" in joint_name or "framos" in joint_name:
            sorted_joints["cameras"][joint_name] = attributes
        elif "radar" in joint_name:
            sorted_joints["radars"][joint_name] = attributes
        elif "lidar" in joint_name:
            sorted_joints["lidars"][joint_name] = attributes
        else:
            sorted_joints["others"][joint_name] = attributes

    # Converting joints dictionary to YAML format
    if output_yaml:
        yaml_output = yaml.dump(sorted_joints, default_flow_style=False)
        with open("sensor_parameter_edgar.yaml", "w") as f:
            f.write(yaml_output)

    return sorted_joints


def render_sensor_as_box(sensor, mycolor):
    box = carla.BoundingBox(sensor.get_location(), carla.Vector3D(0.04, 0.02, 0.02))
    # print("this sensor location: ", sensor.get_location())
    world.debug.draw_box(
        box,
        sensor.get_transform().rotation,
        color=mycolor,
        thickness=0.06,
        life_time=0.5,
    )


def spawn_and_attach_sensor(vehicle, sensor_type, sensor_name, attributes):
    """
    Spawn a sensor and attach it to the vehicle.

    This function spawns a sensor and attach it to the vehicle in the given position
    and orientation. Further attributes can be passed in the dictionary and must be
    configured depending on the specifications of the sensor.
    To avoid performance issues, the listen() method is not called here, and must be
    manually set.

    Parameters:
    - vehicle: The vehicle actor.
    - sensor_type: Type of the sensor ("camera", "radar", "lidar").
    - attributes: Dictionary containing sensor attributes. In this implementation,
                  only the position_xyz and orientation_rpy attributes are used.
    """
    print(f"Adding sensor: {sensor_name}")
    if sensor_type.lower() == "cameras":
        bp = blueprint_library.find("sensor.camera.rgb")
        # Additional attributes can be set here; bp.set_attribute(key, attributes[key])
    elif sensor_type.lower() == "radars":
        bp = blueprint_library.find("sensor.other.radar")
        # Additional attributes can be set here; bp.set_attribute(key, attributes[key])
    elif sensor_type.lower() == "lidars":
        bp = blueprint_library.find("sensor.lidar.ray_cast")
        # Additional attributes can be set here; bp.set_attribute(key, attributes[key])
    else:
        print(f"Unknown sensor type: {sensor_type}")
        return None

    # Set the sensor's location and rotation according to dictionary attributes
    location = carla.Location(
        x=attributes["position_xyz"][0],
        y=attributes["position_xyz"][1] * (-1),  # UE Coordinates: y+ to the right
        z=attributes["position_xyz"][2],
    )
    rotation = carla.Rotation(
        roll=attributes["orientation_rpy"][0],
        pitch=attributes["orientation_rpy"][1]
        * (-1),  # UE Coordinates: y+ to the right
        yaw=attributes["orientation_rpy"][2],
    )
    transform = carla.Transform(location, rotation)

    sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
    world.tick()

    # Define distinct colors for each sensor type
    colors = {
        "cameras": carla.Color(255, 0, 0),  # Red for cameras
        "radars": carla.Color(0, 255, 0),  # Green for radars
        "lidars": carla.Color(0, 0, 255),  # Blue for lidars
    }
    color = colors.get(sensor_type.lower(), (255, 255, 255))  # default to white

    # # Draw a bounding box
    # render_sensor_as_box(sensor, color, world)
    sensor_color_and_sensor = []
    sensor_color_and_sensor.append(color)
    sensor_color_and_sensor.append(sensor)

    return sensor_color_and_sensor


def parse_arguments():
    file_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Program to simulate EDGAR Lidar System with different Sensor "
        "config in CARLA"
    )

    parser.add_argument(
        "--vehicle",
        default="edgar.t7",
        help="<make>.<model> of the vehicle to load from the library",
    )
    parser.add_argument(
        "--urdf",
        default=os.path.normpath(os.path.join(file_dir, "../../source/sensor_parameter/edgar.urdf")),
        help="Path to urdf file describing the sensor positions",
        dest="urdf_path",
    )

    return parser.parse_args()


if __name__ == "__main__":
    # Parse arguments
    args = parse_arguments()

    # Load sensor setup from urdf
    sensor_data = sensor_setup_from_urdf(args.urdf_path)

    # Connect to CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    # Get the world
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    vehicle_actors = []
    # Get our custom vehicle blueprint and spawn it at random spawn point
    vehicle_bp = blueprint_library.filter(f"vehicle.{args.vehicle}")[0]
    vehicle = world.spawn_actor(vehicle_bp, random.choice(spawn_points))
    vehicle_actors.append(vehicle)

    # Focus spectator on the vehicle
    spectator = world.get_spectator()
    spectator.set_transform(vehicle.get_transform())

    sensor_actors = []
    sensor_list_of_color_and_sensor = []
    # Spawn and attach sensors
    for sensor_type, sensors in sensor_data.items():
        for sensor_name, attributes in sensors.items():
            sensor_color_and_sensor = spawn_and_attach_sensor(
                vehicle, sensor_type, sensor_name, attributes
            )  # this is a list, [0] is the color, and [1] is the sensor
            if sensor_color_and_sensor:  # sensor is not None
                sensor_color = sensor_color_and_sensor[0]
                new_sensor = sensor_color_and_sensor[1]
                sensor_list_of_color_and_sensor.append(sensor_color_and_sensor)

    try:
        print("CARLA is running. Press Ctrl+C to stop.")
        while True:
            world.tick()
            for sens_c_and_s in sensor_list_of_color_and_sensor:
                if sens_c_and_s[1]:  # sensor is not None
                    render_sensor_as_box(sens_c_and_s[1], sens_c_and_s[0])

    except KeyboardInterrupt:
        print("Simulation interrupted")

    finally:
        print("Destroying all actors")
        actors_to_destroy = vehicle_actors + sensor_actors
        for actor in actors_to_destroy:
            actor.destroy()
        print("All actors destroyed")
