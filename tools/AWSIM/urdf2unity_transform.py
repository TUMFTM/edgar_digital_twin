import xml.etree.ElementTree as ET

rad2degree = 180.0/ 3.14159265359

def urdf2unity_coordinate_transform(x, y, z):
    return (-y, z, x)

def urdf2unity_orientation_transform(roll, pitch, yaw):
    return (pitch * rad2degree, -yaw * rad2degree, -roll * rad2degree)

def urdf2unity_file_transform(path_to_urdf, path_to_save):
    urdf = ET.parse(path_to_urdf)

    for element in urdf.iter():
        if(element.tag == "origin"):
            if("xyz" in element.attrib):
                xyz = [float(i) for i in element.attrib["xyz"].split(" ")]
                xyz_transform = urdf2unity_coordinate_transform(xyz[0], xyz[1], xyz[2])
                element.attrib["xyz"] = str(xyz_transform[0]) + " " + str(xyz_transform[1]) + " " + str(xyz_transform[2])            
            if("rpy" in element.attrib):
                rpy = [float(i) for i in element.attrib["rpy"].split(" ")]
                rpy_transform = urdf2unity_orientation_transform(rpy[0], rpy[1], rpy[2])
                element.attrib["rpy"] = str(rpy_transform[0]) + " " + str(rpy_transform[1]) + " " + str(rpy_transform[2])
    urdf.write(path_to_save, encoding='utf8')



if __name__ == "__main__":
    import os
    
    current_dir = os.path.dirname(os.path.realpath(__file__))
    path_to_unity_urdf = os.path.join(current_dir, "unity_transform.urdf")
    path_to_urdf = os.path.join(current_dir, "../../source/sensor_parameters/edgar.urdf")
    #path_to_urdf = os.path.join(current_dir, "edgar.urdf")

    print("parsing coordinate system in urdf file" , path_to_urdf, "to unity coordinate system.")
    urdf2unity_file_transform(path_to_urdf, path_to_unity_urdf)
    print("you will find the new file with unity coordinates at", path_to_unity_urdf)