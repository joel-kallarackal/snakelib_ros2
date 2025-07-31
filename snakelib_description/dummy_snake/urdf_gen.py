import xml.dom.minidom as minidom
from xml.etree.ElementTree import Element, SubElement, tostring

def pretty_xml(xml_str):
    return minidom.parseString(xml_str).toprettyxml(indent="  ")

robot = Element('robot', name="snake_robot")

# Define material
material = SubElement(robot, 'material', name="gray")
SubElement(material, 'color', rgba="0.5 0.5 0.5 1")

# Function to create a link as a cylinder aligned along x-axis
def create_link(name):
    link = SubElement(robot, 'link', name=name)

    visual = SubElement(link, 'visual')
    SubElement(visual, 'origin', xyz="0 0 0", rpy="0 1.5708 0")
    geometry = SubElement(visual, 'geometry')
    SubElement(geometry, 'cylinder', length="0.1", radius="0.02")
    SubElement(visual, 'material', name="gray")

    collision = SubElement(link, 'collision')
    SubElement(collision, 'origin', xyz="0 0 0", rpy="0 1.5708 0")
    geometry = SubElement(collision, 'geometry')
    SubElement(geometry, 'cylinder', length="0.1", radius="0.02")

    inertial = SubElement(link, 'inertial')
    SubElement(inertial, 'mass', value="0.1")
    SubElement(inertial, 'origin', xyz="0 0 0", rpy="0 0 0")
    SubElement(inertial, 'inertia',
               ixx="1e-4", iyy="1e-4", izz="1e-4",
               ixy="0", ixz="0", iyz="0")

# Function to create a fixed IMU with proper rotation
def create_imu(i):
    imu_link = SubElement(robot, 'link', name=f"imu_{i}")
    imu_joint = SubElement(robot, 'joint', name=f"imu_joint_{i}", type="fixed")
    SubElement(imu_joint, 'parent', link=f"link_{i}")
    SubElement(imu_joint, 'child', link=f"imu_{i}")

    rpy_map = {
        0: "0 0 0",         # y
        1: "0 0 1.5708",    # z
        2: "3.1416 0 0",    # -y
        3: "0 0 -1.5708",   # -z
    }
    rpy = rpy_map[i % 4]
    SubElement(imu_joint, 'origin', xyz="0 0 0", rpy=rpy)

# Define joint axes: y, z, -y, -z
axes = ["0 1 0", "0 0 1", "0 -1 0", "0 0 -1"]

# Create base link and IMU
create_link("link_0")
create_imu(0)

# Create 15 more modules
for i in range(1, 4):
    joint = SubElement(robot, 'joint', name=f"joint_{i}", type="revolute")
    SubElement(joint, 'parent', link=f"link_{i-1}")
    SubElement(joint, 'child', link=f"link_{i}")
    SubElement(joint, 'origin', xyz="0.1 0 0", rpy="0 0 0")  # offset along x
    SubElement(joint, 'axis', xyz=axes[i % 4])
    SubElement(joint, 'limit', effort="1.0", velocity="1.0", lower="-1.5708", upper="1.5708")

    create_link(f"link_{i}")
    create_imu(i)

# Save to file
urdf_string = pretty_xml(tostring(robot, encoding='unicode'))
with open("snake_robot.urdf", "w") as f:
    f.write(urdf_string)

print("URDF written to snake_robot.urdf")
