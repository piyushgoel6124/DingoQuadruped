import xml.etree.ElementTree as ET

def main():
    # Load robodog_robot_mjcf.xml
    tree = ET.parse("robodog_robot_mjcf.xml")
    root = tree.getroot()
    
    # 1. Update options and structure
    # Add option element with smaller timestep for numerical integration stability (0.002s)
    opt = ET.Element("option", timestep="0.002", iterations="50", solver="PGS", gravity="0 0 -9.81")
    root.insert(1, opt)
    
    # Get worldbody
    worldbody = root.find("worldbody")
    
    # Create lights and floor geoms
    light = ET.Element("light", directional="true", diffuse=".8 .8 .8", specular=".2 .2 .2", pos="0 0 5", dir="0 0 -1")
    floor = ET.Element("geom", name="floor", type="plane", size="100 100 .1", rgba=".8 .9 .8 1", friction="1 0.005 0.0001")
    
    # Insert light and floor at beginning of worldbody
    worldbody.insert(0, light)
    worldbody.insert(1, floor)
    
    # Find base_link body
    base_link = worldbody.find("body[@name='base_link']")
    if base_link is not None:
        # Set base_link initial height to 0.3
        base_link.set("pos", "0 0 0.3")
        # Add freejoint for free floating movement
        freejoint = ET.Element("freejoint", name="root")
        base_link.insert(0, freejoint)
        
    # 1.5 Add armature and damping to all revolute joints to prevent numerical instability/explosions
    for joint in root.iter("joint"):
        joint.set("damping", "1.0")
        joint.set("armature", "0.05")
        joint.set("frictionloss", "0.1")
        
    # 2. Add Actuators
    actuator = ET.Element("actuator")
    joints = [
        ('FL_theta1', '-0.785398 0.785398'),
        ('FL_theta2', '-3 3'),
        ('FL_theta3', '-3 3'),
        ('FR_theta1', '-0.785398 0.785398'),
        ('FR_theta2', '-3 3'),
        ('FR_theta3', '-3 3'),
        ('RL_theta1', '-0.785398 0.785398'),
        ('RL_theta2', '-3 3'),
        ('RL_theta3', '-3 3'),
        ('RR_theta1', '-0.785398 0.785398'),
        ('RR_theta2', '-3 3'),
        ('RR_theta3', '-3 3')
    ]
    for j_name, ctrl_range in joints:
        motor = ET.Element("position", joint=j_name, kp="20", ctrlrange=ctrl_range, ctrllimited="true")
        actuator.append(motor)
        
    root.append(actuator)
    
    # Save output to robodog_scene.xml
    tree.write("robodog_scene.xml", encoding="utf-8", xml_declaration=True)
    print("Successfully built robodog_scene.xml with actuators and floor!")

if __name__ == "__main__":
    main()
