import mujoco

def main():
    urdf_path = "dingo_clean.urdf"
    temp_urdf_path = "temp_dingo.urdf"
    
    with open(urdf_path, "r") as f:
        content = f.read()
        
    # Inject mujoco compiler tag for meshes
    mesh_dir = "d:/vscode/DingoQuadruped/dingo_ws/src/dingo_description/meshes"
    mujoco_config = f'<mujoco><compiler meshdir="{mesh_dir}" discardvisual="false" fusestatic="false" autolimits="true"/></mujoco>'
    
    if "</robot>" in content:
        content = content.replace("</robot>", f"{mujoco_config}\n</robot>")
        
    with open(temp_urdf_path, "w") as f:
        f.write(content)
        
    # Load and save as MJCF
    model = mujoco.MjModel.from_xml_path(temp_urdf_path)
    mujoco.mj_saveLastXML("dingo_robot_mjcf.xml", model)
    print("Successfully converted URDF to MJCF: dingo_robot_mjcf.xml")

if __name__ == "__main__":
    main()
