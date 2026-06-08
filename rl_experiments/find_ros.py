import os
import sys

def find_ros():
    paths_to_check = [
        "C:\\opt\\ros\\noetic\\x64",
        "C:\\Program Files\\ros",
        os.path.expanduser("~\\.local\\lib\\python3.10\\site-packages"),
        "D:\\ros_on_windows"
    ]
    
    for path in paths_to_check:
        if os.path.exists(path):
            print(f"Found potential ROS path: {path}")
            return path
    return None

if __name__ == "__main__":
    ros_path = find_ros()
    if ros_path:
        print(f"ROS path found at {ros_path}")
    else:
        print("ROS not found in common locations.")
        print("Checking environment variables...")
        for key, value in os.environ.items():
            if "ros" in key.lower() or "gazebo" in key.lower():
                print(f"{key}: {value}")
