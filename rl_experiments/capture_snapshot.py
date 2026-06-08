import os
import mujoco
import numpy as np
from PIL import Image
from dingo_mujoco_env import DingoMujocoEnv
from stable_baselines3 import PPO

def capture():
    # 1. Instantiate env
    env = DingoMujocoEnv()
    
    # 2. Load model
    model_path = "dingo_ppo_mujoco"
    if os.path.exists(f"{model_path}.zip"):
        model = PPO.load(model_path)
        print("Loaded trained model for snapshot.")
    else:
        model = None
        print("No trained model found. Capturing initial/random pose.")
        
    # Reset env
    obs, _ = env.reset()
    
    # Setup offscreen renderer
    renderer = mujoco.Renderer(env.model, height=480, width=640)
    
    # Simulate for 1.0 second (20 steps) to let it get into stance/motion
    for _ in range(20):
        if model is not None:
            action, _ = model.predict(obs, deterministic=True)
        else:
            action = env.action_space.sample()
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break
            
    # Setup tracking camera to fit the robot in the frame
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    base_link_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    cam.trackbodyid = base_link_id
    cam.distance = 0.8  # Zoom in close to fit frame
    cam.elevation = -20 # Look down slightly
    cam.azimuth = 135   # 3/4 side perspective
    
    # Update renderer and render frame
    renderer.update_scene(env.data, camera=cam)
    pixels = renderer.render()
    
    # Save image to the artifacts directory so the agent/user can view it
    artifact_dir = r"C:\Users\piyus\.gemini\antigravity-ide\brain\bf7daf70-c8c7-4973-9b0e-e329045fb07a"
    output_path = os.path.join(artifact_dir, "dingo_snapshot.png")
    
    img = Image.fromarray(pixels)
    img.save(output_path)
    print(f"Snapshot successfully saved to: {output_path}")

if __name__ == "__main__":
    capture()
