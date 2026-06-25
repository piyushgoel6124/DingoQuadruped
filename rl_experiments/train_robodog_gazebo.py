import os
import subprocess
import time
from stable_baselines3 import PPO
from robodog_gazebo_env import RoboDogGazeboEnv

def launch_gazebo():
    # Launch Gazebo headless in the background
    print("Launching Gazebo headless...")
    process = subprocess.Popen([
        "roslaunch", "robodog_gazebo", "rl_headless.launch"
    ])
    return process

def main():
    # 1. Start Gazebo
    gazebo_process = launch_gazebo()
    time.sleep(10) # Wait for Gazebo to start
    
    try:
        # 2. Create Environment
        env = RoboDogGazeboEnv()
        
        # 3. Define Model
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            device="cuda" # Use "cpu" if no GPU available
        )
        
        # 4. Train
        print("Starting training...")
        model.learn(total_timesteps=100000)
        
        # 5. Save
        model.save("robodog_ppo_gazebo")
        print("Training complete and model saved.")
        
    finally:
        # Cleanup
        gazebo_process.terminate()
        if os.name == 'nt':
            os.system("taskkill /F /IM gzserver.exe /T")
            os.system("taskkill /F /IM gzclient.exe /T")
            os.system("taskkill /F /IM roscore.exe /T")
            os.system("taskkill /F /IM rosmaster.exe /T")
        else:
            os.system("pkill -9 gzserver")
            os.system("pkill -9 gzclient")
            os.system("pkill -9 roscore")
            os.system("pkill -9 rosmaster")

if __name__ == "__main__":
    main()
