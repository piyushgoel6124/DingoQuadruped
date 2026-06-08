import time
from stable_baselines3 import PPO
from dingo_mujoco_env import DingoMujocoEnv

def enjoy():
    # 1. Instantiate the environment in human rendering mode
    env = DingoMujocoEnv(render_mode="human")
    
    # 2. Load the trained policy model
    # If the user has a pre-trained model or just finished training:
    try:
        model = PPO.load("dingo_ppo_mujoco", env=env)
        print("Loaded trained model successfully.")
    except Exception as e:
        print(f"Trained model not found ({e}). Running random policy instead...")
        model = None
        
    obs, _ = env.reset()
    print("Starting simulation visualization. Press Ctrl+C in terminal or close window to exit.")
    
    try:
        while True:
            start_time = time.time()
            
            if model is not None:
                action, _ = model.predict(obs, deterministic=True)
            else:
                action = env.action_space.sample()
                
            obs, reward, terminated, truncated, _ = env.step(action)
            
            if terminated or truncated:
                obs, _ = env.reset()
                
            # Sleep to match control frequency (~20Hz / 50ms)
            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.05 - elapsed))
            
    except KeyboardInterrupt:
        print("Visualizer stopped.")
    finally:
        env.close()

if __name__ == "__main__":
    enjoy()
