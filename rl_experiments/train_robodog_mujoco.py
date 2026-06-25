import os
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from robodog_mujoco_env import RoboDogMujocoEnv

def make_env(rank, seed=0):
    """
    Utility function to multiprocess training environments.
    """
    def _init():
        # Headless mode for training efficiency
        env = RoboDogMujocoEnv(xml_path="robodog_scene.xml", render_mode=None)
        env.reset(seed=seed + rank)
        return env
    return _init

def main():
    # 1. Check GPU / CUDA availability (for the RTX 3050Ti)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")
    
    model_path = "robodog_ppo_mujoco"
    checkpoint_path = f"{model_path}.zip"
    
    # 2. Instantiate Vectorized Environments (8 parallel CPU workers)
    num_envs = 8
    print(f"Spawning {num_envs} parallel simulation environments on CPU cores...")
    env = SubprocVecEnv([make_env(i) for i in range(num_envs)])
    
    # 3. Define Model (Load to resume if file exists, otherwise initialize new)
    if os.path.exists(checkpoint_path):
        print(f"Found existing checkpoint '{checkpoint_path}'. Resuming training...")
        model = PPO.load(model_path, env=env, device=device)
    else:
        print("No existing checkpoint found. Starting training from scratch...")
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            device=device
        )
    
    # 4. Train
    print("Starting parallel PPO training on MuJoCo environment...")
    try:
        # Training for 500,000 timesteps to learn a robust gait.
        # reset_num_timesteps=False preserves training step counters on resume.
        model.learn(total_timesteps=500000, reset_num_timesteps=False)
        model.save(model_path)
        print(f"Training complete. Model saved to {model_path}.zip")
    except KeyboardInterrupt:
        print("\nTraining interrupted by user. Saving progress to checkpoint...")
        model.save(model_path)
        print(f"Progress saved successfully to {model_path}.zip")
    finally:
        env.close()

if __name__ == "__main__":
    main()
