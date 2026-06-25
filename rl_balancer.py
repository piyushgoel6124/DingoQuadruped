import os
import json
import numpy as np

class RLBalancer:
    def __init__(self, state_dim=4, action_dim=2, lr_actor=0.01, lr_critic=0.02, gamma=0.95, sigma=0.005):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.lr_actor = lr_actor
        self.lr_critic = lr_critic
        self.gamma = gamma
        self.sigma = sigma
        
        # Linear function approximation weights
        # state is [pitch, roll, pitch_rate, roll_rate, 1.0] (bias included)
        self.w_critic = np.zeros(self.state_dim + 1)
        self.W_actor = np.zeros((self.action_dim, self.state_dim + 1))
        
        self.weights_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rl_balancer_weights.json")
        self.load_weights()
        
        # Episode/training statistics
        self.total_steps = 0
        self.accumulated_reward = 0.0
        self.last_reward = 0.0
        self.last_state = None
        self.last_action = None

    def get_state_vector(self, state):
        # state expected: [pitch, roll, pitch_rate, roll_rate]
        return np.array(list(state) + [1.0], dtype=np.float64)

    def select_action(self, state, explore=True):
        s = self.get_state_vector(state)
        mean_action = np.dot(self.W_actor, s)
        
        if explore:
            noise = np.random.normal(0, self.sigma, size=self.action_dim)
            action = mean_action + noise
        else:
            action = mean_action
            
        # Clamp actions to safe Z height offsets (e.g., -6cm to +6cm)
        action = np.clip(action, -0.06, 0.06)
        return action, mean_action

    def update(self, state, action, reward, next_state, mean_action):
        s = self.get_state_vector(state)
        s_next = self.get_state_vector(next_state)
        
        # Critic prediction and TD target
        v = np.dot(self.w_critic, s)
        v_next = np.dot(self.w_critic, s_next)
        
        td_target = reward + self.gamma * v_next
        td_error = td_target - v
        
        # Update Critic
        self.w_critic += self.lr_critic * td_error * s
        
        # Update Actor (policy gradient update)
        # Using linear approximation, gradient of log pi is (action - mean_action) * s^T
        # We also limit/clip updates to prevent instability
        update_val = td_error * (action - mean_action)
        update_val = np.clip(update_val, -0.1, 0.1) # gradient clipping for safety
        self.W_actor += self.lr_actor * np.outer(update_val, s)
        
        # Update training metrics
        self.total_steps += 1
        self.accumulated_reward += reward
        self.last_reward = reward
        
        # Auto-save weights every 100 steps
        if self.total_steps % 100 == 0:
            self.save_weights()

    def save_weights(self):
        try:
            data = {
                "w_critic": self.w_critic.tolist(),
                "W_actor": self.W_actor.tolist(),
                "total_steps": self.total_steps
            }
            with open(self.weights_path, "w") as f:
                json.dump(data, f)
        except Exception as e:
            print(f"Error saving RL balancer weights: {e}")

    def load_weights(self):
        if os.path.exists(self.weights_path):
            try:
                with open(self.weights_path, "r") as f:
                    data = json.load(f)
                if "w_critic" in data:
                    self.w_critic = np.array(data["w_critic"], dtype=np.float64)
                if "W_actor" in data:
                    self.W_actor = np.array(data["W_actor"], dtype=np.float64)
                if "total_steps" in data:
                    self.total_steps = int(data["total_steps"])
                print(f"Loaded RL Balancer weights successfully from: {self.weights_path} (steps: {self.total_steps})")
            except Exception as e:
                print(f"Failed to load RL balancer weights: {e}")
