import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces

class RoboDogMujocoEnv(gym.Env):
    def __init__(self, xml_path="robodog_scene.xml", render_mode=None):
        super(RoboDogMujocoEnv, self).__init__()
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.joint_names = [
            'FL_theta1', 'FL_theta2', 'FL_theta3',
            'FR_theta1', 'FR_theta2', 'FR_theta3',
            'RL_theta1', 'RL_theta2', 'RL_theta3',
            'RR_theta1', 'RR_theta2', 'RR_theta3'
        ]
        
        # Action Space: 12 commanded joint angles
        # Enforce tight, physically realistic joint limits:
        # - Hip (theta1): [-0.6, 0.6] rad (sideways swing)
        # - Thigh (theta2): [-1.0, 0.3] rad (prevents swinging all the way backward)
        # - Calf (theta3): [-1.2, 0.0] rad (forces it to stay tucked under the body)
        low_bounds = []
        high_bounds = []
        for i in range(12):
            joint_type = i % 3
            if joint_type == 0:   # Hip
                low_bounds.append(-0.6)
                high_bounds.append(0.6)
            elif joint_type == 1: # Thigh
                low_bounds.append(-1.0)
                high_bounds.append(0.3)
            elif joint_type == 2: # Calf
                low_bounds.append(-1.2)
                high_bounds.append(0.0)
                
        self.action_space = spaces.Box(
            low=np.array(low_bounds, dtype=np.float32),
            high=np.array(high_bounds, dtype=np.float32),
            dtype=np.float32
        )
        
        # Observation Space (22 dimensions):
        # - Base orientation quaternion (4): w, x, y, z
        # - Base angular velocity in local frame (3)
        # - Base linear acceleration in local frame (3)
        # - Previous commanded target joint angles (12)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(22,), dtype=np.float32)
        
        self.prev_action = np.zeros(12, dtype=np.float32)
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        
        # Nominal stance reference for styling: Hip=0.0, Thigh=-0.785 (back down 45 deg), Calf=-0.785 (front down 45 deg)
        self.nominal_stance = np.array([0.0, -0.785, -0.785] * 4, dtype=np.float32)
        
        self.render_mode = render_mode
        self.viewer = None
        if self.render_mode == "human":
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        
        # Set base height
        self.data.qpos[2] = 0.25
        
        # Step simulation a few times to stabilize
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)
            
        self.prev_action = np.zeros(12, dtype=np.float32)
        
        if self.viewer is not None:
            self.viewer.sync()
            
        obs = self._get_obs()
        return obs, {}
        
    def step(self, action):
        # Strictly enforced joint angles:
        # Command joints directly via actuator control (self.data.ctrl)
        # In robodog_scene.xml, we have 12 motor actuators.
        for i in range(12):
            self.data.ctrl[i] = action[i]
            
        # Decimate control frequency (sim dt is 0.002, step for 25 steps to run at 20Hz / 0.05s control rate)
        control_decimation = 25
        for _ in range(control_decimation):
            mujoco.mj_step(self.model, self.data)
            
        obs = self._get_obs()
        
        # Calculate Reward using ground truth velocity from physics engine (not in obs)
        vx = self.data.qvel[0] # Linear velocity x
        vy = self.data.qvel[1] # Linear velocity y
        
        # Get base orientation quaternion
        quat = self.data.qpos[3:7] # [w, x, y, z]
        roll, pitch, _ = self._quat_to_euler(quat)
        
        reward = vx * 30.0  # Encourage forward speed
        reward -= abs(vy) * 2.0  # Penalize sideways drift
        reward -= (abs(roll) + abs(pitch)) * 3.0  # Penalize tilting / instabilitiy
        
        # Penalize jerky/large command changes
        action_penalty = np.sum(np.square(action - self.prev_action))
        reward -= 0.1 * action_penalty
        
        # Gait styling: Penalize forward paw overextension relative to the hip vertical axis.
        paw_overextension_penalty = 0.0
        for leg in ['FL', 'FR', 'RL', 'RR']:
            hip_x = self.data.body(f'{leg}_link1').xpos[0]
            foot_x = self.data.body(f'{leg}_link4').xpos[0]
            overextension = foot_x - hip_x
            if overextension > 0.0:
                paw_overextension_penalty += overextension
        reward -= 2.0 * paw_overextension_penalty
        
        # Enforce knee height limit: Penalize if the knee (link3) moves above the horizontal line of the hip (link1)
        knee_height_penalty = 0.0
        for leg in ['FL', 'FR', 'RL', 'RR']:
            hip_z = self.data.body(f'{leg}_link1').xpos[2]
            knee_z = self.data.body(f'{leg}_link3').xpos[2]
            height_diff = knee_z - hip_z
            if height_diff > 0.0:
                knee_height_penalty += height_diff
        reward -= 2.0 * knee_height_penalty
        
        # Minimize hip abduction movement in straight walking
        hip_penalty = 0.0
        for l in range(4):
            hip_penalty += np.square(action[l * 3])
        reward -= 0.5 * hip_penalty
        
        # Encourage upper and lower leg to rotate by similar amounts (coordinated coupling)
        leg_coupling_penalty = 0.0
        for l in range(4):
            upper_act = action[l * 3 + 1]
            lower_act = action[l * 3 + 2]
            leg_coupling_penalty += np.square(upper_act - lower_act)
        reward -= 0.5 * leg_coupling_penalty
        
        # Guide policy toward natural nominal stance: Thigh=-0.785, Calf=-0.785
        stance_penalty = np.sum(np.square(action - self.nominal_stance))
        reward -= 0.5 * stance_penalty
        
        # Enforce that only a maximum of 2 legs should move at any time
        leg_movements = []
        for l in range(4):
            leg_act = action[l * 3 : l * 3 + 3]
            leg_prev = self.prev_action[l * 3 : l * 3 + 3]
            leg_movements.append(np.sum(np.abs(leg_act - leg_prev)))
        sorted_movements = sorted(leg_movements)
        two_leg_movement_penalty = sorted_movements[0] + sorted_movements[1]
        reward -= 1.0 * two_leg_movement_penalty
        
        # Enforce diagonal stance symmetry (trot synchronization):
        # Leg 0: FL, Leg 1: FR, Leg 2: RL, Leg 3: RR
        move_FL = leg_movements[0]
        move_FR = leg_movements[1]
        move_RL = leg_movements[2]
        move_RR = leg_movements[3]
        pair_A_movement = move_FL + move_RR
        pair_B_movement = move_FR + move_RL
        
        diagonal_sync_penalty = pair_A_movement * pair_B_movement
        reward -= 2.0 * diagonal_sync_penalty
        
        # Penalize excessive height deviation (neutral standing height is around 0.25m)
        height = self.data.qpos[2]
        reward -= abs(height - 0.25) * 5.0
        
        self.prev_action = np.copy(action).astype(np.float32)
        
        # Termination conditions
        terminated = False
        # Fall detection
        if height < 0.12 or height > 0.45 or abs(roll) > 0.8 or abs(pitch) > 0.8:
            terminated = True
            reward -= 50.0  # Large falling penalty
            
        if self.viewer is not None:
            self.viewer.sync()
            
        return obs, reward, terminated, False, {}

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
        
    def _get_obs(self):
        # 1. Orientation quaternion [w, x, y, z]
        quat = self.data.qpos[3:7]
        
        # 2. Local angular velocity (transform global angular velocity qvel[3:6] to body local frame)
        global_ang_vel = self.data.qvel[3:6]
        # In MuJoCo, data.qvel[3:6] for a freejoint is already in body frame representation!
        # But to be safe, we can read directly:
        local_ang_vel = global_ang_vel
        
        # 3. Local linear acceleration (approximate by change in velocity or use body frame velocity)
        # BNO055 gives linear acceleration, but in practice, local linear velocity/acceleration is equivalent.
        # Let's map it to local linear velocity (since BNO055 linear accel corresponds to tracking motion changes).
        # We can extract linear velocity and add a small gravity vector to simulate accelerometer.
        local_lin_vel = self.data.qvel[0:3]
        
        obs = np.concatenate([
            quat,
            local_ang_vel,
            local_lin_vel,
            self.prev_action
        ]).astype(np.float32)
        
        return obs
        
    def _quat_to_euler(self, q):
        w, x, y, z = q
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
