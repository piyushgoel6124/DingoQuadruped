import gymnasium as gym
import rospy
import numpy as np
from gymnasium import spaces
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class DingoGazeboEnv(gym.Env):
    def __init__(self):
        super(DingoGazeboEnv, self).__init__()
        
        rospy.init_node('dingo_gym_env', anonymous=True)
        
        # Joint names
        self.joint_names = [
            'FL_theta1', 'FL_theta2', 'FL_theta3',
            'FR_theta1', 'FR_theta2', 'FR_theta3',
            'RL_theta1', 'RL_theta2', 'RL_theta3',
            'RR_theta1', 'RR_theta2', 'RR_theta3'
        ]
        
        # Publishers
        self.pub_joints = {}
        for name in self.joint_names:
            self.pub_joints[name] = rospy.Publisher(f'/dingo_controller/{name}/command', Float64, queue_with_latter=1)
            
        # Subscribers
        rospy.Subscriber('/dingo_gazebo/joint_states', JointState, self._joint_state_callback)
        rospy.Subscriber('/imu', Imu, self._imu_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_callback)
        
        # Gazebo Services
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # State variables
        self.joint_positions = np.zeros(12)
        self.joint_velocities = np.zeros(12)
        self.imu_data = None
        self.base_pose = None
        self.base_twist = None
        
        # Gym Spaces
        # Action space: 12 joint angles (radians)
        self.action_space = spaces.Box(low=-1.5, high=1.5, shape=(12,), dtype=np.float32)
        
        # Observation space: 12 joint positions + 12 velocities + 4 orient (quat) + 3 angular vel + 3 linear accel
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(34,), dtype=np.float32)

    def _joint_state_callback(self, data):
        # Map names to indices
        for i, name in enumerate(data.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.joint_positions[idx] = data.position[i]
                self.joint_velocities[idx] = data.velocity[i]

    def _imu_callback(self, data):
        self.imu_data = data

    def _model_states_callback(self, data):
        if 'dingo_gazebo' in data.name:
            idx = data.name.index('dingo_gazebo')
            self.base_pose = data.pose[idx]
            self.base_twist = data.twist[idx]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # 1. Pause
        try:
            self.pause_proxy()
        except rospy.ServiceException:
            pass
            
        # 2. Reset Simulation
        try:
            self.reset_proxy()
        except rospy.ServiceException:
            pass
            
        # 3. Unpause
        try:
            self.unpause_proxy()
        except rospy.ServiceException:
            pass
            
        rospy.sleep(0.1) # Wait for simulation to settle
        
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        # 1. Apply actions
        for i, name in enumerate(self.joint_names):
            self.pub_joints[name].publish(Float64(action[i]))
            
        # 2. Simulate for some time
        rospy.sleep(0.05) # Control frequency ~20Hz
        
        obs = self._get_obs()
        
        # 3. Calculate Reward
        # Reward = forward velocity - energy penalty - stability penalty
        vx = self.base_twist.linear.x if self.base_twist else 0
        reward = vx * 10.0
        
        # Stability (roll/pitch should be close to 0)
        roll, pitch, yaw = self._get_euler()
        reward -= (abs(roll) + abs(pitch)) * 1.0
        
        # 4. Check if done
        terminated = False
        if abs(roll) > 1.0 or abs(pitch) > 1.0 or (self.base_pose and self.base_pose.position.z < 0.1):
            terminated = True
            reward -= 10.0 # Penalty for falling
            
        return obs, reward, terminated, False, {}

    def _get_obs(self):
        # Construct observation vector
        imu_quat = [0, 0, 0, 1]
        imu_ang_vel = [0, 0, 0]
        imu_lin_acc = [0, 0, 0]
        
        if self.imu_data:
            imu_quat = [self.imu_data.orientation.x, self.imu_data.orientation.y, 
                        self.imu_data.orientation.z, self.imu_data.orientation.w]
            imu_ang_vel = [self.imu_data.angular_velocity.x, self.imu_data.angular_velocity.y, self.imu_data.angular_velocity.z]
            imu_lin_acc = [self.imu_data.linear_acceleration.x, self.imu_data.linear_acceleration.y, self.imu_data.linear_acceleration.z]
            
        obs = np.concatenate([
            self.joint_positions,
            self.joint_velocities,
            imu_quat,
            imu_ang_vel,
            imu_lin_acc
        ]).astype(np.float32)
        return obs

    def _get_euler(self):
        if not self.imu_data:
            return 0, 0, 0
        import tf.transformations as tf
        q = [self.imu_data.orientation.x, self.imu_data.orientation.y, 
             self.imu_data.orientation.z, self.imu_data.orientation.w]
        return tf.euler_from_quaternion(q)
