import gymnasium as gym
import numpy as np
import mujoco
import mujoco.viewer
import os

class UR5ReachEnv(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        
        
        # 1. SETUP PATHS
        current_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = "/home/rishabh/ros2_mujoco_ws/src/mujoco_ur5_bridge/mujoco_ur5_bridge/mujoco_menagerie/universal_robots_ur5e/lesson10_ur5_rl.xml"        
        # 2. LOAD MUJOCO
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 3. DEFINE ACTION & OBSERVATION SPACES
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        
        # Obs: 6 qpos + 6 qvel + 3 target_pos = 15
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(15,), dtype=np.float32)
        
        # Renderer setup
        self.render_mode = render_mode
        self.viewer = None
        if self.render_mode == "human":
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.max_steps = 500

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.current_step = 0

        # A. Reset Robot
        reset_pose = np.array([-1.57, -1.57, 1.57, -1.57, -1.57, 0], dtype=np.float32)
        self.data.qpos[:] = reset_pose
        self.data.qvel[:] = 0

        # B. Move Target
        x = np.random.uniform(0.3, 0.7)
        y = np.random.uniform(-0.3, 0.3)
        z = np.random.uniform(0.2, 0.6)
        
        target_id = self.model.body("target").mocapid[0]
        self.data.mocap_pos[target_id] = [x, y, z]
        
        mujoco.mj_step(self.model, self.data)
        
        if self.render_mode == "human" and self.viewer:
            self.viewer.sync()
            
        return self._get_obs(), {}

    def step(self, action):
        # 1. APPLY ACTION (Keep as is)
        self.data.ctrl[:] = self.data.ctrl[:] + (action * 0.1)
        self.data.ctrl[:] = np.clip(self.data.ctrl[:], -3.14, 3.14)
        
        # 2. RUN PHYSICS (Keep as is)
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
            
        # 3. INCREMENT STEP COUNTER
        self.current_step += 1
            
        # 4. CALCULATE REWARD 
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        hand_pos = self.data.site_xpos[site_id]
        target_id = self.model.body("target").mocapid[0]
        target_pos = self.data.mocap_pos[target_id]
        distance = np.linalg.norm(hand_pos - target_pos)
        
        reward = -distance
        
        # 5. CHECK TERMINATION (Win) vs TRUNCATION (Timeout)
        terminated = False
        truncated = False
        
        # Condition A: WIN (Touched the ball)
        if distance < 0.05:
            print("TARGET HIT! +100 Points")
            reward += 100.0
            terminated = True
            
        # Condition B: TIMEOUT (Took too long)
        if self.current_step >= self.max_steps:
            # We didn't hit it, but time is up. Reset anyway.
            truncated = True

        if self.render_mode == "human" and self.viewer:
            self.viewer.sync()
            
        return self._get_obs(), reward, terminated, truncated, {}

    def _get_obs(self):
        # 1. Get Joint Data
        qpos = self.data.qpos.flat[:6]
        qvel = self.data.qvel.flat[:6]
        
        # 2. Get Target Position
        target_id = self.model.body("target").mocapid[0]
        target_pos = self.data.mocap_pos[target_id]
        
        # 3. Get Hand Position
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        hand_pos = self.data.site_xpos[site_id]
        
        # 4. Calculate the "Difference" (The Vector)
        # This tells the robot exactly which direction to move
        dist_vec = target_pos - hand_pos
        
        # 5. Return everything
        # We replace 'target_pos' with 'dist_vec'
        return np.concatenate([qpos, qvel, dist_vec]).astype(np.float32)

    def close(self):
        if self.viewer:
            self.viewer.close()