from gymnasium.envs.classic_control.mountain_car import MountainCarEnv
import numpy as np

# allows us to use the gymnasium mountain car environment with variable mass
class CustomMountainCarEnv(MountainCarEnv):
    def __init__(self, render_mode=None, goal_velocity=0, mass_range=(0.5, 2.0)):
        super().__init__(render_mode=render_mode, goal_velocity=goal_velocity)
        self.mass_range = mass_range
        self.current_mass = 1.0  # default mass
        
    def reset(self, *, seed=None, options=None):
        super(MountainCarEnv, self).reset(seed=seed)
        
        self.current_mass = self.np_random.uniform(self.mass_range[0], self.mass_range[1])
        
        pos_low = self.np_random.uniform(-1.1, -0.8)
        pos_high = self.np_random.uniform(-0.3, 0.2)
        start_pos = self.np_random.uniform(pos_low, pos_high)
        
        start_vel = self.np_random.uniform(-0.05, 0.05)
        
        # Ensure bounds are respected
        start_pos = np.clip(start_pos, self.min_position, self.max_position)
        start_vel = np.clip(start_vel, -self.max_speed, self.max_speed)
        
        self.state = np.array([start_pos, start_vel])
        
        if self.render_mode == "human":
            self.render()
        return np.array(self.state, dtype=np.float32), {}
    
    def step(self, action):
        assert self.action_space.contains(action), f"{action!r} ({type(action)}) invalid"
        
        position, velocity = self.state
        # Modified dynamics with variable mass (F = ma, so a = F/m)
        acceleration = (action - 1) * self.force / self.current_mass + np.cos(3 * position) * (-self.gravity)
        velocity += acceleration
        velocity = np.clip(velocity, -self.max_speed, self.max_speed)
        position += velocity
        position = np.clip(position, self.min_position, self.max_position)
        if position == self.min_position and velocity < 0:
            velocity = 0
            
        terminated = bool(position >= self.goal_position and velocity >= self.goal_velocity)
        reward = -1.0
        
        self.state = (position, velocity)
        if self.render_mode == "human":
            self.render()
        return np.array(self.state, dtype=np.float32), reward, terminated, False, {"mass": self.current_mass}