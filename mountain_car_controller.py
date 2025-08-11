import numpy as np
from mountain_car_env import CustomMountainCarEnv
import matplotlib.pyplot as plt

# height surrogate derivation:
## from gym: velocity += (action - 1) * self.force + math.cos(3 * position) * (-self.gravity)
# p_ddot = -g*dy/dp
# from gym update: dy/dp = cos(3p)
# integrate to get h(p) = sin(3p)/3

class MountainCarController:
    def __init__(self, goal_position=0.5):
        self.goal_position = goal_position
        self.goal_energy = self.calculate_goal_energy()
        self.current_state = None

    def calculate_goal_energy(self):
        """Calculate E* = h(goal_position) where h(p) = sin(3p)/3"""
        return (np.sin(3 * self.goal_position)) / 3.0

    def energy(self, pos, vel):
        """Height surrogate h(p) = sin(3p)/3, total energy E(p,v) = v²/2 + h(p)"""
        return 0.5 * vel**2 + np.sin(3 * pos) / 3.0

    def update_state(self, state):
        """Update the current state"""
        self.current_state = state

    def get_current_energy(self):
        """Get energy of current state"""
        if self.current_state is None:
            return 0.0
        pos, vel = self.current_state
        return self.energy(pos, vel)

    def bang_bang_policy(self, state):
        """Bang-bang control policy"""
        pos, vel = state
        E = self.energy(pos, vel)
        if E < self.goal_energy:
            # pump in direction of current motion; if parked, nudge left
            return 0 if vel < 0 else 2
        else:
            # commit toward the flag (right hill)
            return 2

    def get_action(self, state):
        """Get action for given state"""
        self.update_state(state)
        return self.bang_bang_policy(state)



if __name__ == "__main__":
    env = CustomMountainCarEnv(render_mode="human", goal_velocity=0, mass_range=(0.5, 2.0))
    
    # Create controller with goal position 0.5
    controller = MountainCarController(goal_position=0.5)
    print(f"Goal energy E* = h(0.5) = {controller.goal_energy:.6f}")
    
    for ep in range(10):
        s, info = env.reset()
        mass = env.current_mass
        start_pos, start_vel = s
        # control vs time
        # state vs time
        
        print(f"\nEpisode {ep:02d}:")
        print(f"  Mass: {mass:.3f}")
        print(f"  Start position: {start_pos:.3f}")
        print(f"  Start velocity: {start_vel:.3f}")
        print(f"  Start energy: {controller.energy(start_pos, start_vel):.6f}")
        
        total = 0
        done = truncated = False
        steps = 0
        while not (done or truncated) and steps < 200:
            a = controller.get_action(s)
            s, r, done, truncated, info = env.step(a)
            total += r
            steps += 1
            
        final_pos, final_vel = s
        print(f"  Final position: {final_pos:.3f}")
        print(f"  Final velocity: {final_vel:.3f}")
        print(f"  Final energy: {controller.get_current_energy():.6f}")
        print(f"  Steps: {steps}, Total reward: {total}")
        print(f"  Success: {done}")
    
    env.close()
