import numpy as np
import csv
import gymnasium as gym
import scipy.linalg as sla

# ----------------------------
# Utilities
# ----------------------------
def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def angles_from_obs(obs):
    # obs = [cos th1, sin th1, cos th2, sin th2, dth1, dth2]
    th1 = np.arctan2(obs[1], obs[0])
    th2 = np.arctan2(obs[3], obs[2])  # NOTE: theta2 is relative to link 1
    dth1 = obs[4]; dth2 = obs[5]
    return th1, th2, dth1, dth2

# ----------------------------
# Plant parameters (Gym defaults)
# ----------------------------
m1=m2=1.0
l1=l2=1.0
lc1=lc2=0.5
I1=I2=1.0
g  =9.8

# Useful derived constants at the upright for linearization (theta1=pi, theta2=0)
# d11,d12,d22 evaluated at q2=0:
d11_u = m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*1.0) + I1 + I2
d12_u = m2*(lc2**2 + l1*lc2*1.0) + I2
d22_u = m2*lc2**2 + I2
detD  = d11_u*d22_u - d12_u**2

# ----------------------------
# Linearization (Tedrake Ch.3) -> A,B at [pi,0,0,0]
# State x = [q1-π, q2, dth1, dth2]; input u = τ
# ----------------------------
def acrobot_AB_upright():
    # gravity Jacobian entries at upright (Tedrake Example 3.3)
    a31 = g*(m1*lc1 + m2*l1 + m2*lc2)/detD
    a32 = -g*(m2*lc2)/detD
    a41 = -g*(m2*lc2)/detD
    a42 =  g*(m2*lc2)/detD
    b3  = -d12_u/detD
    b4  =  d11_u/detD
    A = np.array([[0,0,1,0],
                  [0,0,0,1],
                  [a31,a32,0,0],
                  [a41,a42,0,0]], dtype=float)
    B = np.array([[0],[0],[b3],[b4]], dtype=float)
    return A,B

# ----------------------------
# CARE / LQR (with SciPy)
# ----------------------------
def lqr(A,B,Q,R):
    P = sla.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, P

# build LQR once
A,B = acrobot_AB_upright()
Q = np.diag([80.0, 40.0, 10.0, 5.0])   # sensible upright penalties
R = np.array([[0.2]])                  # torque penalty
K, _ = lqr(A,B,Q,R)                    # shape (1,4)

# ----------------------------
# PFL torque (Spong §2)
# ddth2 = ( u + (d2/d1)*phi1 - extra - phi2 ) / (bar_d22)
# => u = bar_d22*v2 - (d2/d1)*phi1 + extra + phi2
# 'extra' term is m2*l1*lc2*dth1^2*sin(th2);
# ----------------------------
def pfl_continuous_tau(th1, th2, dth1, dth2, v2):
    c2 = np.cos(th2); s2 = np.sin(th2)
    d1 = m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*c2) + I1 + I2
    d2 = m2*(lc2**2 + l1*lc2*c2) + I2
    phi2 = m2*lc2*g*np.cos(th1 + th2 - np.pi/2.0)
    phi1 = (-m2*l1*lc2*dth2**2*np.sin(th2)
            -2*m2*l1*lc2*dth2*dth1*np.sin(th2)
            + (m1*lc1+m2*l1)*g*np.cos(th1 - np.pi/2.0)
            + phi2)
    bar_d22 = (m2*lc2**2 + I2) - (d2**2)/d1
    extra = (m2*l1*lc2*dth1**2*np.sin(th2))
    tau = bar_d22*v2 - (d2/d1)*phi1 + extra + phi2
    return float(tau)

# ----------------------------
# Hybrid controller
# ----------------------------
class HybridAcrobotController:
    """
    Spong swing-up with PFL (arctan reference) -> LQR capture near upright.
    Works directly with Gym's observation and action spaces.
    """
    def __init__(self,
                 alpha=1.0,        # swing amplitude (rad) in q2^d = 2*alpha/pi*atan(dth1)
                 kp=18.0, kd=6.0,  # outer-loop PD on q2 tracking
                 switch_angles=0.35,     # |q1-π| and |q2| threshold to switch
                 switch_rates  =np.array([2.0, 4.0]),  # |dth1|,|dth2| limits at switch
                 torques=np.array([-1.0, 0.0, 1.0])): # Gym discrete torques
        self.alpha = alpha
        self.kp = kp
        self.kd = kd
        self.switch_angles = switch_angles
        self.switch_rates  = switch_rates
        self.torques = torques.astype(float)

    def is_upright_region(self, th1, th2, dth1, dth2):
        q1tilde = wrap_pi(th1 - np.pi)
        return (abs(q1tilde) < self.switch_angles and
                abs(th2)     < self.switch_angles and
                abs(dth1)    < self.switch_rates[0] and
                abs(dth2)    < self.switch_rates[1])

    def action(self, obs):
        th1, th2, dth1, dth2 = angles_from_obs(obs)

        # LQR near the top (linearization is for the original plant with input u=τ)
        if self.is_upright_region(th1, th2, dth1, dth2):
            x = np.array([wrap_pi(th1 - np.pi), th2, dth1, dth2])
            u_cont = float(-(K @ x).squeeze())
        else:
            # Spong swing-up with PFL
            q2d = 2.0*self.alpha/np.pi * np.arctan(dth1)       # (Spong eq. 28)
            v2  = self.kp*(q2d - th2) - self.kd*dth2           # realizable PD (Spong eq. 30)
            u_cont = pfl_continuous_tau(th1, th2, dth1, dth2, v2)

        # Map to the discrete torque set {-1,0,1}
        idx = np.argmin(np.abs(self.torques - u_cont))
        return int(idx), u_cont  # return both discrete action and raw torque for logging

if __name__ == "__main__":
    with open('obs.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['step', 'cos(th1)', 'sin(th1)', 'cos(th2)', 'sin(th2)', 'dth1', 'dth2', 'action', 'u_cont'])
    ctrl = HybridAcrobotController(alpha=1.0, kp=18.0, kd=6.0)
    env = gym.make("Acrobot-v1", render_mode="human")        # change to human for video
    for ep in range(1):
        obs, _ = env.reset(seed=ep)
        total = 0
        for step in range(1000):
            # print(f"obs: {obs}")
            action, u_cont = ctrl.action(obs)
            # print(f"action: {action}")
            # print(f"u_cont: {u_cont}")
            with open('obs.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow([step + 1, obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], action, u_cont])
            obs, reward, done, trunc, _ = env.step(action)
            total += reward
            # if done or trunc:
            #     break
        print(f"Episode {ep:02d}  |  total reward  {total}")
