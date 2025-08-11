import gymnasium as gym
import numpy as np
import math
from scipy.linalg import solve_continuous_are     # Riccati for LQR

# ════════════════════════════════════════════════════════════════════════
# helper – convert (cos,sin) observation back to angles
# ════════════════════════════════════════════════════════════════════════
def unpack(obs):
    c1, s1, c2, s2, d1, d2 = obs
    th1 = math.atan2(s1, c1)              # θ1  (0 = DOWN,  π = UP)
    th2 = math.atan2(s2, c2)              # θ2  (relative)
    return np.array([th1, th2, d1, d2])

# ════════════════════════════════════════════════════════════════════════
# physical parameters – identical to AcrobotPlant defaults
# ════════════════════════════════════════════════════════════════════════
m1 = m2 = 1.0
l1 = l2 = 1.0
lc1 = lc2 = 0.5
I1 = I2 = 1.0
g  = 9.8                                  # Gym constant

# from gymnasium.envs.classic_control.acrobot
# LINK_LENGTH_1 = 1.0  # [m]
# LINK_LENGTH_2 = 1.0  # [m]
# LINK_MASS_1 = 1.0  #: [kg] mass of link 1
# LINK_MASS_2 = 1.0  #: [kg] mass of link 2
# LINK_COM_POS_1 = 0.5  #: [m] position of the center of mass of link 1
# LINK_COM_POS_2 = 0.5  #: [m] position of the center of mass of link 2
# LINK_MOI = 1.0  #: moments of inertia for both links

# manipulator matrices   (Tedrake Eq. 3.Hacrobot)
def M(q2):
    c2 = math.cos(q2)
    return np.array([[I1 + I2 + m2*l1**2 + 2*m2*l1*lc2*c2,   I2 + m2*l1*lc2*c2],
                     [I2 + m2*l1*lc2*c2,                    I2              ]])

def tau_g(q1, q2):
    s1, s12 = math.sin(q1), math.sin(q1 + q2)
    return np.array([-(m1*g*lc1 + m2*g*l1)*s1 - m2*g*lc2*s12,
                     -m2*g*lc2*s12])

# ════════════════════════════════════════════════════════════════════════
# 1 ▸ ENERGY-SHAPING SWING-UP  (Spong ʼ94, Tedrake §3)
# ════════════════════════════════════════════════════════════════════════
E_des = g * (m1*lc1 + m2*(l1 + lc2))      # U at (θ1,θ2) = (π,0)

def swing_up(upright_tol=0.15):
    # textbook gains – kE slightly >1 helps with ±1 N·m saturation
    # kE, k1, k2, k3 = 2.0, 1.0, 2.0, 1.0
    kE = 3.0   # energy pump gain (higher = harder swing, but gated below)
    k1 = 8.0   # spring on th2 to keep links colinear
    k2 = 4.0   # damping on d2 to prevent elbow flapping
    k3 = 1.0   # weight on energy term after clipping/gating

    # safety shaping to preserve alignment while pumping
    th2_gate = 0.6   # rad; fade out energy when |th2| grows beyond this
    ubar_max = 2.0   # clip energy drive (prevents huge v)
    v_max    = 8.0   # clip commanded q2 acceleration

    def ctrl(obs):
        th1, th2, d1, d2 = unpack(obs)
        dq = np.array([d1, d2])

        # total mechanical energy  E = ½ dqᵀ M dq + U(q)
        U  = -(m1*g*lc1*math.cos(th1) +
               m2*g*( l1*math.cos(th1) + lc2*math.cos(th1 + th2)))
        E  = 0.5 * dq @ (M(th2) @ dq) + U
        E_tilde = E - E_des              # ★ correct sign

        u_bar_raw = d1 * E_tilde         #  \bar u  (Tedrake Eq. 3.72)
        # fade energy injection as misalignment grows; also clip
        gate = max(0.0, 1.0 - (abs(th2)/th2_gate)**2)
        u_bar = np.clip(u_bar_raw, -ubar_max, ubar_max) * gate

        # ν = q̈2ᵈ: strong alignment + gated energy pump
        v = -k1*th2 - k2*d2 + k3*(kE * u_bar)
        v = float(np.clip(v, -v_max, v_max))

        # collocated PFL torque τ  (algebra from text, no approximations)
        M11, M12 = M(th2)[0]
        M21, M22 = M(th2)[1]
        τ = M21 * (-(M21*v + tau_g(th1,th2)[0]) / M11) + M22*v - tau_g(th1,th2)[1]

        # map to Gym’s discrete actions  {0,1,2}  (τ≈–1,0,+1)
        return int(np.clip(np.round(τ), -1, 1) + 1)


    return ctrl

swing_ctrl = swing_up()

# ════════════════════════════════════════════════════════════════════════
# 2 ▸ LQR BALANCER  (linearise at [π,0,0,0] then solve CARE)
# ════════════════════════════════════════════════════════════════════════
A_lin = np.array([[0, 0, 1, 0],
                  [0, 0, 0, 1],
                  [0,  g*(m1*lc1 + m2*(l1 + lc2)) /
                      (I1 + I2 + m2*l1**2 + 2*m2*l1*lc2), 0, 0],
                  [0,  g*m2*lc2/I2,                       0, 0]])

B_lin = np.array([[0],
                  [0],
                  [1/(I1 + I2 + m2*l1**2 + 2*m2*l1*lc2)],
                  [1/I2]])

Q = np.diag([1, 1, 1, 1])
R = np.array([[1]])

# pre-compute Riccati once
P = solve_continuous_are(A_lin, B_lin, Q, R)
K = (B_lin.T @ P) / R                      # 1×4 gain row

def lqr_ctrl(obs):
    th1, th2, d1, d2 = unpack(obs)
    x = np.array([th1 - math.pi, th2, d1, d2])   # shift to equilibrium
    τ = -float(K @ x)
    return int(np.clip(np.round(τ), -1, 1) + 1)

# ════════════════════════════════════════════════════════════════════════
# 3 ▸ COMBINED POLICY
# ════════════════════════════════════════════════════════════════════════
def acrobot_policy(obs):
    th1, th2, *_ = unpack(obs)
    if abs(th1 - math.pi) < 0.3 and abs(th2) < 0.25:
        print("lqr")
        return lqr_ctrl(obs)
    else:
        print("swing")
        return swing_ctrl(obs)

# ════════════════════════════════════════════════════════════════════════
# quick demo
# ════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    env = gym.make("Acrobot-v1", render_mode=None)        # change to human for video
    for ep in range(5):
        obs, _ = env.reset(seed=ep)
        total = 0
        for _ in range(1000):
            action = acrobot_policy(obs)
            obs, reward, done, trunc, _ = env.step(action)
            total += reward
            # if done or trunc:
            #     break
        print(f"Episode {ep:02d}  |  total reward  {total}")
