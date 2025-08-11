# Swing Up Control of the Acrobot * 

Mark W. Spong<br>Coordinated Science Laboratory<br>University of Illinois at Urbana-Champaign<br>1308 W. Main St., Urbana, Ill. 61801<br>spong@lagrange.csl.uiuc.edu


#### Abstract

In this paper we investigate the problem of swing up control of the Acrobot, a two-link, underactuated robot that is a useful vehicle to study problems in nonlinear control. We develop a swing up strategy based on partial feedback linearization. The algorithm works by creating "unstable zero dynamics" which drives the first link of the Acrobot away from its open loop stable equilibrium toward the inverted position. Control is switched to a linear controller, designed to balance the arm about the inverted configuration, whenever the swing up controller moves the Acrobot into the near vertical position. Simulation results are presented showing the performance of the system.


Key Words: control theory, robotics, Acrobot, underactuated systems, mechatronics.

## 1 Introduction

The Acrobot is a planar, two-link robot with an actuator at the elbow (joint 2) but no actuator at the shoulder (joint 1). The underactuated nature of the Acrobot, therefore, makes it a useful vehicle to study problems in robotics and nonlinear control theory (Refer to Figure 1). Previous work in this area has investigated the problem of balancing the Acrobot in its inverted position and controlling its motion along its unstable equilibrium manifold [11], [2], [3]. The problem studied in this paper is to swing the Ac robot from its open loop stable equilibrium to its inverted position and balance it. Related research has dealt with the swing up control of the inverted pendulum. Results have been published based on energy and Lyapunov techniques [5], [10], [14], and re-

[^0]ward/punishment learning techniques [9] to name a few.

In this paper we derive a swing up algorithm based on the method of partial feedback linearization [7]. The control linearizes and decouples the motion of the second link from that of the first link in a nonlinear inner loop. The motion of the first link, which is excited by the motion of the second link, then represents internal dynamics. We next design an outer loop control to command the motion of the second link as a feedback function of the velocity of the first link. We will see that the combined strategy works by creating "unstable zero dynamics," which drive the first link away from it's open loop stable position toward the inverted configuration. In this case the unstable zero dynamics actually helps to achieve the swing up goal, which is an interesting observation in itself.

A key problem, once the swing up controller is designed is to balance the Acrobot at the top of its swing. The approach used here is to guarantee that the swing up algorithm moves the robot into the basin of attraction of a previously designed balancing controller and then switching algorithms when the basin of attraction is entered. We will demonstrate the performance of such a strategy using a linear quadratic regulator to capture and balance the Acrobot at the top of its swing.

## 2 Partial Feedback Linearization

The equations of motion of the Acrobot shown in Figure 1 are just the standard equations of motion of a two-link planar robot [13] except that there is no independent input to the first equation, i.e.,

$$
\begin{aligned}
& d_{11} \ddot{q}_{1}+d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}=0 \\
& d_{12} \ddot{q}_{1}+d_{22} \ddot{q}_{2}+h_{2}+\phi_{2}=r
\end{aligned}
$$


[^0]:    *This research is partially supported by the National Science Foundation under grants MSS-0100618 and IRI-9216428

where

$$
\begin{aligned}
d_{11} & =m_{1} \ell_{c 1}^{2}+m_{2}\left(\ell_{1}^{2}+\ell_{c 2}^{2}+2 \ell_{1} \ell_{c 2} \cos \left(q_{2}\right)\right)+I_{1}+I_{2} \\
d_{22} & =m_{2} \ell_{c 2}^{2}+I_{2} \\
d_{12} & =m_{2}\left(\ell_{c 2}^{2}+\ell_{1} \ell_{c 2} \cos \left(q_{2}\right)\right)+I_{2} \\
h_{1} & =-m_{2} \ell_{1} \ell_{c 2} \sin \left(q_{2}\right) \dot{q}_{2}^{2}-2 m_{2} \ell_{1} \ell_{c 2} \sin \left(q_{2}\right) \dot{q}_{2} \dot{q}_{1} \\
h_{2} & =m_{2} \ell_{1} \ell_{c 2} \sin \left(q_{2}\right) \dot{q}_{1}^{2} \\
\phi_{1} & =\left(m_{1} \ell_{c 1}+m_{2} \ell_{1}\right) g \cos \left(q_{1}\right)+m_{2} \ell_{c 2} g \cos \left(q_{1}+q_{2}\right) \\
\phi_{2} & =m_{2} \ell_{c 2} g \cos \left(q_{1}+q_{2}\right)
\end{aligned}
$$

![img-0.jpeg](img-0.jpeg)

Figure 1: The Acrobot
If we solve for $\ddot{q}_{1}$ in equation (1) as

$$
\ddot{q}_{1}=-d_{11}^{-1}\left(d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}\right)
$$

and substitute the resulting expression (3) into (2), we obtain

$$
\ddot{d}_{22} \ddot{q}_{2}+\ddot{h}_{2}+\ddot{\phi}_{2}=\tau
$$

where the terms $\ddot{d}_{22}, \dot{h}_{2}, \dot{\phi}_{2}$ are given by

$$
\begin{aligned}
\ddot{d}_{22} & =d_{22}-d_{21} d_{11}^{-1} d_{12} \\
\ddot{h}_{2} & =h_{2}-d_{21} d_{11}^{-1} h_{1} \\
\ddot{\phi}_{2} & =\phi_{2}-d_{21} d_{11}^{-1} \phi_{1}
\end{aligned}
$$

The term $d_{11}$ is positive and is bounded away from zero by the uniform positive definiteness of the robot's inertia matrix. Also, it can be seen that $\ddot{d}_{22}=d_{11}^{-1} \Delta$, where $\Delta=d_{11} d_{22}-d_{21} d_{12}$ is the determinant of the robot's inertia matrix. Hence $d_{22}$ is also positive and bounded away from zero.

Therefore, a feedback linearizing controller can be defined for equation (4) according to

$$
\tau=\ddot{d}_{22} v_{2}+\ddot{h}_{2}+\ddot{\phi}_{2}
$$

where $v_{2} \in R^{m}$ is an additional (outer loop) control input yet to be defined. The complete system up to this point may be written as

$$
\begin{aligned}
d_{11} \ddot{q}_{1}+h_{1}+\phi_{1} & =-d_{12} v_{2} \\
\ddot{q}_{2} & =v_{2}
\end{aligned}
$$

We see that, with respect to an output $y_{2}=q_{2}$, the input/output system is linear and second order. This input/output system therefore has relative degree 2 [7] and the equation (6) represents the internal dynamics.

If $y_{2}^{d}=q_{2}^{d}(t)$ represents a command reference for the active joint, then we may choose the additional control term $v_{2}$ as

$$
v_{2}=\ddot{q}_{2}^{d}+k_{d}\left(\dot{q}_{2}^{d}-\dot{q}_{2}\right)+k_{p}\left(q_{2}^{d}-q_{2}\right)
$$

where $k_{p}$ and $k_{d}$ are positive gains. With state variables

$$
\begin{array}{ll}
z_{1}=q_{2}-q_{2}^{d} & z_{2}=\dot{q}_{2}-\dot{q}_{2}^{d} \\
\eta_{1}=\dot{q}_{1} & \eta_{2}=\dot{q}_{1}
\end{array}
$$

and output error $\ddot{y}_{2}=y_{2}-y_{2}^{d}$, the complete closed loop system may be written as

$$
\begin{aligned}
\dot{z}_{1}= & z_{2} \\
\dot{z}_{2}= & -k_{p} z_{1}-k_{d} z_{2} \\
\dot{\eta}_{1}= & \eta_{2} \\
\dot{\eta}_{2}= & -d_{11}^{-1}\left(h_{1}+\phi_{1}\right) \\
& -d_{11}^{-1} d_{12}\left(\ddot{q}_{2}^{d}-k_{p} z_{1}-k_{d} z_{2}\right) \\
\ddot{y}_{2}= & z_{1}
\end{aligned}
$$

In matrix form we write this as

$$
\begin{aligned}
\dot{z} & =A z \\
\dot{\eta} & =w(z, \eta, t) \\
\ddot{y}_{2} & =C z
\end{aligned}
$$

where $z^{T}=\left(z_{1}^{T}, z_{2}^{T}\right), \eta^{T}=\left(\eta_{1}^{T}, \eta_{2}^{T}\right)$, the matrices $A$ and $C$ are given by

$$
A_{1}=\left[\begin{array}{cc}
0 & 1 \\
-k_{p} & -k_{d}
\end{array}\right] \quad ; \quad C=[1,0]
$$

and the function $w(z, \eta, t)$ is given by

$$
\begin{aligned}
& w(z, \eta, t)= \\
& \binom{\eta_{2}}{-d_{11}^{-1}\left(h_{1}+\phi_{1}\right)-d_{11}^{-1} d_{12}\left(\ddot{q}_{2}^{d}-k_{p} z_{1}-k_{d} z_{2}\right)}
\end{aligned}
$$

We see from (15) and (16) that the surface $z=0$ in state space defines an integral manifold for the system.

Since $A$ is Hurwitz for positive values of gains $k_{p}$ and $k_{d}$ this manifold is globally attractive. The dynamics on the integral manifold are given by

$$
\dot{\eta}=w(0, \eta, t)
$$

and define the zero dynamics [7] relative to the output $\dot{y}=q_{2}-q_{2}^{d}$. We can state the following result whose proof can be found in [7] (see also [8]).

Theorem 1. Consider the system (15)-(17). Suppose that $w\left(0, \eta_{0}, t\right)=0$ for $t \geq 0$, i.e. $\left(0, \eta_{0}\right)$ is an equilibrium of the full system (15)-(17) and $\eta_{0}$ is an equilibrium of the zero dynamics (20). Suppose also that $A$ is a Hurwitz matrix. Then $\left(0, \eta_{0}\right)$ of the full system (15)-(17) is locally stable (respectively, locally asymptotically stable, unstable) if $\eta_{0}$ is locally stable (respectively, locally asymptotically stable, unstable) for the zero dynamics (20).

The point of this theorem is that the local stability properties of the full system may be determined based on the analysis of two reduced order systems, namely (15) and (20). An important point to note is that the Jacobian linearization of (16) may have eigenvalues on the imaginary axis and so not give sufficient information about the stability properties of the full nonlinear system. The proof of this result utilizes the Center Manifold Theorem and the reader is referred to [7], and $[8]$ for details.

## 3 The Swing Up Control

So far we have used an inner loop nonlinear feedback law to linearize and decouple the response of $q_{2}$ from that of $q_{1}$. Note that the response of $q_{1}$ is neither linear nor decoupled from the motion of $q_{2}$. Indeed, the motion of the second link will excite the motion of the first link, whose response represents the internal dynamics. The task is now to properly command the motion of $q_{2}$ so that $q_{1}$ moves away from it's initial position $-\pi / 2$ to the inverted position $\pi / 2$. The crucial step is, therefore, the determination of the reference input $q_{2}^{d}$ for the second link.

The basic idea behind our swingup strategy is to swing the second link between fixed values $\pm \alpha$ in order to "pump energy" into the system and then to schedule the transition of the second link between these two values $\pm \alpha$ "in phase" with the motion of the first link in such a way that the amplitude of the swing of the first link increases with each swing. We do this by making the reference $q_{2}^{d}$ for link 2 a feedback function of the velocity $\dot{q}_{1}$ of link 1 . The simplest strategy is
just to swing $q_{2}$ between fixed values $\pm \alpha$ whenever the velocity $\dot{q}_{1}$ crosses zero, i.e.

$$
q_{2}^{d}=\alpha \operatorname{sgn}\left(\dot{q}_{1}\right)
$$

where $\operatorname{sgn}()$ is the signum function shown in Figure 2.
![img-1.jpeg](img-1.jpeg)

Figure 2: Saturation Function
To see why one might expect such an approach to work consider the motion of a single link with a force $F$ acting at the end of the link as shown in Figure 3. Assume that the force $F$ is directed perpendicular to
![img-2.jpeg](img-2.jpeg)

Figure 3: Single Link
the link for simplicity. Then the torque acting at the joint is equal to $\ell F$ and the equation of motion is

$$
I \ddot{q}_{1}+m g \ell_{c} \sin \left(q_{1}\right)=\ell F
$$

The total energy of the system is given by

$$
V=\frac{1}{2} I \dot{q}_{1}^{2}+m g \ell_{c}\left(1-\cos \left(q_{1}\right)\right)
$$

and the derivative of $V$ along trajectories of the system is given by

$$
\dot{V}=\ell F \dot{q}_{1}
$$

Therefore, the change in total energy over a time interval $[T-1, T]$ is

$$
V(T)-V(T-1)=\ell \int_{T-1}^{T} F(t) \dot{q}_{1}(t) d t
$$

Suppose that $F$ satisfies,

$$
F=|f(t)| \operatorname{sgn}\left(\dot{q}_{1}(t)\right)
$$

Then we see from (25) that

$$
V(T)-V(T-1)=\ell \int_{T-1}^{T}|f(t)| \cdot\left|\dot{q}_{1}\right| d t \geq 0
$$

i.e., the change in energy during the time interval $[T-1, T]$ is nonnegative. Our strategy for swinging the second link using $\operatorname{sgn}\left(\dot{q}_{1}\right)$ as its reference command is thus designed to increase the total energy of the first link with each swing. Since all of the energy is potential energy at the top of each swing, this means that the amplitude of the swing will increase each time.

Although this reference command can be used to swing up the Acrobot, it turns out that an even better approach is to smooth the above signum function according to

$$
q_{2}^{d}=2 \alpha / \pi \arctan \left(\dot{q}_{1}\right)
$$

as shown in Figure 4.
![img-3.jpeg](img-3.jpeg)

Figure 4: Arctangent Function
Any smooth, bounded "first and third quadrant" function may be used. The arctangent function is merely a convenient choice. This modification is desirable for two reasons. First, it straightens out the second link at the top of each swing when $\dot{q}_{1}$ becomes zero, and, second, a smooth reference command allows us to to compute and analyze the zero dynamics. It is important to note that our choice of reference command to link 2 as a pure feedback function of $\dot{q}_{1}$ renders the system autonomous. Therefore the zero dynamics evolve on an invariant manifold in state space.

Substituting (28) into (20) yields, after a straightforward calculation, the following expression for the zero dynamics

$$
\begin{aligned}
\frac{d_{12}\left(q_{2}^{d}\right)}{1+q_{1}^{2}} \frac{d^{2} q_{1}}{d t^{2}}+ & d_{11}\left(q_{2}^{d}\right) \dot{q}_{1}-2 \frac{d_{12}\left(q_{2}^{d}\right) \dot{q}_{1}}{\left(1+q_{1}^{2}\right)^{2}} \dot{q}_{1}^{2} \\
& +h_{1}\left(\dot{q}_{1}, q_{2}^{d}, \dot{q}_{2}^{d}\right)+\phi\left(q_{1}, q_{2}^{d}\right)=0
\end{aligned}
$$

Remarks: We see that the zero dynamics for this system is an autonomous third order nonlinear system. This can be explained as follows. Using the expression (28) for the reference command, $q_{2}^{d}$, in the outer loop control (8) means that the feedforward terms $\dot{q}_{1}^{d}$ and $\dot{q}_{2}^{d}$ contain the acceleration and jerk, respectively, of link 1 and thus the order of the system is increased by one.

We note, however, that the control law (8) is not realizable using only position and velocity measurements. In order to obtain a realizable control input, therefore, we will use, instead of (8), the control

$$
v_{2}=k_{p}\left(q_{2}^{d}-q_{2}\right)-k_{d} \dot{q}_{2}
$$

i.e., (8) without the feedforward terms $\dot{q}_{2}^{d}$ and $\ddot{q}_{2}^{d}$. The control law (30) requires only position and velocity measurements to implement. The price we pay for the simplified outer loop control (30) is that the $z$ coordinates are no longer exactly decoupled from the $\eta$ coordinates in (15)-(16) and the manifold $z=0$ is no longer exactly invariant. However, this is not crucial for the swingup motion and we shall see that the response using the (realizable) outer loop term (30) is nearly the same as it would be with the (unrealizable) outer loop term (8).

## 4 Simulations

The dynamic equations (1)-(2) were simulated in Simnon [4], using the parameters in Table 1 below. Figure 5 shows the response of the zero dynamics (30),

| $m_{1}$ | $m_{2}$ | $\ell_{1}$ | $\ell_{2}$ | $\ell_{c_{1}}$ | $\ell_{c_{2}}$ | $I_{1}$ | $I_{2}$ | $g$ |
| :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |
| 1 | 1 | 1 | 1 | 0.5 | 0.5 | 0.2 | 1.0 | 9.8 |

Table 1: Parameters of the Simulated Acrobot
and Figure 6 shows a portion of the phase portrait (the projection onto the $\left(q_{1}, \dot{q}_{1}\right)$ plane). We see that the equilibrium $q_{1}=-\pi / 2, \dot{q}_{1}=0$ is unstable.

Figure 7 show the response of link 1 for the actual system using the outer loop control (30). Note that the response is quite similar to the response of the ideal zero dynamics (30). As the gains $k_{p}$ and $k_{d}$ are increased in (30) the response of the system becomes nearly the same as the response predicted by the ideal case.

The swing up motion is now accomplished by combining the above partial feedback linearization control with a Linear Quadratic Regulator. Control is

![img-4.jpeg](img-4.jpeg)

Figure 5: Response of the Ideal Zero Dynamics (Plotted modulo $2 \pi$ with Initial Condition $q_{1}(0)=-1.35$ )
![img-5.jpeg](img-5.jpeg)

Figure 6: A Portion of the Phase Portrait of the Ideal Zero Dynamics
switched to the linear regulator to balance the Acrobot about the vertical when the Acrobot enters the basin of attraction of the linear regulator. Figure 8 shows a swing up motion using the reference $q_{2}^{d}$ for $q_{2}$ given by (28). Figure 9 shows a plot of the total energy during the swingup motion. We note that the energy is nearly monotonically increasing during the entire motion which shows that the swing up strategy is quite efficient.

## 5 The Balancing Controller

Linearizing the Acrobot dynamics about the vertical equilibrium $q_{1}=\pi / 2, q_{2}=0$ with the parameters from Table 1 results in the controllable linear system

$$
\dot{x}=A x+B u
$$

where the state vector $x=\left(q_{1}-\pi / 2, q_{2}, \dot{q}_{1}, \dot{q}_{2}\right)$, the control input $u=\tau$, and the matrices $A$ and $B$ are
![img-6.jpeg](img-6.jpeg)

Figure 7: Actual Response of Link 1 with Realizable Control (30) (Initial Condition $q_{1}(0)=-1.1$ )
![img-7.jpeg](img-7.jpeg)

Figure 8: Swingup and Balance of The Acrobot
given by

$$
A=\left[\begin{array}{rrrr}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
10.19 & -1.57 & 0 & 0 \\
-10.35 & 6.12 & 0 & 0
\end{array}\right] ; B=\left[\begin{array}{r}
0 \\
0 \\
-1.12 \\
2.37
\end{array}\right]
$$

Using Matlab, an LQR controller was designed with weighting matrices

$$
Q=\left[\begin{array}{rrrr}
1000 & -500 & 0 & 0 \\
-500 & 1000 & 0 & 0 \\
0 & 0 & 1000 & -500 \\
0 & 0 & -500 & 1000
\end{array}\right] ; R=0.5
$$

yielding the state feedback controller $u=-K x$, where

$$
K=[-1650.0,-460.2,-716.1,-278.2]
$$

The control law (34) is switched on whenever $q_{1}$ is greater than a prescribed value $\beta$ which is determined by trial and error.

![img-8.jpeg](img-8.jpeg)

Figure 9: Total Energy During the Swing Up Motion

## 6 Conclusions

In this paper we have produced a swing up controller for the Acrobot based on the method of partial feedback linearization. It is interesting to note that the entire swingup motion is produced by the "natural response" of an autonomous system. In other words by making the commanded motion of the second depend only on the velocity of the first link the system becomes an autonomous system of differential equations whose solution executes the swingup motion. Research is now underway investigating other issues, such as robustness, in order to develop learning and adaptive strategies to stand the Acrobot optimally. Also, experiments are being performed on a real Acrobot to verify the theory.

## References

[1] Berkemeier, M., and Fearing, R.S., "Control of a Two-Link Robot to Achieve Sliding and Hopping Gaits," Proc. 1992 IEEE Int. Conf. on Robotics and Automation, Nice, France, pp. 286-291, May, 1992.
[2] Bortoff, S.A., Pseudolinearization using Spline Functions with Application to the Acrobot, PhD Thesis, Dept. of Electrical and Computer Engineering, University of Illinois at UrbanaChampaign, 1992.
[3] Bortoff, S., and Spong, M.W., "Pseudolinearization of the Acrobot Using Spline Functions," IEEE Conf. on Decision and Control, Tucson, AZ, pp. 593-598, Dec. 1992.
[4] Elmquist, H., Simnon-User's Guide, Department of Automatic Control, Lund Inst. of Tech., CODEN:LUTFD2/(TFRT-3091), 1975.
[5] Furuta, K., and Yamakita, M., "Swing up Control of Inverted Pendulum," In IECON'91, pp. 21932198, 1991.
[6] J. Hauser and R. M. Murray. Nonlinear controllers for non-integrable systems: the acrobot example. In Proc. American Control Conference, 1990.
[7] Isidori, A., Nonlinear Control Systems, 2nd Edition, Springer-Verlag, Berlin, 1989.
[8] Khalil, H., Nonlinear Systems, Macmillan Publishing Co., New York, 1992.
[9] Kratochwil, K., Engelbrecht, R., and Jorgl, H., "A Reward/Punishment Learning Method to Swing up a Pendulum into its Upright Position," Proc. IFAC Symposium, Sydney, Australia, 1993.
[10] Mori, S., Nishihara, H., and Furuta, K., "Control of Unstable Mechanical Systems: Control of Pendulum," Int. J. Control, v. 23, pp. 673-692, 1976.
[11] Murray, R.M., and Hauser, J., "A Case Study in Approximate Linearization: The Acrobot Example," Proc. American Control Conference, 1990.
[12] Saito, F., Fukuda, T., and Arai, F., "Swing and Locomotion Control for Two-Link Brachiation Robot," Proc. 1993 IEEE Int. Conf. on Robotics and Automation, pp. 719-724, Atlanta, GA, 1993.
[13] Spong, M.W., and Vidyasagar, M., Robot Dynamics and Control, John Wiley \& Sons, Inc., New York, 1989.
[14] Wiklund, M., Kristenson, A., and Astrom, K.J., "A New Strategy for Swinging up an Inverted Pendulum," Proc. IFAC Symposium, Sydney, Australia, 1993.
![img-0.jpeg](img-0.jpeg)

# The Swing Up Control Problem For The Acrobot 

Mark W. Spong

Underactuated mechanical systems are those possessing fewer actuators than degrees of freedom. Examples of such systems abound, including flexible joint and flexible link robots, space robots, mobile robots, and robot models that include actuator dynamics and rigid body dynamics together. Complex internal dynamics, nonholonomic behavior, and lack of feedback linearizability are often exhibited by such systems, making the class a rich one from a control standpoint. In this article we study a particular underactuated system known as the Acrobot: a two-degree-of-freedom planar robot with a single actuator. We consider the so-called swing up control problem using the method of partial feedback linearization. We give conditions under which the response of either degree of freedom may be globally decoupled from the response of the other and linearized. This result can be used as a starting point to design swing up control algorithms. Analysis of the resulting zero dynamics as well as analysis of the energy of the system provides an understanding of the swing up

[^0]algorithms. Simulation results are presented showing the swing up motion resulting from partial feedback linearization designs.

## Introduction

In this paper we study the swing up control problem for the Acrobot, a two-link, underactuated robot that we are using to study problems in nonlinear control and robotics (refer to Fig. (1)). The Acrobot dynamics are complex enough to yield a rich source of nonlinear control problems, yet simple enough to permit a complete mathematical analysis.

The swing up control problem is to move the Acrobot from its stable downward position to its unstable inverted position and balance it about the vertical. Because of the large range of motion, the swing up problem is highly nonlinear and challenging. We derive two distinct algorithms for the swing up control. Both of our algorithms are based on the notion of partial feedback linearization [11], but also share a common design philosophy with the recent method of integrator backstepping [12]. As we shall see, our first algorithm is useful in the case that there are no limits on the rotation of the second link, while our second algorithm can be used in cases where the second link is restricted to less than a full $360^{\circ}$ rotation.

The Acrobot model that we use is a two-link planar robot arm with an actuator at the elbow (joint 2) but no actuator at the shoulder (joint 1). The equations of motion of the system are [23]


[^0]:    The author is with The Coordinated Science Laboratory, University of Illinois at Urbana-Champaign, 1308 W. Main St., Urbana, IL, 61801. This research was partially supported by the National Science Foundation under grants MSS-9100618, IRI-9216428, and CMS-9402229. A preliminary version of this paper was presented at the 1994 IEEE Int. Conf. on Robotics and Automation, San Diego, May 1994.

![img-1.jpeg](img-1.jpeg)

Fig. 1. The Acrobot.

$$
\begin{aligned}
& d_{11} \ddot{q}_{1}+d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}=0 \\
& d_{21} \ddot{q}_{1}+d_{22} \ddot{q}_{2}+h_{2}+\phi_{2}=\tau
\end{aligned}
$$

where

$$
\begin{aligned}
& d_{11}=m_{1} l_{c 1}^{2}+m_{2}\left(l_{1}^{2}+l_{c 2}^{2}+2 l_{1} l_{c 2} \cos \left(q_{2}\right)\right)+I_{1}+I_{2} \\
& d_{22}=m_{2} l_{c 2}^{2}+I_{2} \\
& d_{12}=m_{2}\left(l_{c 2}^{2}+l_{1} l_{c 2} \cos \left(q_{2}\right)\right)+I_{2} \\
& d_{21}=m_{2}\left(l_{c 2}^{2}+l_{1} l_{c 2} \cos \left(q_{2}\right)\right)+I_{2} \\
& h_{1}=-m_{2} l_{1} l_{c 2} \sin \left(q_{2}\right) \dot{q}_{2}^{2}-2 m_{2} l_{1} l_{c 2} \sin \left(q_{2}\right) \dot{q}_{2} \dot{q}_{1} \\
& h_{2}=m_{2} l_{1} l_{c 2} \sin \left(q_{2}\right) \dot{q}_{1}^{2} \\
& \phi_{1}=\left(m_{1} l_{c 1}+m_{2} l_{1}\right) g \cos \left(q_{1}\right)+m_{2} l_{c 2} g \cos \left(q_{1}+q_{2}\right) \\
& \phi_{2}=m_{2} l_{c 2} g \cos \left(q_{1}+q_{2}\right)
\end{aligned}
$$

The difference between the system (1)-(2) and the standard model of a two-link planar robot [23] is, of course, the absence of an input torque to the first equation (1).

There have been a number of previous studies of underactuated mechanical systems; only a few will be mentioned here. The term "Acrobot" was coined at Berkeley, where the first studies of its controllability properties were performed by Murray and Hauser [14]. More recently, Berkemeier and Fearing [3] have investigated the application of nonlinear control to achieve sliding and hopping gaits of an Acrobot that has its first link free, as opposed to this paper in which the first link is pinned.

The first experimental results for the Acrobot were produced by Bortoff [5] in his Ph.D. thesis. The technique of pseudolinearization was used to design both observers and controllers to balance the Acrobot along its (unstable) equilibrium manifold of balancing configurations. The so-called Rolling Acrobot, which is similar to the mechanism of Berkemeier and Fearing, was also studied in this thesis (see also [4]).

In [17] a similar mechanism was designed and built to investigate so-called brachiation motions. Excellent experimental results were achieved using control algorithms quite different from the type considered here. The control of other gymnast-type robots has been considered in [24, 25] and [18, 20]. The control of manipulators with passive joints has been considered in [1]
and [2]. These mechanisms used brakes on the passive joints, which introduces a reduced amount of actuation to the passive joints that is unavailable for the Acrobot.

The area of space robotics contains many opportunities for the study of underactuated systems. The papers by Papadopoulos and Dubowsky $[8,15,16]$, for example, have shown the existence of so-called dynamic singularities in the task space control which greatly complicates the control problems.

A number of other related studies can be mentioned, such as the control of the more classical inverted pendulum [9]. Most previous works have used open loop strategies, sinusoidal excitation, etc., for swing up control. A notable exception is the paper [26], which discusses controlling the energy of the system; an approach related to the one of the algorithms in this paper.

## Partial Feedback Linearization

It has been shown [14] that the Acrobot dynamics are not feedback linearizable with static state feedback and nonlinear coordinate transformation. This is typical of a large class of underactuated mechanical systems. However, as we will show, we may achieve a linear response from either degree of freedom by suitable nonlinear feedback. In this section, we derive and analyze two distinct nonlinear controllers to achieve two distinct systems, which we call $\Sigma_{1}$ and $\Sigma_{2}$, and which represent the linearization of the response of link 1 and link 2 , respectively. We will use these two systems to generate two distinct approaches for the swing up control problem.

The easiest way to see how the partial feedback linearization is accomplished is as follows. In equation (1) suppose that we solve for either $\ddot{q}_{2}$ or $\ddot{q}_{1}$ and use the resulting expression in the second equation (2). In this way the second equation will be a feedback linearizable equation involving only $\ddot{q}_{1}$ in the first case or only $\ddot{q}_{2}$ in the second case. Upon choosing $\tau$ to linearize the resulting equation (2), we achieve either the system $\Sigma_{1}$

$$
\begin{gathered}
d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}=-d_{11} v_{1} \\
\ddot{q}_{1}=v_{1}
\end{gathered}
$$

or the system $\Sigma_{2}$

$$
\begin{gathered}
d_{11} \ddot{q}_{1}+h_{1}+\phi_{1}=-d_{12} v_{2} \\
\ddot{q}_{2}=v_{2}
\end{gathered}
$$

where the terms $v_{1}$ and $v_{2}$ are additional (outer loop) control inputs to be designed. (This will be clarified below.) We use the term non-collocated linearization to describe the system $\Sigma_{1}$ since the unactuated joint response is linearized, and we use the term collocated linearization to describe the system $\Sigma_{2}$ in which the actuated joint response is linearized. (See [20] for further details.)

Thus, under conditions that we will state below, the systems $\Sigma_{1 \text { and } \Sigma_{2}}$ are both feedback equivalents of the Acrobot dynamics. Either of these systems, $\Sigma_{1}$ or $\Sigma_{2}$, may be used to generate a swing up control strategy, as we will show, after first giving the details of the derivations of $\Sigma_{1}$ and $\Sigma_{2}$.

Derivation of the System $\Sigma 1$ : The Non-Collocated Case
Consider the first equation (1)

$$
d_{11} \ddot{q}_{1}+d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}=0
$$

and assume that the term

$$
d_{12}=m_{2}\left(l_{c 2}^{2}+l_{1} l_{c 2} \cos \left(q_{2}\right)\right)+I_{2}
$$

is nonzero for all values of $q 2$. This condition is termed strong inertial coupling in [18] and generalizes to the multi-degree-offreedom case where $d_{12}$ is a matrix function of $q_{2}$. Note that the strong inertial coupling condition imposes some restrictions on the inertia parameters of the robot, namely that $I_{2}>m_{2} l_{c 2}\left(l_{1}-l_{c 2}\right)$. Under this assumption we can solve for $\ddot{q}_{2}$ from (7) as

$$
\ddot{q}_{2}=-\frac{1}{d_{12}}\left(d_{11} \ddot{q}_{1}+h_{1}+\phi_{1}\right)
$$

and substitute the resulting expression (8) into (2) to obtain

$$
\dddot{q}_{1} \ddot{q}_{1}+\ddot{h}_{1}+\ddot{\phi}_{1}=\tau
$$

where the terms $\dddot{q}_{1}, \ddot{h}_{1}, \ddot{\phi}_{1}$ are given by

$$
\begin{aligned}
\ddot{d}_{1} & =d_{21}-d_{22} d_{11} / d_{12} \\
\ddot{h}_{1} & =h_{2}-d_{22} h_{1} / d_{12} \\
\ddot{\phi}_{1} & =\phi_{2}-d_{22} \phi_{1} / d_{12}
\end{aligned}
$$

The term $\dddot{q}_{1}$ can easily be shown to be strictly positive as a consequence of the positive definiteness of the robot inertia matrix and strong inertial coupling. A feedback linearizing controller can therefore be defined for equation (9) according to

$$
\tau=\dddot{q}_{1} v_{1}+\ddot{h}_{1}+\ddot{\phi}_{1}
$$

where $v_{l}$ is an additional outer loop control term that will be used to complete the generation of the swing up control law. The complete system $\Sigma_{1}$, to this point, is given by

$$
\begin{gathered}
d_{12} \ddot{q}_{2}+h_{1}+\phi_{1}=-d_{11} v_{1} \\
\ddot{q}_{1}=v_{1}
\end{gathered}
$$

If $q_{1}^{d}(t)$ is a given reference trajectory for $q_{l}$ we may choose the input term $v_{l}$ as

$$
v_{1}=\dot{q}_{1}^{d}+k_{d}\left(\dot{q}_{1}^{d}-\dot{q}_{1}\right)+k_{p}\left(q_{1}^{d}-q_{1}\right)
$$

where $k_{p}$ and $k_{d}$ are positive gains. With state variables

$$
z_{1}=q_{1}-q_{1}^{d}
$$

$$
\begin{gathered}
z_{2}=\dot{q}_{1}-\dot{q}_{1}^{d} \\
\eta_{1}=q_{2} \\
\eta_{2}=\dot{q}_{2}
\end{gathered}
$$

the closed loop system may be written as

$$
\begin{gathered}
\dot{z}_{1}=z_{2} \\
\dot{z}_{2}=-k_{p} z_{1}-k_{d} z_{2} \\
\dot{\eta}_{1}=\eta_{2} \\
\dot{\eta}_{2}=-\frac{1}{d_{12}}\left(h_{1}+\phi_{1}\right)-\frac{d_{11}}{d_{12}} v_{1}
\end{gathered}
$$

It is interesting to note that the same result can be obtained by choosing an output equation

$$
y=q_{1}-q_{1}^{d}=z_{1}
$$

for the original system (1)-(2), differentiating the output $y$ until the input appears, and then choosing the control input to linearize the resulting equation. The system therefore has relative degree 2 with respect to the output $y$. The manner in which we have arrived at the system $\Sigma_{1}$ has the advantage that the computation and analysis of the resulting zero dynamics is simple.

It is, at first glance, surprising that we can achieve a linear response from the first degree of freedom even though it is not directly actuated but is instead driven only by the coupling forces arising from motion of the second link. The motion of link 2 necessary to achieve this may be complex and precisely defines the zero dynamics of the system. For this reason the analysis of the zero dynamics [11] is crucial to the understanding of the behavior of the complete system. The zero dynamics, with respect to the output $y=z_{l}$ are computed by specifying that the $q_{l}$ identically track the reference trajectory $q_{1}^{d}$. We will analyze the zero dynamics for the case of a constant reference command in the next section.

## Analysis of the Zero Dynamics: The Autonomous Case

If the reference input $q_{1}^{d}$ is a constant, then the system is autonomous and we may write (15)-(18) as

$$
\begin{gathered}
\dot{z}=A z \\
\dot{\eta}=w(z, \eta)
\end{gathered}
$$

with suitable definitions of the matrix $A$ and the function $w(z, \eta)$ (see [18]). We see from the above that the surface $z=0$ in state space defines an invariant manifold for the system. Since $A$ is Hurwitz for positive values of $k_{p}$ and $k_{d}$ this invariant

manifold is globally attractive. The dynamics on this manifold are given by

$$
\dot{\eta}=w(0, \eta)
$$

and are referred to as the "zero dynamics" with respect to the the output $y$ defined above [11]. Since we are interested in the swing up control problem, we consider the case $\dot{q}_{1}^{\mathrm{p}}=\mathrm{s}_{2}$. Substituting $\dot{q}_{1}^{\mathrm{p}}=\mathrm{s}_{2}, \dot{q}_{1}^{\mathrm{p}}=0=\dot{q}_{1}^{\mathrm{p}}$ into the equation (18) and using the original description of the system (1), we arrive at the following expression for the zero dynamics of the system:

$$
\left(m_{1} l_{12}^{p}+m_{2} l_{1} l_{12} \cos \left(q_{2}\right)+I_{2}\right) \ddot{q}_{2}-m_{2} l_{1} l_{12} \sin \left(q_{2}\right) \dot{q}_{2}^{2}-m_{2} l_{12} g \sin \left(q_{2}\right)=0
$$

The system (23), considered as a dynamical system on the cylinder, has two equilibrium points, $p_{1}=(0,0)^{T}$, which is a saddle, and $p_{2}=(\pi, 0)^{T}$, which is a center. A typical phase portrait of this system (23) is shown in Fig. 2.

It follows (locally) that, for initial conditions, $z(0)=z_{0}$, $\eta(0)=\eta_{0}$, the state $z(t)$ converges exponentially to zero, while the state $\eta(t)$ converges to a trajectory of the system (23). The proof of this fact relies on the Center Manifold Theorem [6] and can be found in [11].

It is interesting to note that the expression for the zero dynamics, Equation (23), is independent of the gains $k_{p}$ and $k_{d}$ used in the outer loop control (13). These gains, however, together with the intial conditions, completely determine the particular trajectory of the zero dynamics to which the response of the complete system converges. We will see then that the tuning of these gains is crucial to the achievement of a successful swingup.

Since almost all trajectories of the system (23) are periodic, the typical steady state behavior is for the first link to converge exponentially to $q_{1}=\mathrm{s}_{2}$ and for the second link to oscillate, either about the center point equilibrium $(\pi, 0)$ of (23), or "outside" the homoclinic orbit of the saddle point equilibrium. The strategy for the swing up control is then to determine an appropriate set of gains $k_{p}, k_{d}$ for the outer loop control (13) that swings the second link close to its saddle point equilibrium and then to switch from the above partial feedback linearization controller to a linear, quadratic regulator designed to balance the Acrobot about this equilibrium, whenever the trajectory enters the basin of attraction
![img-2.jpeg](img-2.jpeg)

Fig. 2. Phase portrait of the zero dynamics.
defined by the LQR controller. This will be illustrated in the next section by simulation results.

## Simulation Results

We have simulated the Acrobot in Simnon [7], using the parameters in Table 1. The links are modeled as uniform thin rods and so the moments of inertia are given by the formula $I=\frac{1}{2}\left(2 m l^{2}\right.$. It can easily be checked that the Strong Inertial Coupling condition holds for this set of parameters.

Fig. 3 shows the response of the partial feedback linearization controller with gains $k_{p}=16, k_{d}=8$. The angle $q_{2}$ is plotted modulo $2 \pi$, which is the reason for any apparent jumps in the joint angle during the transient response.

Fig. 4 shows the response of the partial feedback linearization controller for the gains $k_{p}=20$ and $k_{d}=8$. In this case link 2 rotates $360^{\circ}$ in the steady state.

The "tuning problem" is then to choose a set of gains to move the Acrobot as close as possible to the saddle point equilibrium and then switch to a "balancing" controller to capture and balance the Acrobot about this equilibrium. We illustrate this below using a linear, quadratic regulator to balance the Acrobot about the vertical.

## The Balancing Controller

Linearizing the Acrobot dynamics about the vertical equilibrium $q_{1}=\mathrm{s}_{2}, q_{2}=0$, using the parameters in Table 1 results in the controllable linear system

$$
\dot{x}=A x+B u
$$

where the state vector $x=\left(q_{1}-\mathrm{s}_{2}, q_{2}, \dot{q}_{1}, \dot{q}_{2}\right)$, the control input $u=\tau$, and the matrices $A$ and $B$ are given by

Table 1
Parameters of the Simulated Acrobot

| $\mathrm{m}_{1}$ | $\mathrm{~m}_{2}$ | $\mathrm{l}_{1}$ | $\mathrm{l}_{2}$ | $\mathrm{l}_{\mathrm{c} 1}$ | $\mathrm{l}_{\mathrm{c} 2}$ | $\mathrm{I}_{1}$ | $\mathrm{I}_{2}$ | g |
| :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: | :--: |
| 1 | 1 | 1 | 2 | 0.5 | 1 | 0.083 | 0.33 | 9.8 |

![img-3.jpeg](img-3.jpeg)

Fig. 3. Partial feedback linearization response with gains $k_{p}=16$, $k_{d}=8$.

![img-4.jpeg](img-4.jpeg)

Fig. 4. Partial feedback linearization response with gains $k_{p}=20$, $k_{d}=8$.
![img-5.jpeg](img-5.jpeg)

Fig. 5. Swing up motion of the Acrobot using $\Sigma_{1}$.

$$
\begin{gathered}
A=\left[\begin{array}{rrrr}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
12.49 & -12.54 & 0 & 0 \\
-14.49 & 29.36 & 0 & 0
\end{array}\right] \\
B=\left[\begin{array}{c}
0 \\
0 \\
-2.98 \\
5.98
\end{array}\right]
\end{gathered}
$$

Using Matlab, an LQR controller was designed with weighting matrices

$$
Q=\left[\begin{array}{llll}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{array}\right]
$$

and $R=1$, yielding the state feedback controller $u=-K x$, where

$$
K=[-242.52,-96.33,-104.59,-49.05]
$$

The linear control law is switched on whenever the Acrobot reaches the near vertical configuration. Fig. 5 shows a plot of a successful swing up and balance using the partial feedback linearization followed by the linear, quadratic regulator.

## Derivation of the System $\Sigma_{i}$ Collocated Linearization

In this section we derive an alternative swing up control algorithm which can be used in the case that the second link is constrained to rotate less than a full revolution, as for example, with the experimental Acrobot considered in [5]. The alternative algorithm derived here is based on linearizing the system with respect to $q 2$ instead of $q_{1}$. Consider the Equation (2),

$$
d_{21} \ddot{q}_{1}+d_{22} \ddot{q}_{2}+h_{2}+\phi_{2}=\tau
$$

This time we solve for $\ddot{q}_{1}$ from Equation (1) and substitute the resulting expression into (29) to obtain

$$
\dddot{q}_{2} \ddot{q}_{2}+\bar{h}_{2}+\bar{\phi}_{2}=\tau
$$

where the terms $\dddot{q}_{2}, \bar{h}_{2}, \bar{\phi}_{2}$ are given by

$$
\begin{aligned}
\dddot{q}_{2} & =d_{22}-d_{21} d_{12} / d_{11} \\
\bar{h}_{2} & =h_{2}-d_{21} h_{1} / d_{11} \\
\phi_{2} & =\phi_{2}-d_{21} \phi_{1} / d_{11}
\end{aligned}
$$

Note that this requires that the term $d_{11}$ be nonzero over the configuration manifold of the robot. This, however, involves no restrictions on the inertia parameters since $d_{11}$ is always bounded away from zero as a consequence of the uniform positive definiteness of the robot inertia matrix. A feedback linearizing controller can be defined for equation (30) according to

$$
\tau=\dddot{q}_{22} v_{2}+\bar{h}_{2}+\bar{\phi}_{2}
$$

Substituting the control (31) into (30) yields the system $\Sigma_{2}$

$$
\begin{gathered}
d_{11} \ddot{q}_{1}+h_{1}+\phi_{1}=-d_{12} v_{2} \\
\ddot{q}_{2}=v_{2}
\end{gathered}
$$

The input term $v_{2}$ can now be chosen so that $q_{2}$ tracks any given reference trajectory $q_{2}^{\mathrm{d}}$. The important problem now is to choose the reference signal $q_{2}^{\mathrm{d}}$ to execute the swing up maneuver. In [19] an energy pumping strategy was used to solving the swing up control problem. The result in [19] contains an analysis of the resulting zero dynamics for $\Sigma_{2}$ similar to that contained here for $\Sigma_{1}$. We will not repeat the analysis of the zero dynamics in this paper. Instead we will discuss the original energy pumping interpretation of our algorithm that was the original motivation for its derivation.

## Energy Based Swing Up Algorithm

If the second link angle $q 2$ is constrained to lie in an interval $q_{2} \in\{-\beta, \beta\}$ then we choose an $\alpha$ less than $\beta$ and swing the second link as follows: Let the reference $q_{2}^{\mathrm{d}}$ for link 2 be given as

$$
q_{2}^{\mathrm{d}}=\alpha \arctan \left(\dot{q}_{1}\right)
$$

and choose the outer loop control term $v_{2}$ as

$$
v_{2}=k_{p}\left(q_{2}^{\mathrm{d}}-q_{2}\right)-k_{d} \dot{q}_{2}
$$

with $k_{p}$ and $k_{d}$ positive gains. The idea behind this choice of reference position for $q_{2}$ is to "pump energy" into the system by swinging link 2 "in phase" with the motion of link 1 so that energy is transferred from link 2 to link 1 . In this way, the amplitude of link 1 may be increased with each swing.

To see how this might be expected to work, consider the motion of a single link with a force $F$ acting at the end of the link. Assume that the force $F$ is directed perpendicular to the link for simplicity. Then the torque acting at the joint is equal to $l F$ and the equation of motion is

$$
l \ddot{q}_{1}+m g l_{c} \sin \left(q_{1}\right)=l F
$$

The total energy of the system is given by

$$
V=\frac{1}{2} l \dot{q}_{1}^{2}+m g l_{c}\left(1-\cos \left(q_{1}\right)\right)
$$

and the derivative of V along trajectories of the system is given by

$$
\dot{V}=l F \dot{q}_{1}
$$

Therefore, the change in total energy over a time interval $[T-1$, $T]$ is

$$
V(T)-V(T-1)=\int_{T-1}^{T} F(t) \dot{q}_{1}(t) d t
$$

Suppose that the force $F(t)$ is any so-called 1st and 3rd quadrant function of $\dot{q}_{1}$, i.e., suppose that,

$$
F=|F| \operatorname{sgn}\left(\dot{q}_{1}(t)\right)
$$

Then we see from (39) that

$$
V(T)-V(T-1)=\int_{T-1}^{T}|F| \cdot\left|\dot{q}_{1}\right| d t \geq 0
$$

i.e., the change in energy during the time interval $[T-1, T]$ is nonnegative. Our strategy for swinging link 2 rapidly in the direction of motion of $\dot{q}_{1}$ is designed to produce a net force during the time $[T-1, T]$ of each swing with the "correct sign" as above.
![img-6.jpeg](img-6.jpeg)

Fig. 6. Swing up motion of the Acrobot using $\Sigma_{2}$.
![img-7.jpeg](img-7.jpeg)

Fig. 7. Total energy during the swing up motion.
Although the above simplified analysis only approximately describes the true Acrobot, we will see below that the total energy is indeed increased with each swing as we might expect from the above considerations.

Our choice of reference position for $q_{2}$ also has the effect of straightening out the Acrobot at the top of each swing, which facilitates the capturing of the Acrobot at the vertical position. Other choices of reference $q_{2}^{\mathrm{d}}$ are, of course, possible such as $q_{2}^{\mathrm{d}}=\alpha \operatorname{sgn}\left(\dot{q}_{1}\right)$ or $q_{2}^{\mathrm{d}}=\alpha \operatorname{sat}\left(\dot{q}_{1}\right)$. The essential feature is that the reference function be a so-called "first and third quadrant" function of $\dot{q}_{1}$. See [20] for additional details and an analysis of the resulting zero dynamics.

Fig. 6 shows a swing up motion using the reference for $q_{2}$ given by (34). Again the LQR controller is switched on at the top of the swing. Fig. 7 shows a plot of the total energy during the swing up motion.

## Conclusions

In this paper we have discussed two distinctly different swing up control strategies for the Acrobot, both based on the concept of partial feedback linearization. It is quite interesting that the complex swing up motions are realized in the closed-loop as the "natural responses" of autonomous nonlinear differential equations. It is also interesting that, in both cases, unstable behavior of the zero dynamics is exploited to realize the swing up motion.

The general principles discussed in this paper are applicable to a broader class of control problems. For fully actuated (and therefore feedback linearizable) systems, the nonlinear control problem is considered essentially solved once the system is linearized. We have seen in the case of the Acrobot that the second stage (or outer loop) design remains a non-trivial and nonlinear task. Interesting control problems remaining for this class of systems include the robust and adaptive control. We note that the partial feedback linearization approach leads to a system in which the inertia parameters appear nonlinearly. Thus standard adaptive techniques that have been developed for fully actuated rigid robots cannot be applied in a direct adaptive control scheme.

The simulation indicate that the response of the system is very sensitive to the values of the outer loop gains and to the switching times. Thus, the "tuning issues" in these types of problems are important, and, moreover, naturally lend themselves to methods of repetitive learning control. The reader is referred to [22] for an application of machine learning methods to this problem.

Another interesting problem is the further investigation of robust control to the balancing control. The basin of attraction of the LQR controller used here is very small, making the capture and balance phase of the swing up motion difficult. The application of more robust designs in order to increase the basin of attraction of the balancing controller is thus important and would ameliorate the difficulties of tuning the gains in the swing up phase. For example, the work of Bortoff [5] has shown that techniques such as pseudolinearization can greatly enlarge the basin of attraction of balancing controllers for these systems.

## Acknowledgement

The author would like to thank the reviewers for several helpful suggestions to improve the exposition in this paper.

## References

[1] H. Arai and S. Tachi, "Position Control of a Manipulator with Passive Joints Using Dynamic Coupling," IEEE Trans. Robotics and Automation, vol. 7, no. 4, pp. 528-534, Aug. 1991.
[2] H. Arai, K. Tanie, and S. Tachi, "Dynamic Control of a Manipulator with Passive Joints in Operational Space," IEEE Trans. Robotics and Automation, vol. 9, no. 1, pp. 85-93, Feb. 1993.
[3] M. Berkemeier and R.S. Fearing, "Control of a Two-Link Robot to Achieve Sliding and Hopping Gaits," Proc. 1992 IEEE Int. Conf. on Robotics and Automation, Nice, France, pp. 286-291, May 1992.
[4] S.A. Bortoff and M.W. Spong, "Observer-Based Pseudo- Linearization Using Splines: The Rolling Acrobot Example," ASME Winter Annual Meeting, Anaheim, CA, 1992.
[5] S.A. Bortoff, "Pseudolinearization Using Spline Functions With Application to the Acrobot," Ph.D. thesis, Dept. of Electrical and Computer Engineering, University of Illinois at Urbana- Champaign, 1992.
[6] Carr, Applications of Center Manifold Theory, Spring-Verlag, New York, 1981.
[7] H. Elmquist, Simnon-User's Guide. Dept. of Automatic Control, Lund Inst. of Technology, 1975.
[8] S. Dubowsky and E. Papadopoulos, "The Kinematics, Dynamics, and Control of Free-Flying and Free-Floating Space Robotic Systems," IEEE Trans. on Robotics and Automation, vol. 9, no. 5, 1993.
[9] K. Furuta and M. Yamakita, "Swing up Control of Inverted Pendulum," in IECON'91, pp. 2193-2198, 1991.
[10] Y.-L. Gu, and Y. Xu, "A Normal Form Augmentation Approach to Adaptive Control of Space Robot Systems," Proc. IEEE Int. Conf. on Robotics and Automation, pp. 731-737, Atlanta, GA. May 1993.
[11] A. Isidori, Nonlinear Control Systems, second ed., Springer-Verlag, Berlin, 1989.
[12] P.V. Kokotovic, M. Krstic, and I. Kanellakopoulos, "Backstepping to Passivity: Recursive Design of Adaptive Systems," IEEE Conf. on Decision and Control, Tucson, AZ., pp. 3276-3280, 1992.
[13] S. Mori, H. Nishihara, and K. Furuta, "Control of Unstable Mechanical Systems: Control of Pendulum," Int. J. Control, vol. 23, pp. 673-692, 1976.
[14] R.M. Murray and J. Hauser, "A Case Study in Approximate Linearization: The Acrobot Example," Proc. American Control Conference, 1990.
[15] E. Papadopoulos and S. Dubowsky, "Coordinated Manipulator/Spacecraft Motion Control for Space Robotic Systems," Proc. IEEE Intl. Conf. on Robotics and Automation, Sacramento, CA, April, 1991.
[16] E. Papadopoulos and S. Dubowsky, "Dynamic Singularities in FreeFloating Space Manipulators," ASME J. Dynamical Systems, Measurement, and Control, vol. 115, March, 1993.
[17] F. Saito, T. Fukuda, and F. Arai, "Swing and Locomotion Control for Two-Link Brachiation Robot," Proc. 1993 IEEE Int. Conf. on Robotics and Automation, pp. 719-724, Atlanta, GA, 1993.
[18] M.W. Spong, "The Control of Underactuated Mechanical Systems," First International Symposium on Mechatronics, Mexico City, Jan. 1994.
[19] M.W. Spong, "Swing Up Control of the Acrobot," 1994 IEEE Int. Conf. on Robotics and Automation, pp. 2356-2361, San Diego, CA, May, 1994.
[20] M.W. Spong, "Partial Feedback Linearization of Underactuated Mechanical Systems," Proc. IROS'94, pp. 314-321, Munich, Germany, Sept. 1994.
[21] M.W. Spong, "Swing Up Control of the Acrobot Using Partial Feedback Linearization," Proc. of the SYROCO'94, pp. 838-838, Capri, Sept., 1994.
[22] M.W. Spong and G. DeJong, "Standing the Acrobot: An Example of Intelligent Control," Proc. American Control Conference, pp. 2158-2162, Baltimore, MD, June 1994.
[23] M.W. Spong and M. Vidyasagar, Robot Dynamics and Control, John Wiley \& Sons, Inc., New York, NY, 1989.
[24] S. Takashima, "Dynamic Modeling of a Gymnast on a High Bar," IEEE International Workshop on Intelligent Robots and Systems, IROS'90, pp. $955-962$.
[25] S. Takashima, "Control of a Gymnast on a High Bar," IEEE Int'l Workshop on Intelligent Robots and Systems, IROS'91, pp. 1424-1429, Osaka, Japan, Nov. 1991.
[26] M. Wiklund, A. Kristenson, and K.J. Astrom, "A New Strategy for Swinging up an Inverted Pendulum," Proc. IFAC Symposium, Sydney, Australia, 1993.
![img-8.jpeg](img-8.jpeg)

Mark W. Spong was born in Warren, Ohio, on Nov. 5, 1952. He received the B.A. degree (magna cum laude, Phi Beta Kappa) in mathematics and physics from Hiram College in 1975, the M.S. degree in mathematics from New Mexico State University in 1977, and the M.S. and D.Sc. degrees in systems science and mathematics from Washington University in St. Louis in 1979 and 1981, respectively. Since August 1984, Spong has been at the University of Illinois at Urbana-Champaign, where he is currently professor of general engineering, professor of electrical and computer engineering, research professor in the Coordinated Science Laboratory, and director of the Department of General Engineering Robotics and Automation Laboratory, which he founded in January 1987.
# Underactuated Robotics 

## Algorithms for Walking, Running, Swimming, Flying, and Manipulation

Russ Tedrake

(c) Russ Tedrake, 2024

Last modified 2024-11-18.
How to cite these notes, use annotations, and give feedback.
Note: These are working notes used for a course being taught at MIT. They will be updated throughout the Spring 2024 semester. Lecture videos are available on YouTube.

Previous Chapter
Table of contents
Next Chapter

## Chapter 3

Launch in Deepnote

## Acrobots, Cart-Poles, and Quadrotors

A great deal of work in the control of underactuated systems has been done in the context of lowdimensional model systems. These model systems capture the essence of the problem without introducing all of the complexity that is often involved in more real-world examples. In this chapter we will focus on two of the most well-known and well-studied model systems--the Acrobot and the Cart-Pole. After we have developed some tools, we will see that they can be applied directly to other model systems; we will give a number of examples using Quadrotors. All of these systems are trivially underactuated, having fewer actuators than degrees of freedom.

### 3.1 The Acrobot

The Acrobot is a planar two-link robotic arm in the vertical plane (working against gravity), with an actuator at the elbow, but no actuator at the shoulder. It was first described in detail in [1]. The companion system, with an actuator at the shoulder but not at the elbow, is known as the Pendubot[2]. The Acrobot is so named because of its resemblance to a gymnast (or acrobat) on a parallel bar, who controls his/her motion predominantly by effort at the waist (and not effort at the wrist). The most common control task studied for the Acrobot is the swing-up task, in which the system must use the elbow (or waist) torque to move the system into a vertical configuration then balance.

![img-0.jpeg](img-0.jpeg)

Figure 3.1 - The Acrobot. Click here to see a physical Acrobot swing up and balance.
The Acrobot is representative of the primary challenge in underactuated robots. In order to swing up and balance the entire system, the controller must reason about and exploit the state-dependent coupling between the actuated degree of freedom and the unactuated degree of freedom. It is also an important system because, as we will see, it closely resembles one of the simplest models of a walking robot.

# 3.1.1 Equations of motion 

Figure 3.1 illustrates the model parameters used in our analysis. $\theta_{1}$ is the shoulder joint angle, $\theta_{2}$ is the elbow (relative) joint angle, and we will use $\mathbf{q}=\left[\theta_{1}, \theta_{2}\right]^{T}, \mathbf{x}=[\mathbf{q}, \dot{\mathbf{q}}]^{T}$. The zero configuration is with both links pointed directly down. The moments of inertia, $I_{1}, I_{2}$ are taken about the pivots. The task is to stabilize the unstable fixed point $\mathbf{x}=[\pi, 0,0,0]^{T}$.

We will derive the equations of motion for the Acrobot using the method of Lagrange. The locations of the center of mass of each link, $\mathbf{p}_{c 1}, \mathbf{p}_{c 2}$, are given by the kinematics:

$$
\mathbf{p}_{c 1}=\left[\begin{array}{c}
l_{c 1} s_{1} \\
-l_{c 1} c_{1}
\end{array}\right], \quad \mathbf{p}_{c 2}=\left[\begin{array}{c}
l_{1} s_{1}+l_{c 2} s_{1+2} \\
-l_{1} c_{1}-l_{c 2} c_{1+2}
\end{array}\right]
$$

where $s_{1}$ is shorthand for $\sin \left(\theta_{1}\right), c_{1+2}$ is shorthand for $\cos \left(\theta_{1}+\theta_{2}\right)$, etc. The energy is given by:

$$
\begin{gathered}
T=T_{1}+T_{2}, \quad T_{1}=\frac{1}{2} I_{1} \dot{q}_{1}^{2} \\
T_{2}=\frac{1}{2}\left(m_{2} l_{1}^{2}+I_{2}+2 m_{2} l_{1} l_{c 2} c_{2}\right) \dot{q}_{1}^{2}+\frac{1}{2} I_{2} \dot{q}_{2}^{2}+\left(I_{2}+m_{2} l_{1} l_{c 2} c_{2}\right) \dot{q}_{1} \dot{q}_{2} \\
U=-m_{1} g l_{c 1} c_{1}-m_{2} g\left(l_{1} c_{1}+l_{c 2} c_{1+2}\right)
\end{gathered}
$$

Entering these quantities into the Lagrangian yields the equations of motion:

$$
\begin{gathered}
\left(I_{1}+I_{2}+m_{2} l_{1}^{2}+2 m_{2} l_{1} l_{c 2} c_{2}\right) \ddot{q}_{1}+\left(I_{2}+m_{2} l_{1} l_{c 2} c_{2}\right) \ddot{q}_{2}-2 m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{1} \dot{q}_{2} \\
-m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{2}^{2}+m_{1} g l_{c 1} s_{1}+m_{2} g\left(l_{1} s_{1}+l_{c 2} s_{1+2}\right)=0 \\
\left(I_{2}+m_{2} l_{1} l_{c 2} c_{2}\right) \ddot{q}_{1}+I_{2} \ddot{q}_{2}+m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{1}^{2}+m_{2} g l_{c 2} s_{1+2}=\tau
\end{gathered}
$$

In standard, manipulator equation form:

$$
\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}}+\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}=\tau_{g}(\mathbf{q})+\mathbf{B u}
$$

using $\mathbf{q}=\left[\theta_{1}, \theta_{2}\right]^{T}, \mathbf{u}=\tau$ we have:

$$
\begin{gathered}
\mathbf{M}(\mathbf{q})=\left[\begin{array}{cc}
I_{1}+I_{2}+m_{2} l_{1}^{2}+2 m_{2} l_{1} l_{c 2} c_{2} & I_{2}+m_{2} l_{1} l_{c 2} c_{2} \\
I_{2}+m_{2} l_{1} l_{c 2} c_{2} & I_{2}
\end{array}\right] \\
\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})=\left[\begin{array}{cc}
-2 m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{2} & -m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{2} \\
m_{2} l_{1} l_{c 2} s_{2} \dot{q}_{1} & 0
\end{array}\right] \\
\tau_{g}(\mathbf{q})=\left[\begin{array}{cc}
-m_{1} g l_{c 1} s_{1}-m_{2} g\left(l_{1} s_{1}+l_{c 2} s_{1+2}\right) \\
-m_{2} g l_{c 2} s_{1+2}
\end{array}\right], \quad \mathbf{B}=\left[\begin{array}{l}
0 \\
1
\end{array}\right]
\end{gathered}
$$

# Example 3.1 (Dynamics of the Acrobot) 

You can experiment with the Acrobot dynamics in Drake using, e.g.

## Launch in Deepnote

### 3.2 The Cart-Pole System

The other model system that we will investigate here is the cart-pole system, in which the task is to balance a simple pendulum around its unstable equilibrium, using only horizontal forces on the cart. Balancing the cart-pole system is used in many introductory courses in control, including 6.003 at MIT, because it can be accomplished with simple linear control (e.g. pole placement) techniques. In this chapter we will consider the full swing-up and balance control problem, which requires a full nonlinear control treatment.
![img-1.jpeg](img-1.jpeg)

Figure 3.2 - The Cart-Pole system. Click here to see a real robot.
The figure shows our parameterization of the system. $x$ is the horizontal position of the cart, $\theta$ is the counter-clockwise angle of the pendulum (zero is hanging straight down). We will use $\mathbf{q}=[x, \theta]^{T}$, and $\mathbf{x}=[\mathbf{q}, \dot{\mathbf{q}}]^{T}$. The task is to stabilize the unstable fixed point at $\mathbf{x}=[0, \pi, 0,0]^{T}$.

### 3.2.1 Equations of motion

The kinematics of the system are given by

$$
\mathbf{x}_{1}=\left[\begin{array}{l}
x \\
0
\end{array}\right], \quad \mathbf{x}_{2}=\left[\begin{array}{c}
x+l \sin \theta \\
-l \cos \theta
\end{array}\right]
$$

The energy is given by

$$
\begin{aligned}
& T=\frac{1}{2}\left(m_{c}+m_{p}\right) \dot{x}^{2}+m_{p} \dot{x} \dot{\theta} l \cos \theta+\frac{1}{2} m_{p} l^{2} \dot{\theta}^{2} \\
& U=-m_{p} g l \cos \theta
\end{aligned}
$$

The Lagrangian yields the equations of motion:

$$
\begin{gathered}
\left(m_{c}+m_{p}\right) \ddot{x}+m_{p} l \ddot{\theta} \cos \theta-m_{p} l \dot{\theta}^{2} \sin \theta=f_{x} \\
m_{p} l \ddot{x} \cos \theta+m_{p} l^{2} \ddot{\theta}+m_{p} g l \sin \theta=0
\end{gathered}
$$

In standard, manipulator equation form:

$$
\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}}+\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}=\tau_{g}(\mathbf{q})+\mathbf{B u}
$$

using $\mathbf{q}=[x, \theta]^{T}, \mathbf{u}=f_{x}$, we have:

$$
\begin{aligned}
\mathbf{M}(\mathbf{q})=\left[\begin{array}{cc}
m_{c}+m_{p} & m_{p} l \cos \theta \\
m_{p} l \cos \theta & m_{p} l^{2}
\end{array}\right], \quad \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})= & {\left[\begin{array}{cc}
0 & -m_{p} l \dot{\theta} \sin \theta \\
0 & 0
\end{array}\right], } \\
\tau_{g}(\mathbf{q})=\left[\begin{array}{c}
0 \\
-m_{p} g l \sin \theta
\end{array}\right], \quad \mathbf{B}= & {\left[\begin{array}{l}
1 \\
0
\end{array}\right] }
\end{aligned}
$$

In this case, it is particularly easy to solve directly for the accelerations:

$$
\begin{aligned}
& \ddot{x}=\frac{1}{m_{c}+m_{p} \sin ^{2} \theta}\left[f_{x}+m_{p} \sin \theta\left(l \dot{\theta}^{2}+g \cos \theta\right)\right] \\
& \ddot{\theta}=\frac{1}{l\left(m_{c}+m_{p} \sin ^{2} \theta\right)}\left[-f_{x} \cos \theta-m_{p} l \dot{\theta}^{2} \cos \theta \sin \theta-\left(m_{c}+m_{p}\right) g \sin \theta\right]
\end{aligned}
$$

In some of the analysis that follows, we will study the form of the equations of motion, ignoring the details, by arbitrarily setting all constants to 1 :

$$
\begin{gathered}
2 \ddot{x}+\ddot{\theta} \cos \theta-\dot{\theta}^{2} \sin \theta=f_{x} \\
\ddot{x} \cos \theta+\ddot{\theta}+\sin \theta=0
\end{gathered}
$$

# Example 3.2 (Dynamics of the Cart-Pole System) 

You can experiment with the Cart-Pole dynamics in Drake using, e.g.

## Launch in Deepnote

### 3.3 QuADROTORS

Quadrotors have become immensely popular over the last few years -- advances in outrunner motors from the hobby community made them powerful, light-weight, and inexpensive! They are strong enough to carry an interesting payload (e.g. of sensors for mapping / photographing the environment), but dynamic enough to move relatively quickly. The most interesting dynamics start showing up at higher speeds, when the propellers start achieving lift from the airflow across them due to horizontal motion, or when they are close enough to the ground to experience significant ground-effect, but we won't try to capture those effects (yet) here.

When the quadrotor revolution started to happen, I predicted that it would be followed quickly by a realization that fixed-wing vehicles are better for most applications. Propellers are almost optimally efficient for producing thrust -- making quadrotors very efficient for hovering -- but to be efficient in forward flight you probably want to have an airfoil. Wings are a really good idea! But I was wrong -- quadrotors have completely dominated fixed-wings for commercial UAVs. Perhaps it's only because they are easier to control? I suspect that as the field matures and achieving core functionality is not the primary obstacle, then people will eventually start worrying again about efficiency (a bit like we think about efficiency for automobiles).

# 3.3.1 The Planar Quadrotor 

We can get started with an extremely simple model of a quadrotor that is restricted to live in the plane. In that case, it actually only needs two propellers, but calling it a "birotor" doesn't have the same ring to it. The equations of motion are almost trivial, since it is only a single rigid body, and certainly fit into our standard manipulator equations:

$$
\begin{gathered}
m \ddot{x}=-\left(u_{1}+u_{2}\right) \sin \theta \\
m \ddot{y}=\left(u_{1}+u_{2}\right) \cos \theta-m g \\
I \ddot{\theta}=r\left(u_{1}-u_{2}\right)
\end{gathered}
$$

![img-2.jpeg](img-2.jpeg)

Figure 3.3 - The Planar Quadrotor System (which we also refer to in the code as "Quadrotor2D", to keep it next to "Quadrotor" in the directory listing). The model parameters are mass, $m$, moment of inertia, $I$, and the distance from the center to the base of the propeller, $r$.

### 3.3.2 The Full 3D Quadrotor

The dynamics of the 3D Quadrotor follow similarly. The only complexity comes from handling the 3D rotations. The code implementing the dynamics of the QuadrotorPlant example in Drake is almost as readable as a LaTeX derivation. The most interesting feature of this model that we include the moment produced by the rotating propellers. Interestingly, without these moments, the system linearized about the hovering configuration is actually not controllable.

The QuadrotorPlant example implements the simple model. But often you may want to add extra features to the quadrotor model. For instance, you might wish to add collision dynamics to land on the ground or to bounce off obstacles, or you might want to add a pendulum to the quadrotor to balance, or a suspended payload hanging off the bottom. In these cases, rather than implement the equations by hand, it is much more convenient to use Drake's main physics engine (in MultibodyPlant). We just need to manually wire the Propeller forces into the Diagram (because Propeller is not a concept supported in URDF nor SDF just yet). I've implemented both versions side-by-side in the notebook.

# Launch in Deepnote 

Note that by default, the orientation of each floating body in MultibodyPlant is represented by a unit quaternion. If you were to try to linearize the model using the quaternion representation of state, without accounting for unit norm constraint, the linearization of this model is also uncontrollable. To avoid this for now, I have manually added a "roll-pitch-yaw" floating base to the model. This keeps linearization simple but does introduce a singularity at "gimble lock". I've tried to mostly avoid dealing with 3D rotations throughout these notes, but my notes on manipulation cover 3D rotations in a bit more detail.

### 3.4 BALANCING

For both the Acrobot and the Cart-Pole systems, we will begin by designing a linear controller which can balance the system when it begins in the vicinity of the unstable fixed point. To accomplish this, we will linearize the nonlinear equations about the fixed point, examine the controllability of this linear system, then using linear quadratic regulator (LOR) theory to design our feedback controller.

### 3.4.1 Linearizing the manipulator equations

Although the equations of motion of both of these model systems are relatively tractable, the forward dynamics still involve quite a few nonlinear terms that must be considered in any linearization. Let's consider the general problem of linearizing a system described by the manipulator equations.

We can perform linearization around a fixed point, $\left(\mathbf{x}^{*}, \mathbf{u}^{*}\right)$, using a Taylor expansion:

$$
\dot{\mathbf{x}}=\mathbf{f}(\mathbf{x}, \mathbf{u}) \approx \mathbf{f}\left(\mathbf{x}^{*}, \mathbf{u}^{*}\right)+\left[\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\right]_{\mathbf{x}=\mathbf{x}^{*}, \mathbf{u}=\mathbf{u}^{*}}\left(\mathbf{x}-\mathbf{x}^{*}\right)+\left[\frac{\partial \mathbf{f}}{\partial \mathbf{u}}\right]_{\mathbf{x}=\mathbf{x}^{*}, \mathbf{u}=\mathbf{u}^{*}}\left(\mathbf{u}-\mathbf{u}^{*}\right)
$$

Let us consider the specific problem of linearizing the manipulator equations around a (stable or unstable) fixed point. In this case, $\mathbf{f}\left(\mathbf{x}^{*}, \mathbf{u}^{*}\right)$ is zero, and we are left with the standard linear statespace form:

$$
\begin{aligned}
\dot{\mathbf{x}} & =\left[\begin{array}{c}
\dot{\mathbf{q}} \\
\mathbf{M}^{-1}(\mathbf{q})\left[\tau_{g}(\mathbf{q})+\mathbf{B}(\mathbf{q}) \mathbf{u}-\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}\right]
\end{array}\right] \\
& \approx \mathbf{A}_{\text {lin }}\left(\mathbf{x}-\mathbf{x}^{*}\right)+\mathbf{B}_{\text {lin }}\left(\mathbf{u}-\mathbf{u}^{*}\right)
\end{aligned}
$$

where $\mathbf{A}_{\text {lin }}$, and $\mathbf{B}_{\text {lin }}$ are constant matrices. Let us define $\overline{\mathbf{x}}=\mathbf{x}-\mathbf{x}^{*}, \overline{\mathbf{u}}=\mathbf{u}-\mathbf{u}^{*}$, and write

$$
\dot{\overline{\mathbf{x}}}=\mathbf{A}_{l i n} \overline{\mathbf{x}}+\mathbf{B}_{l i n} \overline{\mathbf{u}}
$$

Evaluation of the Taylor expansion around a fixed point yields the following, very simple equations, given in block form by:

$$
\begin{aligned}
& \mathbf{A}_{l i n}=\left[\begin{array}{cc}
\mathbf{0} & \mathbf{I} \\
\mathbf{M}^{-1} \frac{\partial \tau_{g}}{\partial \mathbf{q}}+\sum_{j} \mathbf{M}^{-1} \frac{\partial \mathbf{B}_{j}}{\partial \mathbf{q}} u_{j} & \mathbf{0}
\end{array}\right]_{\mathbf{x}=\mathbf{x}^{*}, \mathbf{u}=\mathbf{u}^{*}} \\
& \mathbf{B}_{l i n}=\left[\begin{array}{c}
\mathbf{0} \\
\mathbf{M}^{-1} \mathbf{B}
\end{array}\right]_{\mathbf{x}=\mathbf{x}^{*}, \mathbf{u}=\mathbf{u}^{*}}
\end{aligned}
$$

where $\mathbf{B}_{j}$ is the $j$ th column of $\mathbf{B}$. Note that the term involving $\frac{\partial \mathbf{M}^{-1}}{\partial q_{i}}$ disappears because $\tau_{g}+\mathbf{B u}-\mathbf{C} \dot{\mathbf{q}}$ must be zero at the fixed point. All of the $\mathbf{C} \dot{\mathbf{q}}$ derivatives drop out, too, because $\dot{\mathbf{q}}^{*}=0$, and any terms with $\mathbf{C}$ drop out as well, since centripetal and centrifugal forces are zero when velocity is zero. In many cases, including both the Acrobot and Cart-Pole systems (but not the Quadrotors), the matrix $\mathbf{B}(\mathbf{q})$ is a constant, so the $\frac{\partial \mathbf{B}}{\partial \mathbf{q}}$ terms also drop out.

# Example 3.3 (Linearization of the Acrobot) 

Linearizing around the (unstable) upright point, we have:

$$
\left[\begin{array}{c}
\frac{\partial \tau_{\mathbf{g}}}{\partial \mathbf{q}}
\end{array}\right]_{\mathbf{x}=\mathbf{x}^{*}}=\left[\begin{array}{cc}
g\left(m_{1} l_{c 1}+m_{2} l_{1}+m_{2} l_{c 2}\right) & m_{2} g l_{c 2} \\
m_{2} g l_{c 2} & m_{2} g l_{c 2}
\end{array}\right]
$$

The linear dynamics follow directly from these equations and the manipulator form of the Acrobot equations.

## Example 3.4 (Linearization of the Cart-Pole System)

Linearizing around the unstable fixed point in this system, we have:

$$
\left[\begin{array}{c}
\frac{\partial \tau_{g}}{\partial \mathbf{q}}
\end{array}\right]_{\mathbf{x}=\mathbf{x}^{*}}=\left[\begin{array}{cc}
0 & 0 \\
0 & m_{p} g l
\end{array}\right]
$$

Again, the linear dynamics follow simply.

Studying the properties of the linearized system can tell us some things about the (local) properties of the nonlinear system. For instance, having a strictly stable linearization implies local exponential stability of the nonlinear system [3] (Theorem 4.15). It's worth noting that having an unstable linearization also implies that the system is locally unstable, but if the linearization is marginally stable then one cannot conclude whether the nonlinear system is asymptotically stable, stable i.s.L., or unstable (only that it is not exponentially stable)[4].

### 3.4.2 Controllability of linear systems

Definition 3.1 (Controllability) A control system of the form

$$
\dot{\mathbf{x}}=\mathbf{f}(\mathbf{x}, \mathbf{u})
$$

is called controllable if it is possible to construct an unconstrained input signal, $\mathbf{u}(t)$, $t \in\left[0, t_{f}\right]$, which will move the system from any initial state to any final state in a finite interval of time, $0<t<t_{f}[5]$.

For the linear system

$$
\dot{\mathbf{x}}=\mathbf{A x}+\mathbf{B u}
$$

we can integrate this linear system in closed form, so it is possible to derive the exact conditions of controllability. In particular, for linear systems it is sufficient to demonstrate that there exists a control input which drives any initial condition to the origin.

# The special case of non-repeated eigenvalues 

Let us first examine a special case, which falls short as a general tool but may be more useful for understanding the intuition of controllability. Let's perform an eigenvalue analysis of the system matrix $\mathbf{A}$, so that:

$$
\mathbf{A v}_{i}=\lambda_{i} \mathbf{v}_{i}
$$

where $\lambda_{i}$ is the $i$ th eigenvalue, and $\mathbf{v}_{i}$ is the corresponding (right) eigenvector. There will be $n$ eigenvalues for the $n \times n$ matrix $\mathbf{A}$. Collecting the (column) eigenvectors into the matrix $\mathbf{V}$ and the eigenvalues into a diagonal matrix $\boldsymbol{\Lambda}$, we have

$$
\mathbf{A V}=\mathbf{V} \boldsymbol{\Lambda}
$$

Here comes our primary assumption: let us assume that each of these $n$ eigenvalues takes on a distinct value (no repeats). With this assumption, it can be shown that the eigenvectors $\mathbf{v}_{i}$ form a linearly independent basis set, and therefore $\mathbf{V}^{-1}$ is well-defined.

We can continue our eigenmodal analysis of the linear system by defining the modal coordinates, $\mathbf{r}$, with:

$$
\mathbf{x}=\mathbf{V r}, \quad \text { or } \quad \mathbf{r}=\mathbf{V}^{-1} \mathbf{x}
$$

In modal coordinates, the dynamics of the linear system are given by

$$
\dot{\mathbf{r}}=\mathbf{V}^{-1} \mathbf{A} \mathbf{V} \mathbf{r}+\mathbf{V}^{-1} \mathbf{B} \mathbf{u}=\boldsymbol{\Lambda} \mathbf{r}+\mathbf{V}^{-1} \mathbf{B} \mathbf{u}
$$

This illustrates the power of modal analysis; in modal coordinates, the dynamics diagonalize yielding:

$$
\dot{r}_{i}=\lambda_{i} r_{i}+\sum_{j} \beta_{i j} u_{j}, \quad \beta=\mathbf{V}^{-1} \mathbf{B}
$$

Now the concept of controllability becomes clear. Input $j$ can influence the dynamics in modal coordinate $i$ if and only if $\beta_{i j} \neq 0$. In the special case of non-repeated eigenvalues, having control over each individual eigenmode is sufficient to (in finite time) regulate all of the eigenmodes[5]. Therefore, we say that the system is controllable if and only if

$$
\forall i, \exists j \text { such that } \beta_{i j} \neq 0
$$

## A general solution

- Included only for completeness. Click to expand the details.


## Controllability vs. underactuated

Analysis of the controllability of both the Acrobot and Cart-Pole systems reveals that the linearized dynamics about the upright are, in fact, controllable. This implies that the linearized system, if started away from the zero state, can be returned to the zero state in finite time. This is potentially surprising - after all the systems are underactuated. For example, it is interesting and surprising that the Acrobot can balance itself in the upright position without having a shoulder motor.

The controllability of these model systems demonstrates an extremely important, point: An underactuated system is not necessarily an uncontrollable system. Underactuated systems cannot follow arbitrary trajectories, but that does not imply that they cannot arrive at arbitrary points in state space. However, the trajectory required to place the system into a particular state may be arbitrarily complex.

The controllability analysis presented here is for linear time-invariant (LTI) systems. A comparable analysis exists for linear time-varying (LTV) systems. We will even see extensions to nonlinear systems; although it will often be referred to by the synonym of "reachability" analysis.

# Stabilizability of a linear system 

Closely related to controllability is the notion of stabilizability. For linear systems, stabilizability is a strictly weaker condition than controllability (a system can be stabilizable but not controllable, but not the other way around).

Definition 3.2 (Stabilizability of a linear system) A control system of the form

$$
\dot{\mathbf{x}}=\mathbf{A x}+\mathbf{B u}
$$

is called stabilizable if it is possible to construct an unconstrained input signal, $\mathbf{u}(t)$, $t \in[0, \infty]$, which results in

$$
\lim _{t \rightarrow \infty} \mathbf{x}(t)=0
$$

Controllability requires that we arrive at the origin in a finite time, stabilizability allows for asympotitic convergence. Essentially, a system can still be stabilizable if the uncontrollable subspace is naturally stable.

Interestingly, for nonlinear systems the relationship between stabilizability and controllability is much more subtle. In a famous (sometimes misquoted) result from Roger Brockett states that nonlinear controllability does not necessarily imply stabilizability by differentiable control policies[8].

### 3.4.3 LQR feedback

Controllability and stabilizability tell us that a trajectory to the fixed point exists, but does not tell us which one we should take or what control inputs cause it to occur. Why not? There are potentially infinitely many solutions. We have to pick one.

The tools for controller design in linear systems are very advanced. In particular, as we describe in the linear optimal control chapter, one can easily design an optimal feedback controller for a regulation task like balancing, so long as we are willing to linearize the system around the operating point and define optimality in terms of a quadratic cost function:

$$
J\left(\mathbf{x}_{0}\right)=\int_{0}^{\infty}\left[\mathbf{x}^{T}(t) \mathbf{Q} \mathbf{x}(t)+\mathbf{u}^{T}(t) \mathbf{R} \mathbf{u}(t)\right] d t, \quad \mathbf{x}(0)=\mathbf{x}_{0}, \mathbf{Q}=\mathbf{Q}^{T}>0, \mathbf{R}=\mathbf{R}^{T}>0
$$

The linear feedback matrix $\mathbf{K}$ used as

$$
\mathbf{u}(t)=-\mathbf{K} \mathbf{x}(t)
$$

is the so-called optimal linear quadratic regulator (LQR). Even without understanding the detailed derivation, we can quickly become practitioners of LQR. DraKe provides a function,

$$
\mathrm{K}=\text { LinearQuadraticRegulator(A, B, Q, R) }
$$

for linear systems. It also provides a version
controller = LinearQuadraticRegulator(system, context, Q, R)

that will linearize the system for you around an equilibrium and return the controller (in the original coordinates). Therefore, to use LQR, one simply needs to define the symmetric positive-definite cost matrices, $\mathbf{Q}$ and $\mathbf{R}$. In their most common form, $\mathbf{Q}$ and $\mathbf{R}$ are positive diagonal matrices, where the entries $Q_{i i}$ penalize the relative errors in state variable $x_{i}$ compared to the other state variables, and the entries $R_{i i}$ penalize actions in $u_{i}$.

Take a moment to appreciate this. If the linearized system is stabilizable, then for any (positive semi-definite) $\mathbf{Q}$ and (positive definite) $\mathbf{R}$ matrices, LQR will give us a stabilizing controller. If the system is not stabilizable, then LQR will tell us that no linear controller exists. Pretty amazing!

# Example 3.5 (LQR for the Acrobot and Cart-Pole) 

Take a minute to play around with the LQR controller for the Acrobot and the Cart-Pole

## Launch in Deepnote

## Launch in Deepnote

Make sure that you take a minute to look at the code which runs during these examples. Can you set the $\mathbf{Q}$ and $\mathbf{R}$ matrices differently, to improve the performance?

Simulation of the closed-loop response with LQR feedback shows that the task is indeed completed - and in an impressive manner. Oftentimes the state of the system has to move violently away from the origin in order to ultimately reach the origin. Further inspection reveals the (linearized) closedloop dynamics are in fact non-minimum phase (acrobot has 3 right-half zeros, cart-pole has 1 ).

## Example 3.6 (LQR for Quadrotors)

LQR works essentially out of the box for Quadrotors, if linearized around a nominal fixed point (where the non-zero thrust from the propellers is balancing gravity).

## Launch in Deepnote

## Launch in Deepnote

or Click here for the animation.

Note that LQR, although it is optimal for the linearized system, is not necessarily the best linear control solution for maximizing basin of attraction of the fixed-point. The theory of robust control(e.g., [9]), which explicitly takes into account the differences between the linearized model and the nonlinear model, will produce controllers which outperform our LQR solution in this regard.

### 3.5 Partial feedback linearization

In the introductory chapters, we made the point that the underactuated systems are not feedback equivalent to $\ddot{q}=\mathbf{u}$. Although we cannot always simplify the full dynamics of the system, it is still

possible to linearize a portion of the system dynamics. The technique is called partial feedback linearization.

Consider the cart-pole example. The dynamics of the cart are affected by the motions of the pendulum. If we know the model, then it seems quite reasonable to think that we could create a feedback controller which would push the cart in exactly the way necessary to counter-act the dynamic contributions from the pendulum - thereby linearizing the cart dynamics. What we will see, which is potentially more surprising, is that we can also use a feedback controller for the cart to feedback linearize the dynamics of the passive pendulum joint.

We'll use the term collocated partial feedback linearization to describe a controller which linearizes the dynamics of the actuated joints. What's more surprising is that it is often possible to achieve non-collocated partial feedback linearization - a controller which linearizes the dynamics of the unactuated joints. The treatment presented here follows from [10].

# 3.5.1 PFL for the Cart-Pole System 

## Collocated

Starting from the equations 18 and 19, we have

$$
\begin{gathered}
\ddot{\theta}=-\ddot{x} c-s \\
\ddot{x}\left(2-c^{2}\right)-s c-\dot{\theta}^{2} s=f_{x}
\end{gathered}
$$

Therefore, applying the feedback control

$$
f_{x}=\left(2-c^{2}\right) \ddot{x}^{d}-s c-\dot{\theta}^{2} s
$$

results in

$$
\begin{aligned}
& \ddot{x}=\ddot{x}^{d} \\
& \ddot{\theta}=-\ddot{x}^{d} c-s
\end{aligned}
$$

which are valid globally.
Look carefully at the resulting equations. Of course, it says that we can impose whatever accelerations we like on the cart. But even the resulting equations of the pole happen to take a nice form here: they have been reduced to the equations of the simple pendulum (without a cart), where the torque input is now given instead by $\ddot{x} c$. It's as if we have a simple pendulum with torque control, except our command is modulated by a $\cos \theta$ term, and this $\cos \theta$ term is fundamental -- it's true that our control authority goes to zero when the pole is horizontal, because no amount of force on the cart in that configuration can act like a torque on the pole.

## Non-collocated

Starting again from equations 18 and 19, we have

$$
\begin{gathered}
\ddot{x}=-\frac{\ddot{\theta}+s}{c} \\
\ddot{\theta}\left(c-\frac{2}{c}\right)-2 \tan \theta-\dot{\theta}^{2} s=f_{x}
\end{gathered}
$$

Applying the feedback control

$$
f_{x}=\left(c-\frac{2}{c}\right) \ddot{\theta}^{d}-2 \tan \theta-\dot{\theta}^{2} s
$$

results in

$$
\begin{aligned}
& \ddot{\theta}=\ddot{\theta}^{d} \\
& \ddot{x}=-\frac{1}{c} \ddot{\theta}^{d}-\tan \theta
\end{aligned}
$$

Note that this expression is only valid when $\cos \theta \neq 0$. Once again, we know that the force cannot create a torque when the pole is perfectly horizontal. In fact, the controller we have written will "blow-up" -- requesting infinite force at $\cos \theta=0$; so make sure you saturate the command before you implement it on hardware (or even in simulation). Although it may come as a small consolation, at least we have that $\left(c-\frac{2}{c}\right)$ never goes to zero; in fact you can check for yourself that $\left|c-\frac{2}{c}\right| \geq 1$.

# 3.5.2 General form 

For systems that are trivially underactuated (torques on some joints, no torques on other joints), we can, without loss of generality, reorganize the joint coordinates in any underactuated system described by the manipulator equations into the form:

$$
\begin{aligned}
& \mathbf{M}_{11} \ddot{\mathbf{q}}_{1}+\mathbf{M}_{12} \ddot{\mathbf{q}}_{2}=\tau_{1} \\
& \mathbf{M}_{21} \ddot{\mathbf{q}}_{1}+\mathbf{M}_{22} \ddot{\mathbf{q}}_{2}=\tau_{2}+\mathbf{u}
\end{aligned}
$$

with $\mathbf{q} \in \mathbb{R}^{n}, \mathbf{q}_{1} \in \mathbb{R}^{l}, \mathbf{q}_{2} \in \mathbb{R}^{m}, l=n-m . \mathbf{q}_{1}$ represents all of the passive joints, and $\mathbf{q}_{2}$ represents all of the actuated joints, and the $\tau=\tau_{g}-\mathbf{C} \dot{\mathbf{q}}$ terms capture all of the Coriolis and gravitational terms, and

$$
\mathbf{M}(\mathbf{q})=\left[\begin{array}{ll}
\mathbf{M}_{11} & \mathbf{M}_{12} \\
\mathbf{M}_{21} & \mathbf{M}_{22}
\end{array}\right]
$$

Fortunately, because $\mathbf{M}$ is uniformly (e.g. for all $\mathbf{q}$ ) positive definite, $\mathbf{M}_{11}$ and $\mathbf{M}_{22}$ are also positive definite, by the Schur complement condition for positive definiteness.

## Collocated linearization

Performing the same substitutions into the full manipulator equations, we get:

$$
\begin{gathered}
\ddot{\mathbf{q}}_{1}=\mathbf{M}_{11}^{-1}\left[\tau_{1}-\mathbf{M}_{12} \ddot{\mathbf{q}}_{2}\right] \\
\left(\mathbf{M}_{22}-\mathbf{M}_{21} \mathbf{M}_{11}^{-1} \mathbf{M}_{12}\right) \ddot{\mathbf{q}}_{2}-\tau_{2}+\mathbf{M}_{21} \mathbf{M}_{11}^{-1} \tau_{1}=\mathbf{u}
\end{gathered}
$$

It can be easily shown that the matrix $\left(\mathbf{M}_{22}-\mathbf{M}_{21} \mathbf{M}_{11}^{-1} \mathbf{M}_{12}\right)$ is invertible[10]; we can see from inspection that it is symmetric. PFL follows naturally, and is valid globally.

## Non-collocated linearization

$$
\begin{gathered}
\ddot{\mathbf{q}}_{2}=\mathbf{M}_{12}^{+}\left[\tau_{1}-\mathbf{M}_{11} \ddot{\mathbf{q}}_{1}\right] \\
\left(\mathbf{M}_{21}-\mathbf{M}_{22} \mathbf{M}_{12}^{+} \mathbf{M}_{11}\right) \ddot{\mathbf{q}}_{1}-\tau_{2}+\mathbf{M}_{22} \mathbf{M}_{12}^{+} \tau_{1}=\mathbf{u}
\end{gathered}
$$

where $\mathbf{M}_{12}^{+}$is a Moore-Penrose pseudo-inverse. This inverse provides a unique solution when the rank of $\mathbf{M}_{12}$ equals $l$, the number of passive degrees of freedom in the system (it cannot be more, since the matrix only has $l$ rows). The rank condition in this context is sometimes called the property of "Strong Inertial Coupling". It is state dependent; in the cart-pole example above

$\mathbf{M}_{12}=\cos \theta$ and drops rank exactly when $\cos \theta=0$. A system has Global Strong Inertial Coupling if it exhibits Strong Inertial Coupling in every state.
Task-space partial feedback linearization
In general, we can define some combination of active and passive joints that we would like to control. This combination is sometimes called a "task space". Consider an output function of the form,

$$
\mathbf{y}=\mathbf{h}(\mathbf{q})
$$

with $\mathbf{y} \in \mathbb{R}^{p}$, which defines the task space. Define $\mathbf{H}_{1}=\frac{\partial \mathbf{h}}{\partial \mathbf{q}_{1}}, \mathbf{H}_{2}=\frac{\partial \mathbf{h}}{\partial \mathbf{q}_{2}}, \mathbf{H}=\left[\mathbf{H}_{1}, \mathbf{H}_{2}\right]$.

# Theorem 3.1 - Task Space PFL 

If the actuated joints are commanded so that

$$
\ddot{\mathbf{q}}_{2}=\ddot{\mathbf{H}}^{+}\left[\ddot{\mathbf{y}}^{d}-\dot{\mathbf{H}} \dot{\mathbf{q}}-\mathbf{H}_{1} \mathbf{M}_{11}^{-1} \tau_{1}\right]
$$

where $\ddot{\mathbf{H}}=\mathbf{H}_{2}-\mathbf{H}_{1} \mathbf{M}_{11}^{-1} \mathbf{M}_{12}$. and $\ddot{\mathbf{H}}^{+}$is the right Moore-Penrose pseudo-inverse,

$$
\ddot{\mathbf{H}}^{+}=\ddot{\mathbf{H}}^{T}\left(\ddot{\mathbf{H}} \ddot{\mathbf{H}}^{T}\right)^{-1}
$$

then we have

$$
\ddot{\mathbf{y}}=\ddot{\mathbf{y}}^{d}
$$

subject to

$$
\operatorname{rank}(\ddot{\mathbf{H}})=p
$$

Proof Sketch. Differentiating the output function we have

$$
\begin{gathered}
\dot{\mathbf{y}}=\mathbf{H} \dot{\mathbf{q}} \\
\ddot{\mathbf{y}}=\dot{\mathbf{H}} \dot{\mathbf{q}}+\mathbf{H}_{1} \ddot{\mathbf{q}}_{1}+\mathbf{H}_{2} \ddot{\mathbf{q}}_{2}
\end{gathered}
$$

Solving 32 for the dynamics of the unactuated joints we have:

$$
\ddot{\mathbf{q}}_{1}=\mathbf{M}_{11}^{-1}\left(\tau_{1}-\mathbf{M}_{12} \ddot{\mathbf{q}}_{2}\right)
$$

Substituting, we have

$$
\begin{aligned}
\ddot{\mathbf{y}} & =\dot{\mathbf{H}} \dot{\mathbf{q}}+\mathbf{H}_{1} \mathbf{M}_{11}^{-1}\left(\tau_{1}-\mathbf{M}_{12} \ddot{\mathbf{q}}_{2}\right)+\mathbf{H}_{2} \ddot{\mathbf{q}}_{2} \\
& =\dot{\mathbf{H}} \dot{\mathbf{q}}+\ddot{\mathbf{H}} \ddot{\mathbf{q}}_{2}+\mathbf{H}_{1} \mathbf{M}_{11}^{-1} \tau_{1} \\
& =\ddot{\mathbf{y}}^{d}
\end{aligned}
$$

Note that the last line required the rank condition (40) on $\ddot{\mathbf{H}}$ to ensure that the rows of $\ddot{\mathbf{H}}$ are linearly independent, allowing $\ddot{\mathbf{H}} \ddot{\mathbf{H}}^{+}=\mathbf{I}$.

In order to execute a task space trajectory one could command

$$
\ddot{\mathbf{y}}^{d}=\mathbf{K}_{d}\left(\dot{\mathbf{y}}^{d}-\dot{\mathbf{y}}\right)+\mathbf{K}_{p}\left(\mathbf{y}^{d}-\mathbf{y}\right)
$$

Assuming the internal dynamics are stable, this yields converging error dynamics when $\mathbf{K}_{p}, \mathbf{K}_{d}>0[4]$, which implies $y(t) \rightarrow y^{d}(t)$. For a position control robot, the acceleration command of (38) suffices. Alternatively, a torque command follows by substituting (38) and (41) into (33).

# Example 3.7 (End-point trajectory following with the Cart-Pole system) 

Consider the task of trying to track a desired kinematic trajectory with the endpoint of pendulum in the cart-pole system. With one actuator and kinematic constraints, we might be hard-pressed to track a trajectory in both the horizontal and vertical coordinates. But we can at least try to track a trajectory in the vertical position of the end-effector.

Using the task-space PFL derivation, we have:

$$
\begin{gathered}
y=h(\mathbf{q})=-l \cos \theta \\
\dot{y}=l \dot{\theta} \sin \theta
\end{gathered}
$$

If we define a desired trajectory:

$$
y^{d}(t)=\frac{l}{2}+\frac{l}{4} \sin (t)
$$

then the task-space controller is easily implemented using the derivation above.

The task space derivation above provides a convenient generalization of the partial feedback linearization (PFL) [10], which encompasses both the collocated and non-collocated results. If we choose $\mathbf{y}=\mathbf{q}_{2}$ (collocated), then we have

$$
\mathbf{H}_{1}=0, \mathbf{H}_{2}=\mathbf{I}, \dot{\mathbf{H}}=0, \ddot{\mathbf{H}}=\mathbf{I}, \ddot{\mathbf{H}}^{+}=\mathbf{I}
$$

From this, the command in (38) reduces to $\ddot{\mathbf{q}}_{2}=\ddot{\mathbf{q}}_{2}^{d}$. The actuator command is then

$$
\mathbf{u}=\mathbf{M}_{21} \mathbf{M}_{11}^{-1}\left(\tau_{1}-\mathbf{M}_{12} \ddot{\mathbf{q}}_{2}^{d}\right)+\mathbf{M}_{22} \ddot{\mathbf{q}}_{2}^{d}-\tau_{2}
$$

and the rank condition (40) is always met.
If we choose $\mathbf{y}=\mathbf{q}_{1}$ (non-collocated), we have

$$
\mathbf{H}_{1}=\mathbf{I}, \mathbf{H}_{2}=0, \dot{\mathbf{H}}=0, \ddot{\mathbf{H}}=-\mathbf{M}_{11}^{-1} \mathbf{M}_{12}
$$

The rank condition (40) requires that $\operatorname{rank}\left(\mathbf{M}_{12}\right)=l$, in which case we can write $\ddot{\mathbf{H}}^{+}=-\mathbf{M}_{12}^{+} \mathbf{M}_{11}$, reducing the rank condition to precisely the "Strong Inertial Coupling" condition described in [10]. Now the command in (38) reduces to

$$
\ddot{\mathbf{q}}_{2}=\mathbf{M}_{12}^{+}\left[\tau_{1}-\mathbf{M}_{11} \ddot{\mathbf{q}}_{1}^{d}\right]
$$

The actuator command is found by substituting (45) into (33), yielding the same results as in [10].

### 3.6 SWING-UP CONTROL

### 3.6.1 Energy shaping

Recall in the last chapter, we used energy shaping to swing up the simple pendulum. This idea turns out to be a bit more general than just for the simple pendulum. As we will see, we can use similar concepts of "energy shaping" to produce swing-up controllers for the acrobot and cart-pole systems. It's important to note that it only takes one actuator to change the total energy of a system.

Although a large variety of swing-up controllers have been proposed for these model systems (c.f. [11], [12], [13], [14], [15], [16], [1], [17]), the energy shaping controllers tend to be the most natural to derive and perhaps the most well-known.

# 3.6.2 Cart-Pole 

Let's try to apply the energy-shaping ideas to the cart-pole system. The basic idea, from [18], is to use collocated PFL to simplify the dynamics, use energy shaping to regulate the pendulum to its homoclinic orbit, then to add a few terms to make sure that the cart stays near the origin. This is a bit surprising... if we want to control the pendulum, shouldn't we use the non-collocated version? Actually, we ultimately want to control both the cart and the pole, and we will build on the collocated version both because it avoids inverting the $\cos \theta$ term that can go to zero and because (when all parameters are set to 1 ) we were left with a particularly simple form of the equations:

$$
\begin{gathered}
\ddot{x}=u \\
\ddot{\theta}=-u c-s
\end{gathered}
$$

The first equation is clearly simple, but even the second equation is exactly the equations of a decoupled pendulum, just with a slightly odd control input that is modulated by $\cos \theta$.

Let us regulate the energy of this decoupled pendulum using energy shaping. The energy of the pendulum (a unit mass, unit length, simple pendulum in unit gravity) is given by:

$$
E(\mathbf{x})=\frac{1}{2} \dot{\theta}^{2}-\cos \theta
$$

The desired energy, equivalent to the energy at the desired fixed-point, is

$$
E^{d}=1
$$

Again defining $\tilde{E}(\mathbf{x})=E(\mathbf{x})-E^{d}$, we now observe that

$$
\begin{aligned}
\dot{\tilde{E}}(\mathbf{x}) & =\dot{E}(\mathbf{x})=\dot{\theta} \ddot{\theta}+\dot{\theta} s \\
& =\dot{\theta}[-u c-s]+\dot{\theta} s \\
& =-u \dot{\theta} \cos \theta
\end{aligned}
$$

Therefore, if we design a controller of the form

$$
u=k \dot{\theta} \cos \theta \tilde{E}, \quad k>0
$$

the result is

$$
\dot{\tilde{E}}=-k \dot{\theta}^{2} \cos ^{2} \theta \tilde{E}
$$

This guarantees that $|\tilde{E}|$ is non-increasing, but isn't quite enough to guarantee that it will go to zero. For example, if $\theta=\dot{\theta}=0$, the system will never move. However, if we have that

$$
\int_{0}^{t} \dot{\theta}^{2}\left(t^{\prime}\right) \cos ^{2} \theta\left(t^{\prime}\right) d t^{\prime} \rightarrow \infty, \quad \text { as } t \rightarrow \infty
$$

then we have $\tilde{E}(t) \rightarrow 0$. This condition, a version of the LaSalle's theorem that we will develop in our notes on Lyapunov methods, is satisfied for all but the trivial constant trajectories at fixed points.

Now we return to the full system dynamics (which includes the cart). [18] and [19] use the simple pendulum energy controller with an addition PD controller designed to regulate the cart:

$$
\ddot{x}^{d}=k_{E} \dot{\theta} \cos \theta \tilde{E}-k_{p} x-k_{d} \dot{x}
$$

[18] provides a proof of convergence for this controller with some nominal parameters. [11] provides a slightly different controller derived directly from a Lyapunov argument.
![img-3.jpeg](img-3.jpeg)

Figure 3.4 - Cart-Pole Swingup: Example phase plot of the pendulum subsystem using energy shaping control. The controller drives the system to the homoclinic orbit, then switches to an LQR balancing controller near the top.

# 3.6.3 Acrobot 

Swing-up control for the acrobot can be accomplished in much the same way. [14] - pump energy. Clean and simple. No proof. Slightly modified version (uses arctan instead of sat) in [20]. Clearest presentation in [19].

Use collocated PFL. $\left(\ddot{q}_{2}=\ddot{q}_{2}^{d}\right)$.

$$
\begin{gathered}
E(\mathbf{x})=\frac{1}{2} \dot{\mathbf{q}}^{T} \mathbf{M} \dot{\mathbf{q}}+U(\mathbf{x}) \\
E_{d}=U\left(\mathbf{x}^{\star}\right) \\
\ddot{u}=\dot{q}_{1} \tilde{E} \\
\ddot{q}_{2}^{d}=-k_{1} q_{2}-k_{2} \dot{q}_{2}+k_{3} \ddot{u}
\end{gathered}
$$

Extra PD terms prevent proof of asymptotic convergence to homoclinic orbit. Proof of another energy-based controller in [13].

### 3.6.4 Discussion

The energy shaping controller for swing-up presented here is a pretty faithful representative from the field of nonlinear underactuated control. Typically these control derivations require some clever tricks for simplifying or canceling out terms in the nonlinear equations, then some clever Lyapunov function to prove stability. In these cases, PFL was used to simplify the equations, and therefore the controller design.

We will see another nice example of changing coordinates in the nonlinear equations to make the problem easier when we study "differential flatness" for trajectory optimization. This approach is perhaps most famous these days for very dynamic trajectory planning with quadrotors.

These controllers are important, representative, and relevant. But clever tricks with nonlinear equations seem to be fundamentally limited. Most of the rest of the material presented in this book will emphasize more general computational approaches to formulating and solving these and other control problems.

# 3.7 OTHER MODEL SYSTEMS 

The acrobot and cart-pole systems are just two of the model systems used heavily in underactuated control research. Other examples include:

- Pendubot
- Inertia wheel pendulum
- Furuta pendulum (horizontal rotation and vertical pendulum)
- Hovercraft


### 3.8 EXERCISES

## Exercise 3.1 (Cart-Pole: Linearization and Balancing)

For this exercise you will work exclusively in this notebook. You will be asked to complete the following steps.
a. Derive the state-space dynamics $\dot{\mathbf{x}}=f(\mathbf{x}, \mathbf{u})$ of the cart-pole system.
b. Linearize the dynamics from point (a) around the unstable equilibrium point.
c. Analyze the linearization error for different values of the state $\mathbf{x}$ and the control $\mathbf{u}$.
d. Among the states from point (c), identify which ones are stabilized to the origin by the LQR controller.

## Exercise 3.2 (Cart-Poles: Writing URDFs and balancing with LQR)

For this exercise you will work in this notebook. You will complete the following steps.
a. Construct the cart-pole pendulum URDF structure for the single pendulum cart-pole..
b. Extend the single pendulum cart-pole to a double pendulum cart-pole by modifying the URDF model of the robot and testing LQR's ability to control it.

## Exercise 3.3 (Controllability of Discrete and Continuous LTI Systems)

a. (2-D grid with discrete double integrator) Consider a discretized grid world where

$$
\mathbf{x}=\left[\begin{array}{l}
x_{1} \\
x_{2}
\end{array}\right], \text { and } x_{1}[n], x_{2}[n] \in \mathbb{Z}
$$

The dynamics of $x[n]$ is a discrete-time version of double integrator,

$$
\mathbf{x}[n+1]=\left[\begin{array}{l}
x_{1}[n]+x_{2}[n] \\
x_{2}[n]+u[n]
\end{array}\right]
$$

where the control input $u[n] \in \mathbb{Z}$ is also an integer.

1. Suppose we represent the discretized double integrator dynamics on the following graph, where the nodes $s_{i, j}$ represent state vector values ( $\mathbf{x}=s_{i, j}$ means $x_{1}=i, x_{2}=j$ ) and edges are state transitions defined by a control law. In the following truncated graph, draw the edges when $u=-1, u=0$, and $u=1$. (Please draw edges for all the nine nodes in the truncated graph. If a node trainsitions to another node outside the graph, please draw the destination node as well.)
![img-4.jpeg](img-4.jpeg)
![img-5.jpeg](img-5.jpeg)
2. Suppose we start from $s_{0,0}$, construct a control input sequence ( $u[n]$ could be a function of time step and can take any arbitrary intger values) to reach an arbitrary state $s_{i, j}$ that satisfies $j>i>0$. What would be the minimal number of time steps needed to reach any $s_{i, j}$ that satisfies $j>i>0$ ? Show that the number you've chosen is indeed minimum.
3. Now consider a more general case, from an arbitrary initial state $s_{i_{1}, j_{1}}$, can you find a control input sequence $u[n]$ with the same number of time steps as in (2) that allows the state to reach any final state $s_{i_{2}, j_{2}}$ ? Also show that the number you've chosen is minimum.
b. 1. Considering linear systems

$$
\dot{\mathbf{x}}=\mathbf{A x}+\mathbf{B u}
$$

are the following linear systems controllable?

$$
\begin{gathered}
\mathbf{A}_{1}=\left[\begin{array}{ll}
1 & 0 \\
0 & 1
\end{array}\right], \mathbf{B}_{1}=\left[\begin{array}{l}
0 \\
1
\end{array}\right] ; \quad \mathbf{A}_{2}=\left[\begin{array}{ll}
1 & 0 \\
0 & 1
\end{array}\right], \mathbf{B}_{2}=\left[\begin{array}{l}
1 \\
1
\end{array}\right] \\
\mathbf{A}_{3}=\left[\begin{array}{ll}
0 & 1 \\
0 & 0
\end{array}\right], \mathbf{B}_{3}=\left[\begin{array}{l}
0 \\
1
\end{array}\right] ; \quad \mathbf{A}_{4}=\left[\begin{array}{llll}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 1 & 0 & 0 \\
0 & 2 & 0 & 0
\end{array}\right], \mathbf{B}_{4}=\left[\begin{array}{l}
0 \\
0 \\
1 \\
1
\end{array}\right]
\end{gathered}
$$

You should only need the conditions described in the definition of controllability to explain your conclusion. You can also use the more general tools found in this collapsable section; these will prove especially for the $\left(\mathbf{A}_{4}, \mathbf{B}_{4}\right)$ matrix pair.
2. Consider the two systems, $\left(\mathbf{A}_{3}, \mathbf{B}_{3}\right)$ and $\left(\mathbf{A}_{4}, \mathbf{B}_{4}\right)$. Can you make a conclusion whether these two systems are underactuated? (Note that the definition of underactuation requires the system to be interpreted as a second-order system, i.e., $\mathbf{x}=[\mathbf{q}, \dot{\mathbf{q}}]^{T}$

# Exercise 3.4 (Comparison between stability of nonlinear system and linearization) 

In this problem, we use phase portraits to examine how linearization technique helps with local stability analysis for nonlinear systems and its limitations using the nonlinear pendulum with zero torque as an example. The pendulum parameters are chosen as $m=1, l=1, g=9.81$. Follow the exercises below and complete the graphical analysis for equilibrium points $\mathbf{x}^{*}=[0,0]^{T}, \mathbf{x}^{*}=[\pi, 0]^{T}$, which will guide you to studying the systems. When you are done, populate this table:

|  | $\operatorname{sign}\left(\operatorname{Re}\left(\lambda_{1}\right)\right)$ | $\operatorname{sign}\left(\operatorname{Re}\left(\lambda_{2}\right)\right)$ | stable i.s.L. | Asymp. Stable | Exp. Stable |
| :--: | :--: | :--: | :--: | :--: | :--: |
| Nonlinear $b=0, \mathbf{x}^{*}=[0,0]^{T}$ | N/A | N/A |  |  |  |
| Nonlinear $b=0, \mathbf{x}^{*}=[\pi, 0]^{T}$ | N/A | N/A |  |  |  |
| Linearization $b=0, \mathbf{x}^{*}=[0,0]^{T}$ |  |  |  |  |  |
| Linearization $b=0, \mathbf{x}^{*}=[\pi, 0]^{T}$ |  |  |  |  |  |
| Nonlinear $b=1, \mathbf{x}^{*}=[0,0]^{T}$ | N/A | N/A |  |  |  |
| Nonlinear $b=1, \mathbf{x}^{*}=[\pi, 0]^{T}$ | N/A | N/A |  |  |  |
| Linearization $b=1, \mathbf{x}^{*}=[0,0]^{T}$ |  |  |  |  |  |
| Linearization $b=1, \mathbf{x}^{*}=[\pi, 0]^{T}$ |  |  |  |  |  |

a. Let's first look at an undamped pendulum $b=0$.

1. Using this notebook as an example, draw the phase portrait using plot_2d_phase_portrait function around equilibrium $\mathbf{x}^{*}$. Is the nonlinear undamped pendulum stable i.s.L.? asymptotically stable? exponentially stable?
2. Derive the linearization of the undamped nonlinear pendulum around equilibrium $\mathbf{x}^{*}$. Compute the eigenvalues of the Jacobian $\left[\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\right]_{\mathbf{x}=\mathbf{x}^{*}}$, what are the signs of the real parts of these two eigenvalues, $\operatorname{sign}\left(\operatorname{Re}\left(\lambda_{1}\right)\right), \operatorname{sign}\left(\operatorname{Re}\left(\lambda_{2}\right)\right)$ ?
3. Using phase portrait, is the linearized system $\dot{\mathbf{x}}=\left[\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\right]_{\mathbf{x}=\mathbf{x}^{*}}\left(\mathbf{x}-\mathbf{x}^{*}\right)$ stable i.s.L.? exponentially stable?
b. Now let's consider a pendulum with damping coefficient $b=1$.
4. Using this notebook as an example, draw the phase portrait using plot_2d_phase_portrait function around equilibrium $\mathbf{x}^{*}$. Is the nonlinear damped

pendulum stable i.s.L.? asymptotically stable? exponentially stable?
2. Derive the linearization of the damped nonlinear pendulum around equilibrium $\mathbf{x}^{+}$. Compute the eigenvalues of the Jacobian $\left[\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\right]_{\mathbf{x}=\mathbf{x}^{*}}$, what are the signs of the real parts of these two eigenvalues, $\operatorname{sign}\left(\operatorname{Re}\left(\lambda_{1}\right)\right), \operatorname{sign}\left(\operatorname{Re}\left(\lambda_{2}\right)\right)$ ?
3. Using phase portrait, is the linearized system $\dot{\mathbf{x}}=\left[\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\right]_{\mathbf{x}=\mathbf{x}^{*}}\left(x-\mathbf{x}^{*}\right)$ stable i.s.L.? exponentially stable?

# REFERENCES 

1. Richard M. Murray and John Hauser, "A case Study in Approximate Linearization: The Acrobot Example", Memorandum No. UCB/ERL (unknown), April, 1991.
2. M. W. Spong, "Underactuated Mechanical Systems", Control Problems in Robotics and Automation, 1997.
3. Hassan K. Khalil, "Nonlinear Systems", Prentice Hall, December, 2001.
4. Jean-Jacques E. Slotine and Weiping Li, "Applied Nonlinear Control", Prentice Hall , October, 1990.
5. Katsuhiko Ogata, "Modern Control Engineering", Prentice Hall Incorporated, August, 1996.
6. Gilbert Strang, "Introduction to Linear Algebra", Wellesley-Cambridge Press, October, 1998.
7. Chi-Tsong Chen, "Linear System Theory and Design", Oxford University Press, Sept 10, 1998.
8. R.W. Brockett, "Asymptotic stability and feedback stabilization", Differential Geometric Control Theory, pp. 181-191, 1983.
9. Kemin Zhou and John C. Doyle, "Essentials of Robust Control", Prentice Hall , 1997.
10. Mark Spong, "Partial feedback linearization of underactuated mechanical systems", Proceedings of the IEEE International Conference on Intelligent Robots and Systems, vol. 1, pp. 314-321, September, 1994.
11. Isabelle Fantoni and Rogelio Lozano, "Non-linear Control for Underactuated Mechanical Systems", Springer-Verlag , 2002.
12. Araki and N.; Okada and M.; Konishi and Y.; Ishigaki and H.;, "Parameter identification and swing-up control of an Acrobot system", International Conference on International Technology, 2005.
13. Xin Xin and M. Kaneda, "New analytical results of the energy based swinging up control of the Acrobot", Proceedings of the 43rd IEEE Conference on Decision and Control (CDC), vol. 1, pp. 704 - 709, Dec, 2004.
14. Mark W. Spong, "Swing up control of the Acrobot", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2356-2361, 1994.
15. Arun D. Mahindrakar and Ravi N. Banavar, "A swing-up of the acrobot based on a simple pendulum strategy", International Journal of Control, vol. 78, no. 6, pp. 424-429, April,

2005. 
2006. M.D. Berkemeier and R.S. Fearing, "Tracking fast inverted trajectories of the underactuated Acrobot", Robotics and Automation, IEEE Transactions on, vol. 15, no. 4, pp. 740-750, Aug, 1999.
1. Xu-Zhi Lai and Jin-Hua She and Simon X. Yang and Min Wu, "Stability Analysis and Control Law Design for Acrobots", Proceedings of the 2006 IEEE International Conference on Robotics and Automation, May, 2006.
2. Chung Choo Chung and John Hauser, "Nonlinear Control of a Swinging Pendulum", Automatica, vol. 31, no. 6, pp. 851-862, June, 1995.
3. Mark W. Spong, "Energy Based Control of a Class of Underactuated Mechanical Systems", Proceedings of the 1996 IFAC World Congress, 1996.
4. Mark Spong, "The Swingup Control Problem for the Acrobot", IEEE Control Systems Magazine, vol. 15, no. 1, pp. 49-55, February, 1995.

Previous Chapter
Table of contents
Next Chapter
Accessibility
(c) Russ Tedrake, 2024
# Energy-Based Control for a Class of Under-Actuated Mechanical Systems 

Yunyun Dong ${ }^{1}$,Zhonghua Wang ${ }^{1}$,Zhiquan Feng ${ }^{2}$,Dongxue Wang ${ }^{1}$,Hui Fang ${ }^{1}$<br>${ }^{1}$ School of Control Science and Engineering<br>${ }^{2}$ School of Information Science and Engineering<br>University of Jinan, Jinan, China, 250022<br>dyyun82860948@163.com


#### Abstract

In this paper ${ }^{1}$, we discuss an energy-based control method for a class of under-actuated mechanical systems. The control law is proposed and the convergence analysis is carried out based on Lyapunov stability theory. The conditions on the parameters in the control law such that the total energy of the controlled objects converge to the potential energy of their desired position are given. Under such convergence of the energy, simulation results are presented showing the performance of the systems.


## 1. Introduction

In recent years, there have been growing attention and increasing interest in under-actuated mechanical systems[1]. Under-actuated mechanical systems are those possessing fewer actuators than degrees of freedom. Thus, the design method of controllers of under-actuated systems gives some reductions of numbers of necessary actuators, of the cost and of the weight of systems, hence this problem is interesting research topics in industry.

At one time, we have shown that the collocated linearization methods[2] provide effective design tools for the trajectory tracking control of a class of under-actuated mechanical systems. In this method, the dynamics corresponding to the active degrees of freedom are linearized based on the exact linearization design technique[3], while the dynamics corresponding to the passive degrees are taken as the internal dynamics of the system. The trajectory tracking control of the linearized system is studied by choosing the active degrees as the system outputs. The analysis of the system internal dynamics shows that the stability of the zero dynamics guarantees the stability of the control system, but that method is lack of robustness.

[^0]There is not a uniform useful theory to solve the control problem of under-actuated mechanical systems, so many researchers have to analyze the systems properties, choose and fit some common techniques or propose new techniques.

Energy-based control for a class of under-actuated mechanical systems is proposed in this paper, which is based on the Lyapunov theory[4] and the systems passivity. The rest of the paper is organized as follows. Section 2 presents the dynamic model of under-actuated mechanical systems. The section 3 is the controller design and the stability analysis. The simulation results in order to illustrate the validity of the controller are arranged in section 4. Finally, the conclusion is presented in section 5.

## 2. Under-actuated systems dynamic

Based on the Lagrangian formulation[5], and under the standard assumption such as a no friction, the dynamic equations for the under-actuated mechanical systems having variables (assumption of passive variables and active variables), written in the usual way, as

$$
D(q) \ddot{q}+C(q, \dot{q}) \dot{q}+G(q)=\tau
$$

where is $q \in R^{n}$ the generalized coordinates(position vectors), $D(q) \in R^{n \times n}$ is the symmetric, positive definite inertia matrix vector function; represents the Coriolis and centripetal forces, and $\tau=\left[\tau_{a}, 0\right]^{T} \in R^{n}$ represents the input generalized force produced by the m actuators. The dynamic equation has a structure property: the matrix $D(q)-2 C(q, \dot{q})$ is skew-symmetric with a suitable definition of $C(q, \dot{q}) . G(q)$ is the term derived from the potential energy, such as gravitational and elastic generalized forces.

Without loss of generality, the generalized position vectors of the under-actuated mechanical systems can be pati tioned into two groups: $q=\left[q_{a}, q_{a}\right]^{T}$, where $q_{a} \in R^{m}$ is the actuated generalized vectors, and $q_{a} \in R^{p}$ is the unactuated


[^0]:    ${ }^{1}$ This paper is supported by National Natural Science Foundation of China under Grant 60773109 and Natural Science Foundation of Shandong Province under Grant Y2006G26.

position vectors. We may write the system (1) as

$$
\begin{aligned}
{\left[\begin{array}{cc}
d_{11} & d_{12} \\
d_{21} & d_{21}
\end{array}\right]\left[\begin{array}{c}
\ddot{q}_{a} \\
\ddot{q}_{a}
\end{array}\right] } & +\left[\begin{array}{cc}
c_{11} & c_{12} \\
c_{21} & c_{21}
\end{array}\right]\left[\begin{array}{c}
\dot{q}_{a} \\
\dot{q}_{a}
\end{array}\right] \\
& +\left[\begin{array}{c}
g_{1}(q) \\
g_{2}(q)
\end{array}\right]=\left[\begin{array}{c}
\tau_{a} \\
0
\end{array}\right]
\end{aligned}
$$

and, in particular, the dynamic equation relative to the unactuated subsystem is

$$
d_{21} \ddot{q}_{a}+d_{22} \ddot{q}_{u}+c_{21} \dot{q}_{a}+c_{22} \dot{q}_{u}+g_{2}(q)=0
$$

The equation (3) represents a second-order non-holonomic constraint. The dynamic equation of the actuated subsystem is

$$
d_{11} \ddot{q}_{a}+d_{12} \ddot{q}_{u}+c_{11} \dot{q}_{a}+c_{12} \dot{q}_{u}+g_{1}(q)=\tau_{a}
$$

From (2) we can see that the system has generalized coordinates, but only control inputs. Passivity terms explicitly appears in equation (3), which may thus be interpreted as an dimentional constraints involving generalized coordinates as well as their first and second time derivatives.

## 3. Controller Design and Stability Analysis

The desired generalized position vector of the underactuated mechanical system is supposed at $q_{d}$ (a desired position), and the corresponding desired actuated generalized position is $q_{a}^{d}$.

The passivity property[6] of the system suggests us to use the total energy in the controller design. Let us consider $\ddot{q}_{a}=q_{a}-q_{a}^{d}$ and $\dot{E}=E(q, \dot{q})-P\left(q_{d}\right)$, where $E(q, \dot{q})=$ $\frac{1}{2} \dot{q}^{T} D(q) \dot{q}+P(q)$ is the total energy of the under-actuated mechanical system, $P(q)$ is the total potential energy of the under-actuated mechanical system, $P\left(q_{d}\right)$ is the system potential energy in the desired position. We wish to bring to zero $\dot{q}_{a}, \dot{q}_{a}$, and $E$. We propose the following Lyapunov function candidate:

$$
\begin{gathered}
V=\frac{1}{j} k_{E}\left(E(q, \dot{q})-P\left(q_{d}\right)\right)^{j}+\frac{1}{2} k_{D} \dot{q}_{a}^{T} \dot{q}_{a} \\
+\frac{1}{2} k_{P}\left(q_{a}-q_{a}^{d}\right)^{T}\left(q_{a}-q_{a}^{d}\right) \\
j=1 \text { or } 2
\end{gathered}
$$

where $k_{E}, k_{D}$ and $k_{P}$ are strictly positive constant matrix to be defined later,j is the constant 1 or 2 to ensure the Lyapunov function is positive, therefore, the defined the Lyapunov function is a positive definite function. Note that $V(q, \dot{q})$ is a positive semi-definite function. Differentiating and using systems' passivity properties, we can obtain the control law.

First, we take $j=1$ to calculate the energy based control. The differential of Lyapunov function can be calculated

$$
\begin{aligned}
\dot{V}(q, \dot{q}) & =k_{E} \dot{E}(q, \dot{q})+k_{D} \dot{q}_{a}^{T} \ddot{q}_{a}+k_{p}\left(\dot{q}_{a}-\dot{q}_{a}^{d}\right)^{T}\left(q_{a}-q_{a}^{d}\right) \\
& =\dot{q}_{a}^{T}\left(k_{E} \tau+k_{D} \ddot{q}_{a}+k_{p}\left(q_{a}-q_{a}^{d}\right)\right)
\end{aligned}
$$

In order to ensure the stability of the system, we take

$$
\dot{V}(q, \dot{q})=-k \dot{q}_{a}^{T} \ddot{q}_{a}
$$

where $k$ is a positive, we have

$$
k_{E} \tau_{a}+k_{D} \ddot{q}_{a}+k_{P}\left(q_{a}-q_{a}^{d}\right)=-k \dot{q}_{a}
$$

so

$$
\begin{aligned}
\tau_{a}= & -\left(k_{E} I+k_{D} Z D^{-1}(q) Z^{T}\right)^{-1}\left(k_{P}\left(q_{a}-q_{a}^{d}\right)+\right. \\
& \left.k_{D} Z D^{-1}(q)(C(q, \dot{q}) \dot{q}+G(q))+k \dot{q}_{a}\right)
\end{aligned}
$$

where $Z=[I 0], Z \in R^{m \times n}, I \in R^{m \times m}$.
For $j=1$, from equation (5) and equation (7), we can get $V(q, \dot{q}), \dot{V}(q, \dot{q})$, the under-actuated mechanical system is asymptotic stable. Moreover $E(q, \dot{q}), D(q)$ and $q_{a}$ are bounded. Therefore, q and $\dot{q}$ are bounded. $\ddot{q}$ is bounded from equation (1), and then $\dot{V}(q, \dot{q})=-2 \dot{q}_{a}^{T} \ddot{q}_{a}$ is bounded. Thus, $\dot{V}(q, \dot{q})$ is uniformly continuous. According to the Barbalat's lemma, it can be obtained:
$\lim _{t \rightarrow \infty} \dot{V}(q, \dot{q})$, So $\lim _{t \rightarrow \infty} \dot{q}_{a}=0, \lim _{t \rightarrow \infty} E(q, \dot{q}), \lim _{t \rightarrow \infty} V(q, \dot{q})$, $\lim _{t \rightarrow \infty}\left(q_{a}-q_{a}^{d}\right)$ and $\lim _{t \rightarrow \infty} \tau$ are constants from the system's passivity properties and the equation(7) and equation(9). If $\lim _{t \rightarrow \infty} \tau \neq 0$, then $\left(q_{a}-q_{a}^{d}\right)$ will change with $t \rightarrow \infty$, which is inconsistent with $\lim _{t \rightarrow \infty}\left(q_{a}-q_{a}^{d}\right)=0$ is constant. Then the following equation must be true: $\lim _{t \rightarrow \infty} \tau=0$. It follows from equation(9): $\lim _{t \rightarrow \infty}\left(q_{a}-q_{a}^{d}\right)=0, \lim _{t \rightarrow \infty} V(q, \dot{q})=$ $\lim _{t \rightarrow \infty} k_{E}\left(E(q, \dot{q})-P\left(q_{a}\right)\right)$. After the control algorithm is added to the under-actuated mechanical system, the system dynamics will change until arrives at its minimum, i.e. arrives at its minimum. When arrives at its minimum, the kinetic energy of the under-actuated system is zero and the potential energy arrive at its minimum. At this time, $q=q_{d}$, $\dot{q}=0$, the system will stabilize at its desired position.

Second, we take $j=2$ to calculate the energy-based control is

$$
\begin{aligned}
& \tau_{a}=-\left(k_{E}\left(E(q, \dot{q})-P\left(q_{a}\right)+k_{D} Z D^{-1}(q) Z^{T}\right)^{-1} \times\right. \\
& \left(k_{P}\left(q_{a}-q_{a}^{d}\right)+k_{D} Z D^{-1}(q)(C(q, \dot{q}) \dot{q}+G(q))+k \dot{q}_{a}\right.
\end{aligned}
$$

where $Z=[I 0], Z \in R^{m \times n}, I \in R^{m \times m}$
For $j=2$, the energy-based control law(10) can also stabilize the under-actuated mechanical system to its desired position $q_{a}^{d}$. The similar analysis algorithm do not repeat here.

![img-0.jpeg](img-0.jpeg)

Fig. 1. Overhead crane system
![img-1.jpeg](img-1.jpeg)

Fig. 2. States of the system

## 4. Simulation Results

In this section we give examples of energy-based analysis method for a class of under-actuated mechanical systems in order to observe the performance of the proposed control law, we have performed simulations on MATLAB.

As a typical representation, overhead crane[7] system (shown as Fig.1) is a typical under-actuated system. The control object of the overhead crane is to move the trolley to its destination and complement anti-swing of the load at the same time.

For simplicity, the following assumptions are made :
a) The trolley and the load can be regarded as point masses
b) Friction force which may exists in the trolley can be neglected
c) Elongation of the rope due to tension force is neglected
d) The trolley moves along the rail and the load moves in the X-Y plane.

From Fig. 1 we can find that $x_{m}=x+L \sin (q), y_{m}=$
![img-2.jpeg](img-2.jpeg)

Fig. 3. Performance of the system
$-L \cos (q)$. Using Lagrange's method, we can obtain the model of the overhead crane system as follow:

$$
\begin{aligned}
(M+m) \ddot{x}+m L\left(\ddot{q} \cos (q)-\dot{q}^{2} \sin (q)\right) & =F \\
\ddot{x} \cos (q)+L \ddot{q}+g \sin (q) & =0
\end{aligned}
$$

where $M$ is the mass of the trolley and m is the mass of the load. $q$ is the sway angle of load and $L$ is the length of suspension rope, $F$ is the vector of control torque added to the trolley. When the proposed energy-based control algorithm is added to the overhead crane system, the system dynamics are simulated and the results are showed in Fig. 2 and Fig.3.

Fig. 2 shows the displacement of the trolley and the swing angle of the load with the proposed method. The simulation results show that the energy control scheme can control the trolley to destination and implement anti-sway control at the same time. With the choice of the controller parameters, such as the $k_{D}$, the method can greatly reduce the number of system states that need measured. Fig. 3 shows that $\bar{E}$ goes to zero, i.e. that the energy $E$ goes to the energy at the desired position $P\left(q_{d}\right)$. The Lyapunov function $V$ is always decreasing and converges to zero.

As another typical representation, under-actuated robots are referred to as a class of mechanical systems with fewer control inputs than degrees of freedom. This class of robots has many advantages such as light weight, low cost and low energy consumption and recently more and more researchers have paid their attention to this new field. Now, we also study the swing-up control for the under-actuated manipulators Pendubot (shown as Fig.4) based on energy method in the vertical plane. Pendubot is a two-link robot with an actuator at the shoulder but no actuator at the elbow. The equations of motion for the Pendubot are:

$$
\begin{aligned}
& d_{11} \ddot{q}_{1}+d_{12} \ddot{q_{2}}+c_{11} \dot{q}_{1}+c_{12} \dot{q}_{2}+g_{1}=\tau_{a} \\
& d_{21} \ddot{q}_{1}+d_{22} \ddot{q_{2}}+c_{21} \dot{q}_{1}+c_{22} \dot{q}_{2}+g_{2}=0
\end{aligned}
$$

![img-3.jpeg](img-3.jpeg)

Fig. 4. The pendubot
where

$$
\begin{aligned}
d_{11} & =\theta_{1}+\theta_{2}+2 \theta_{3} \cos \left(q_{2}\right) \\
d_{12} & =\theta_{2}+\theta_{3} \cos \left(q_{2}\right) \\
d_{21} & =\theta_{2}+\theta_{3} \cos \left(q_{2}\right) \\
d_{22} & =\theta 2 \\
c_{11} & =-\theta_{3} \sin \left(q_{2}\right) \dot{q}_{2} \\
c_{12} & =-\theta_{3} \sin \left(q_{2}\right) \dot{q}_{2}-\theta_{3} \sin \left(q_{2}\right) \dot{q}_{1} \\
c_{21} & =\theta_{3} \sin \left(q_{2}\right) \dot{q}_{1} \\
c_{22} & =0 \\
g_{1} & =\theta_{4} g \cos \left(q_{1}\right)+\theta_{5} g \cos \left(q_{1}+q_{2}\right) \\
g_{2} & =\theta_{5} g \cos \left(q_{1}+q_{2}\right) \\
\theta_{1} & =m_{1} l_{c 1}^{2}+m_{2} l_{1}^{2}+I_{1} \\
\theta_{2} & =m_{2} l_{c 2}^{2}+I_{2} \\
\theta_{3} & =m_{2} l_{1} l_{c 2} \\
\theta_{4} & =m_{1} l_{c 1}+m_{2} l_{1} \\
\theta_{5} & =m_{2} l_{c 2}
\end{aligned}
$$

Parameters of the Simulated Pendubot[8]

$$
\begin{aligned}
\theta_{1} & =0.0260 \\
\theta_{2} & =0.0119 \\
\theta_{3} & =0.0098 \\
\theta_{4} & =0.1673 \\
\theta_{5} & =0.0643
\end{aligned}
$$

Fig. 5 shows the simulations that our control law brings the state of the system to the homoclinic orbit. Fig. 6 shows that $\bar{E}$ goes to zero. $E$ goes to the energy at the top position: $E_{\text {top }}$. The Lyapunov function V is always decreasing and converges to zero.
![img-4.jpeg](img-4.jpeg)

Fig. 5. States of the pendubot
![img-5.jpeg](img-5.jpeg)

Fig. 6. Performance of the pendubot

# 5. Conclusions 

In this paper, an energy-based control method is proposed for the under-actuated mechanical system. The control scheme is based on an energy approach and the passivity properties of the under-actuated mechanical systems. A Lyapunov function is obtained using the total energy of the system. The under-actuated overhead crane system and the Pendubot are used to validity the proposed control scheme. Simulation results illustrate the effectiveness of proposed control algorithm.

## References

[1] Spong, M.W, "Partial feedback linearization of underactuated mechanical systems", in Proc. IEEE/RSJ/GI Int.Conf. Intelligent Robots Systems, vol.1, 1994, pp.314-321.
[2] Dong Yunyun,Wang Zhonghua. "Trajectory tracking of under-actuated mechanical robots based on partial feedback linearization",Information technology and information,no.129,June.2007,pp.81-83.
[3] M. W. Spong, "The swing up control problem for the Acrobot". IEEE Control Systems Magazine, vol.15, no.1, Feb.1995, pp.49-55.
[4] Henmi,T.; Mingcong Deng; Inoue, "A Swing-up Control of the Acrobot Using a New Partial Linearization Controller Based on the Lyapunov Theorem. Networking", Sensing and Control, 2006, Apr.2006, pp.60-65.
[5] Seiji Maruyama; Tomohiro Henmi; Mingcong Deng; Akira Inoue and Nobuyuki Ueki, "Design of swing-up controller and experimental study of an Acrobot ",Proceeding of the 2nd International Symposium on Advanced Control of Industrial Processes, 2005.
[6] Arjan van der Schaft, Gain and Passivity Techniques in Nonlinear Control, New York: Springer. 2000.
[7] Nalley M.J.; Trabia M. B. "Control of Overhead Cranes Using a Fuzzy Logic Controller",Journal of Intelligent and fuzzy System, no.8, 2000, pp.1-18.
[8] Wei Wang, J. Q. Yi,D.B Zhao, X. J. Liu, "Adaptive sliding mode controller for an under-actuated manipulator", The International Conference on Machine Learning and Cybernetics, Aug.2004, pp.882-887.