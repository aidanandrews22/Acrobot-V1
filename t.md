Ch. 3 - Acrobots, Cart-Poles, and Quadrotors  
# [Underactuated Robotics](index.html)

Algorithms for Walking, Running, Swimming, Flying, and Manipulation

[Russ Tedrake](http://people.csail.mit.edu/russt/)

© Russ Tedrake, 2024  
Last modified .  
var d = new Date(document.lastModified); document.getElementById("last\_modified").innerHTML = d.getFullYear() + "-" + (d.getMonth()+1) + "-" + d.getDate(); [How to cite these notes, use annotations, and give feedback.](misc.html)  

**Note:** These are working notes used for [a course being taught at MIT](https://underactuated.csail.mit.edu/Spring2024/). They will be updated throughout the Spring 2024 semester. [Lecture videos are available on YouTube](https://www.youtube.com/playlist?list=PLkx8KyIQkMfU5szP43GlE_S1QGSPQfL9s).

|     |     |     |
| --- | --- | --- |
| [Previous Chapter](pend.html) | [Table of contents](index.html) | [Next Chapter](simple_legs.html) |

document.write(notebook\_header('acrobot'))

# Acrobots, Cart-Poles, and Quadrotors

A great deal of work in the control of underactuated systems has been done in the context of low-dimensional model systems. These model systems capture the essence of the problem without introducing all of the complexity that is often involved in more real-world examples. In this chapter we will focus on two of the most well-known and well-studied model systems--the Acrobot and the Cart-Pole. After we have developed some tools, we will see that they can be applied directly to other model systems; we will give a number of examples using Quadrotors. All of these systems are trivially underactuated, having fewer actuators than degrees of freedom.

# The Acrobot

The Acrobot is a planar two-link robotic arm in the vertical plane (working against gravity), with an actuator at the elbow, but no actuator at the shoulder. It was first described in detail in Murray91. The companion system, with an actuator at the shoulder but not at the elbow, is known as the PendubotSpong97. The Acrobot is so named because of its resemblance to a gymnast (or acrobat) on a parallel bar, who controls his/her motion predominantly by effort at the waist (and not effort at the wrist). The most common control task studied for the Acrobot is the swing-up task, in which the system must use the elbow (or waist) torque to move the system into a vertical configuration then balance.

![](figures/acrobot.svg)

The Acrobot. [Click here to see a physical Acrobot swing up and balance](http://youtu.be/FeCwtvrD76I).

The Acrobot is representative of the primary challenge in underactuated robots. In order to swing up and balance the entire system, the controller must reason about and exploit the state-dependent coupling between the actuated degree of freedom and the unactuated degree of freedom. It is also an important system because, as we will see, it closely resembles one of the simplest models of a walking robot.

# Equations of motion

Figure 3.1 illustrates the model parameters used in our analysis. $\\theta\_1$ is the shoulder joint angle, $\\theta\_2$ is the elbow (relative) joint angle, and we will use $\\bq = \[\\theta\_1,\\theta\_2\]^T$, $\\bx = \[\\bq,\\dot\\bq\]^T$. The zero configuration is with both links pointed directly down. The moments of inertia, $I\_1,I\_2$ are taken about the pivots. The task is to stabilize the unstable fixed point $\\bx = \[\\pi,0,0,0\]^T$.

We will derive the equations of motion for the Acrobot using the method of Lagrange. The locations of the center of mass of each link, $\\bp\_{c1}, \\bp\_{c2},$ are given by the kinematics: \\begin{equation} \\bp\_{c1} = \\begin{bmatrix} l\_{c1} s\_1 \\\\ -l\_{c1} c\_1 \\end{bmatrix}, \\quad \\bp\_{c2} = \\begin{bmatrix} l\_1 s\_1 + l\_{c2} s\_{1+2} \\\\ -l\_1 c\_1 - l\_{c2} c\_{1+2} \\end{bmatrix}, \\end{equation} where $s\_1$ is shorthand for $\\sin(\\theta\_1)$, $c\_{1+2}$ is shorthand for $\\cos(\\theta\_1+\\theta\_2)$, etc. The energy is given by: \\begin{gather} T = T\_1 + T\_2, \\quad T\_1 = \\frac{1}{2} I\_1 \\dot{q}\_1^2 \\\\ T\_2 = \\frac{1}{2} ( m\_2 l\_1^2 + I\_2 + 2 m\_2 l\_1 l\_{c2} c\_2 ) \\dot{q}\_1^2 + \\frac{1}{2} I\_2 \\dot{q}\_2^2 + (I\_2 + m\_2 l\_1 l\_{c2} c\_2) \\dot{q}\_1 \\dot{q}\_2 \\\\ U = -m\_1 g l\_{c1} c\_1 - m\_2 g (l\_1 c\_1 + l\_{c2} c\_{1+2}) \\end{gather} Entering these quantities into the Lagrangian yields the equations of motion: \\begin{gather} (I\_1 + I\_2 + m\_2 l\_1^2 + 2m\_2 l\_1 l\_{c2} c\_2) \\ddot{q}\_1 + (I\_2 + m\_2 l\_1 l\_{c2} c\_2)\\ddot{q}\_2 - 2m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_1 \\dot{q}\_2 \\\\ \\quad -m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_2^2 + m\_1 g l\_{c1}s\_1 + m\_2 g (l\_1 s\_1 + l\_{c2} s\_{1+2}) = 0 \\\\ (I\_2 + m\_2 l\_1 l\_{c2} c\_2) \\ddot{q}\_1 + I\_2 \\ddot{q}\_2 + m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_1^2 + m\_2 g l\_{c2} s\_{1+2} = \\tau \\end{gather} In standard, manipulator equation form: $$\\bM(\\bq)\\ddot\\bq + \\bC(\\bq,\\dot\\bq)\\dot\\bq = \\btau\_g(\\bq) + \\bB\\bu,$$ using $\\bq = \[\\theta\_1,\\theta\_2\]^T$, $\\bu = \\tau$ we have: \\begin{gather} \\bM(\\bq) = \\begin{bmatrix} I\_1 + I\_2 + m\_2 l\_1^2 + 2m\_2 l\_1 l\_{c2} c\_2 & I\_2 + m\_2 l\_1 l\_{c2} c\_2 \\\\ I\_2 + m\_2 l\_1 l\_{c2} c\_2 & I\_2 \\end{bmatrix},\\label{eq:Hacrobot}\\\\ \\bC(\\bq,\\dot{\\bq}) = \\begin{bmatrix} -2 m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_2 & -m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_2 \\\\ m\_2 l\_1 l\_{c2} s\_2 \\dot{q}\_1 & 0 \\end{bmatrix}, \\\\ \\btau\_g(\\bq) = \\begin{bmatrix} -m\_1 g l\_{c1}s\_1 - m\_2 g (l\_1 s\_1 + l\_{c2}s\_{1+2}) \\\\ -m\_2 g l\_{c2} s\_{1+2} \\end{bmatrix}, \\quad \\bB = \\begin{bmatrix} 0 \\\\ 1 \\end{bmatrix}. \\end{gather}

# Dynamics of the Acrobot

You can experiment with the Acrobot dynamics in using, e.g.

document.write(notebook\_link('acrobot','acrobot'))

# The Cart-Pole system

The other model system that we will investigate here is the cart-pole system, in which the task is to balance a simple pendulum around its unstable equilibrium, using only horizontal forces on the cart. Balancing the cart-pole system is used in many introductory courses in control, including 6.003 at MIT, because it can be accomplished with simple linear control (e.g. pole placement) techniques. In this chapter we will consider the full swing-up and balance control problem, which requires a full nonlinear control treatment.

![](figures/cartpole.svg)

The Cart-Pole system. Click [here to see a real robot](http://youtu.be/Bzq96V1yN5k).

add swing-up + balance swf

The figure shows our parameterization of the system. $x$ is the horizontal position of the cart, $\\theta$ is the counter-clockwise angle of the pendulum (zero is hanging straight down). We will use $\\bq = \[x,\\theta\]^T$, and $\\bx = \[\\bq,\\dot\\bq\]^T$. The task is to stabilize the unstable fixed point at $\\bx = \[0,\\pi,0,0\]^T.$

# Equations of motion

The kinematics of the system are given by \\begin{equation}\\bx\_1 = \\begin{bmatrix} x \\\\ 0 \\end{bmatrix}, \\quad \\bx\_2 = \\begin{bmatrix} x + l\\sin\\theta \\\\ -l\\cos\\theta \\end{bmatrix}. \\end{equation} The energy is given by \\begin{align} T=& \\frac{1}{2} (m\_c + m\_p)\\dot{x}^2 + m\_p \\dot{x}\\dot\\theta l \\cos{\\theta} + \\frac{1}{2}m\_p l^2 \\dot\\theta^2 \\\\ U =& -m\_p g l \\cos\\theta. \\end{align} The Lagrangian yields the equations of motion: \\begin{gather} (m\_c + m\_p)\\ddot{x} + m\_p l \\ddot\\theta \\cos\\theta - m\_p l \\dot\\theta^2 \\sin\\theta = f\_x \\\\ m\_p l \\ddot{x} \\cos\\theta + m\_p l^2 \\ddot\\theta + m\_p g l \\sin\\theta = 0 \\end{gather} In standard, manipulator equation form: $$\\bM(\\bq)\\ddot\\bq + \\bC(\\bq,\\dot\\bq)\\dot\\bq = \\btau\_g(\\bq) + \\bB\\bu,$$ using $\\bq = \[x,\\theta\]^T$, $\\bu = f\_x$, we have: \\begin{gather\*} \\bM(\\bq) = \\begin{bmatrix} m\_c + m\_p & m\_p l \\cos\\theta \\\\ m\_p l \\cos\\theta & m\_p l^2 \\end{bmatrix}, \\quad \\bC(\\bq,\\dot{\\bq}) = \\begin{bmatrix} 0 & -m\_p l \\dot\\theta \\sin\\theta \\\\ 0 & 0 \\end{bmatrix}, \\\\ \\btau\_g(\\bq) = \\begin{bmatrix} 0 \\\\ - m\_p gl \\sin\\theta \\end{bmatrix}, \\quad \\bB = \\begin{bmatrix} 1 \\\\ 0 \\end{bmatrix} \\end{gather\*} In this case, it is particularly easy to solve directly for the accelerations: \\begin{align} \\ddot{x} =& \\frac{1}{m\_c + m\_p \\sin^2\\theta}\\left\[ f\_x + m\_p \\sin\\theta (l \\dot\\theta^2 + g\\cos\\theta)\\right\] \\label{eq:ddot\_x}\\\\ \\ddot{\\theta} =& \\frac{1}{l(m\_c + m\_p \\sin^2\\theta)} \\left\[ -f\_x \\cos\\theta - m\_p l \\dot\\theta^2 \\cos\\theta \\sin\\theta - (m\_c + m\_p) g \\sin\\theta \\right\] \\label{eq:ddot\_theta} \\end{align} In some of the analysis that follows, we will study the form of the equations of motion, ignoring the details, by arbitrarily setting all constants to 1: \\begin{gather} 2\\ddot{x} + \\ddot\\theta \\cos\\theta - \\dot\\theta^2 \\sin\\theta = f\_x \\label{eq:simple}\\\\ \\ddot{x}\\cos\\theta + \\ddot\\theta + \\sin\\theta = 0. \\label{eq:simple2} \\end{gather}

# Dynamics of the Cart-Pole System

You can experiment with the Cart-Pole dynamics in using, e.g.

document.write(notebook\_link('acrobot','cartpole'))

# Quadrotors

Quadrotors have become immensely popular over the last few years -- advances in outrunner motors from the hobby community made them powerful, light-weight, and inexpensive! They are strong enough to carry an interesting payload (e.g. of sensors for mapping / photographing the environment), but dynamic enough to move relatively quickly. The most interesting dynamics start showing up at higher speeds, when the propellers start achieving lift from the airflow across them due to horizontal motion, or when they are close enough to the ground to experience significant ground-effect, but we won't try to capture those effects (yet) here.

When the quadrotor revolution started to happen, I predicted that it would be followed quickly by a realization that fixed-wing vehicles are better for most applications. Propellers are almost optimally efficient for producing thrust -- making quadrotors very efficient for hovering -- but to be efficient in forward flight you probably want to have an airfoil. Wings are a really good idea! But I was wrong -- quadrotors have completely dominated fixed-wings for commercial UAVs. Perhaps it's only because they are easier to control? I suspect that as the field matures and achieving core functionality is not the primary obstacle, then people will eventually start worrying again about efficiency (a bit like we think about efficiency for automobiles).

# The Planar Quadrotor

We can get started with an extremely simple model of a quadrotor that is restricted to live in the plane. In that case, it actually only needs two propellers, but calling it a "birotor" doesn't have the same ring to it. The equations of motion are almost trivial, since it is only a single rigid body, and certainly fit into our standard manipulator equations: \\begin{gather} m \\ddot{x} = -(u\_1 + u\_2)\\sin\\theta, \\label{eq:quad\_x}\\\\ m \\ddot{y} = (u\_1 + u\_2)\\cos\\theta - mg, \\label{eq:quad\_y}\\\\ I \\ddot\\theta = r (u\_1 - u\_2) \\label{eq:quad\_theta} \\end{gather}

![](figures/quadrotor2d.svg)

The Planar Quadrotor System (which we also refer to in the code as "Quadrotor2D", to keep it next to "Quadrotor" in the directory listing). The model parameters are mass, $m$, moment of inertia, $I$, and the distance from the center to the base of the propeller, $r$.

# The Full 3D Quadrotor

The dynamics of the 3D Quadrotor follow similarly. The only complexity comes from handling the 3D rotations. The code implementing the dynamics of the [`QuadrotorPlant`](https://github.com/RobotLocomotion/drake/blob/53571e1d65d1716d8b4b72ae8d3571ee89fa36cf/examples/quadrotor/quadrotor_plant.cc#L55) example in Drake is almost as readable as a LaTeX derivation. The most interesting feature of this model that we include the _moment_ produced by the rotating propellers. Interestingly, without these moments, the system linearized about the hovering configuration is actually not controllable.

The `QuadrotorPlant` example implements the simple model. But often you may want to add extra features to the quadrotor model. For instance, you might wish to add collision dynamics to land on the ground or to bounce off obstacles, or you might want to add a pendulum to the quadrotor to balance, or a suspended payload hanging off the bottom. In these cases, rather than implement the equations by hand, it is much more convenient to use Drake's main physics engine (in `MultibodyPlant`). We just need to manually wire the [`Propeller`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_propeller.html) forces into the `Diagram` (because `Propeller` is not a concept supported in URDF nor SDF just yet). I've implemented both versions side-by-side in the notebook.

document.write(notebook\_link('acrobot','quadrotor'))

Note that by default, the orientation of each floating body in `MultibodyPlant` is represented by a unit quaternion. If you were to try to linearize the model using the quaternion representation of state, without accounting for unit norm constraint, the linearization of this model is also uncontrollable. To avoid this for now, I have manually added a "roll-pitch-yaw" floating base to the model. This keeps linearization simple but does introduce a singularity at "gimble lock". I've tried to mostly avoid dealing with 3D rotations throughout these notes, but my [notes on manipulation cover 3D rotations](https://manipulation.csail.mit.edu/pick.html#floating-base) in a bit more detail.

# Balancing

For both the Acrobot and the Cart-Pole systems, we will begin by designing a linear controller which can balance the system when it begins in the vicinity of the unstable fixed point. To accomplish this, we will linearize the nonlinear equations about the fixed point, examine the controllability of this linear system, then using [linear quadratic regulator (LQR)](lqr.html) theory to design our feedback controller.

# Linearizing the manipulator equations

Although the equations of motion of both of these model systems are relatively tractable, the forward dynamics still involve quite a few nonlinear terms that must be considered in any linearization. Let's consider the general problem of linearizing a system described by the manipulator equations.

We can perform linearization around a fixed point, $(\\bx^\*, \\bu^\*)$, using a Taylor expansion: \\begin{equation} \\dot\\bx = {\\bf f}(\\bx,\\bu) \\approx {\\bf f}(\\bx^\*,\\bu^\*) + \\left\[ \\pd{\\bf f}{\\bx}\\right\]\_{\\bx=\\bx^\*,\\bu=\\bu^\*} (\\bx - \\bx^\*) + \\left\[ \\pd{\\bf f}{\\bu}\\right\]\_{\\bx=\\bx^\*,\\bu=\\bu^\*} (\\bu - \\bu^\*) \\end{equation} Let us consider the specific problem of linearizing the manipulator equations around a (stable or unstable) fixed point. In this case, ${\\bf f}(\\bx^\*,\\bu^\*)$ is zero, and we are left with the standard linear state-space form: \\begin{align} \\dot\\bx =& \\begin{bmatrix} \\dot\\bq \\\\ \\bM^{-1}(\\bq) \\left\[ \\btau\_g(\\bq) + {\\bf B}(\\bq)\\bu - \\bC(\\bq,\\dot\\bq)\\dot\\bq \\right\] \\end{bmatrix},\\\\ \\approx& {\\bf A}\_{lin} (\\bx-\\bx^\*) + \\bB\_{lin} (\\bu - \\bu^\*), \\end{align} where ${\\bf A}\_{lin}$, and $\\bB\_{lin}$ are constant matrices. Let us define $\\bar\\bx = \\bx - \\bx^\*, \\bar\\bu = \\bu - \\bu^\*$, and write $$\\dot{\\bar\\bx} = {\\bf A}\_{lin}\\bar\\bx + \\bB\_{lin}\\bar\\bu.$$ Evaluation of the Taylor expansion around a fixed point yields the following, very simple equations, given in block form by: \\begin{align} {\\bf A}\_{lin} =& \\begin{bmatrix} {\\bf 0} & {\\bf I} \\\\ \\bM^{-1} \\pd{\\btau\_g}{\\bq} + \\sum\_{j} \\bM^{-1}\\pd{\\bB\_j}{\\bq} u\_j & {\\bf 0} \\end{bmatrix}\_{\\bx=\\bx^\*,\\bu=\\bu^\*} \\\\ \\bB\_{lin} =& \\begin{bmatrix} {\\bf 0} \\\\ \\bM^{-1} \\bB \\end{bmatrix}\_{\\bx=\\bx^\*, \\bu=\\bu^\*} \\end{align} where $\\bB\_j$ is the $j$th column of $\\bB$. Note that the term involving $\\pd{\\bM^{-1}}{q\_i}$ disappears because $\\btau\_g + \\bB\\bu - \\bC\\dot{\\bq}$ must be zero at the fixed point. All of the $\\bC\\dot\\bq$ derivatives drop out, too, because $\\dot{\\bq}^\* = 0$, and any terms with $\\bC$ drop out as well, since centripetal and centrifugal forces are zero when velocity is zero. In many cases, including both the Acrobot and Cart-Pole systems (but not the Quadrotors), the matrix $\\bB(\\bq)$ is a constant, so the $\\pd\\bB{\\bq}$ terms also drop out.

# Linearization of the Acrobot

Linearizing around the (unstable) upright point, we have: \\begin{gather} \\left\[\\pd{\\bf \\tau\_g}{\\bq}\\right\]\_{\\bx=\\bx^\*} = \\begin{bmatrix} g (m\_1 l\_{c1} + m\_2 l\_1 + m\_2 l\_{c2}) & m\_2 g l\_{c2} \\\\ m\_2 g l\_{c2} & m\_2 g l\_{c2} \\end{bmatrix} \\end{gather} The linear dynamics follow directly from these equations and the manipulator form of the Acrobot equations.

# Linearization of the Cart-Pole System

Linearizing around the unstable fixed point in this system, we have: \\begin{gather} \\left\[\\pd{\\btau\_g}{\\bq}\\right\]\_{\\bx=\\bx^\*} = \\begin{bmatrix} 0 & 0 \\\\ 0 & m\_p g l \\end{bmatrix} \\end{gather} Again, the linear dynamics follow simply.

Studying the properties of the linearized system can tell us some things about the (local) properties of the nonlinear system. For instance, having a strictly stable linearization implies local exponential stability of the nonlinear system Khalil01 (Theorem 4.15). It's worth noting that having an unstable linearization also implies that the system is locally unstable, but if the linearization is marginally stable then one cannot conclude whether the nonlinear system is asymptotically stable, stable i.s.L., or unstable (only that it is not exponentially stable)Slotine90.

# Controllability of linear systems

# Controllability

A control system of the form $$\\dot{\\bx} = {\\bf f}(\\bx,\\bu)$$ is called _controllable_ if it is possible to construct an unconstrained input signal, $\\bu(t)$, $t \\in \[0,t\_f\],$ which will move the system from any initial state to any final state in a finite interval of time, $0 < t < t\_f$ Ogata96.

For the linear system $$\\dot{\\bx} = {\\bf A}\\bx + \\bB\\bu,$$ we can integrate this linear system in closed form, so it is possible to derive the exact conditions of controllability. In particular, for linear systems it is sufficient to demonstrate that there exists a control input which drives any initial condition to the origin.

# The special case of non-repeated eigenvalues

Let us first examine a special case, which falls short as a general tool but may be more useful for understanding the intuition of controllability. Let's perform an eigenvalue analysis of the system matrix ${\\bf A}$, so that: $${\\bf A}{\\bf v}\_i = \\lambda\_i {\\bf v}\_i,$$ where $\\lambda\_i$ is the $i$th eigenvalue, and ${\\bf v}\_i$ is the corresponding (right) eigenvector. There will be $n$ eigenvalues for the $n \\times n$ matrix ${\\bf A}$. Collecting the (column) eigenvectors into the matrix ${\\bf V}$ and the eigenvalues into a diagonal matrix ${\\bf \\Lambda}$, we have $${\\bf A}{\\bf V} = {\\bf V}{\\bf \\Lambda}.$$ Here comes our primary assumption: let us assume that each of these $n$ eigenvalues takes on a distinct value (no repeats). With this assumption, it can be shown that the eigenvectors ${\\bf v}\_i$ form a linearly independent basis set, and therefore ${\\bf V}^{-1}$ is well-defined.

We can continue our eigenmodal analysis of the linear system by defining the modal coordinates, ${\\bf r}$, with: $$\\bx = {\\bf V}{\\bf r},\\quad \\text{or}\\quad {\\bf r} = {\\bf V}^{-1}\\bx.$$ In modal coordinates, the dynamics of the linear system are given by $$\\dot{\\bf r} = {\\bf V}^{-1} {\\bf A} {\\bf V} {\\bf r} + {\\bf V}^{-1} \\bB \\bu = {\\bf \\Lambda} {\\bf r} + {\\bf V}^{-1}\\bB \\bu.$$ This illustrates the power of modal analysis; in modal coordinates, the dynamics diagonalize yielding: $$\\dot{r}\_i = \\lambda\_i r\_i + \\sum\_j \\beta\_{ij} u\_j,\\quad {\\bf \\beta} = {\\bf V}^{-1} \\bB.$$

Now the concept of controllability becomes clear. Input $j$ can influence the dynamics in modal coordinate $i$ if and only if ${\\bf \\beta}\_{ij} \\neq 0$. In the special case of non-repeated eigenvalues, having control over each individual eigenmode is sufficient to (in finite time) regulate all of the eigenmodesOgata96. Therefore, we say that the system is controllable if and only if $$\\forall i, \\exists j \\text{ such that }\\beta\_{ij} \\neq 0.$$

# A general solution

Included only for completeness. Click to expand the details.

A more general solution to the controllability issue, which removes our assumption about the eigenvalues, can be obtained by examining the time-domain solution of the linear equations. The solution of this system is $$\\bx(t) = e^{{\\bf A}t} \\bx(0) + \\int\_0^{t} e^{{\\bf A}(t - \\tau)} \\bB \\bu(\\tau) d\\tau.$$ Without loss of generality, lets consider the that the final state of the system is zero. Then we have: $$\\bx(0) = - \\int\_0^{t\_f} e^{-{\\bf A}\\tau}\\bB \\bu(\\tau) d\\tau.$$ You might be wondering what we mean by $e^{{\\bf A}t}$; a scalar raised to the power of a matrix..? Recall that $e^{z}$ is actually defined by a convergent infinite sum: $$e^{z} =1 + z + \\frac{1}{2} z^2 + \\frac{1}{6} z^3 + ... .$$ The notation $e^{{\\bf A}t}$ uses the same definition: $$e^{{\\bf A}t} = {\\bf I} + {\\bf A}t + \\frac{1}{2}({\\bf A}t)^2 + \\frac{1}{6}({\\bf A}t)^3 + ... .$$ Not surprisingly, this has many special forms. For instance, if ${\\bf A}$ is diagonalizable, $e^{{\\bf A}t} = {\\bf V}e^{{\\bf\\Lambda}t}{\\bf V}^{-1},$ where ${\\bf A} = {\\bf V \\Lambda V}^{-1}$ is the eigenvalue decomposition of ${\\bf A}$ Strang98. The particular form we will use here is $$e^{-{\\bf A}\\tau} = \\sum\_{k=0}^{n-1} \\alpha\_k(\\tau) {\\bf A}^k.$$ This is a particularly surprising form, because the infinite sum above is represented by this finite sum; the derivation uses Sylvester's Theorem Ogata96, Chen98a. Then we have, \\begin{align\*} \\bx(0) =& - \\sum\_{k=0}^{n-1} {\\bf A}^k \\bB \\int\_0^{t\_f} \\alpha\_k(\\tau) \\bu(\\tau) d\\tau \\\\ =& -\\sum\_{k=0}^{n-1} {\\bf A}^k \\bB w\_k \\text{, where } \\bw\_k = \\int\_0^{t\_f} \\alpha\_k(\\tau) \\bu(\\tau) d\\tau \\\\ =& - \\begin{bmatrix} \\bB & {\\bf AB} & {\\bf A}^2\\bB & \\cdots & {\\bf A}^{n-1}\\bB \\end{bmatrix}\_{n \\times (nm)} \\begin{bmatrix} \\bw\_0 \\\\ \\bw\_1 \\\\ \\bw\_2 \\\\ \\vdots \\\\ \\bw\_{n-1} \\end{bmatrix} \\end{align\*} The matrix containing the vectors $\\bB$, ${\\bf AB}$, ... ${\\bf A}^{n-1}\\bB$ is called the controllability matrix. In order for the system to be controllable, for every initial condition $\\bx(0)$ we must be able to find the corresponding vector ${\\bf w}$. This is only possible when the rows of the controllability matrix are linearly independent. Therefore, the condition of controllability is that this controllability matrix is full rank. In you can obtain the controllability matrix for a linear system using [`ControllabilityMatrix`](https://drake.mit.edu/doxygen_cxx/group__control__systems.html#gaedff9f0a8ddce5d29888ed611ce8bee2), or check the rank condition directly using [`IsControllable`](https://drake.mit.edu/doxygen_cxx/group__control__systems.html#ga8bc6169b50b1b127ed5f1b70afcb64ca).

Note that a linear feedback to change the eigenvalues of the eigenmodes is not sufficient to accomplish our goal of getting to the goal in finite time. In fact, the open-loop control to reach the goal is easily obtained with a final-value LQR problem, and (for ${\\bf R}={\\bf I}$) is actually a simple function of the controllability GrammianChen98a.

# Controllability vs. underactuated

Analysis of the controllability of both the Acrobot and Cart-Pole systems reveals that the linearized dynamics about the upright are, in fact, controllable. This implies that the linearized system, if started away from the zero state, can be returned to the zero state in finite time. This is potentially surprising - after all the systems are underactuated. For example, it is interesting and surprising that the Acrobot can balance itself in the upright position without having a shoulder motor.

The controllability of these model systems demonstrates an extremely important, point: _An underactuated system is not necessarily an uncontrollable system._ Underactuated systems cannot follow arbitrary trajectories, but that does not imply that they cannot arrive at arbitrary points in state space. However, the trajectory required to place the system into a particular state may be arbitrarily complex.

The controllability analysis presented here is for linear time-invariant (LTI) systems. A comparable analysis exists for linear time-varying (LTV) systems. We will even see extensions to nonlinear systems; although it will often be referred to by the synonym of "reachability" analysis.

# Stabilizability of a linear system

Closely related to controllability is the notion of _stabilizability_. For linear systems, stabilizability is a strictly weaker condition than controllability (a system can be stabilizable but not controllable, but not the other way around).

# Stabilizability of a linear system

A control system of the form $$\\dot{\\bx} = \\bA\\bx + \\bB \\bu$$ is called _stabilizable_ if it is possible to construct an unconstrained input signal, $\\bu(t)$, $t \\in \[0,\\infty\],$ which results in $$\\lim\_{t \\rightarrow \\infty} \\bx(t) = 0.$$

Controllability requires that we arrive at the origin in a finite time, stabilizability allows for asympotitic convergence. Essentially, a system can still be stabilizable if the uncontrollable subspace is naturally stable.

Interestingly, for nonlinear systems the relationship between stabilizability and controllability is much more subtle. In a famous (sometimes misquoted) result from Roger Brockett states that nonlinear controllability does not necessarily imply stabilizability by differentiable control policiesBrockett83.

# LQR feedback

Controllability and stabilizability tell us that a trajectory to the fixed point exists, but does not tell us which one we should take or what control inputs cause it to occur. Why not? There are potentially infinitely many solutions. We have to pick one.

The tools for controller design in linear systems are very advanced. In particular, as we describe in the [linear optimal control chapter](lqr.html), one can easily design an optimal feedback controller for a regulation task like balancing, so long as we are willing to linearize the system around the operating point and define optimality in terms of a quadratic cost function: $$J(\\bx\_0) = \\int\_0^\\infty \\left\[ \\bx^T(t) \\bQ \\bx(t) + \\bu^T(t) \\bR \\bu(t) \\right\]dt, \\quad \\bx(0)=\\bx\_0, \\bQ=\\bQ^T>0, \\bR=\\bR^T>0.$$ The linear feedback matrix $\\bK$ used as $$\\bu(t) = - \\bK\\bx(t),$$ is the so-called optimal linear quadratic regulator (LQR). Even without understanding the detailed derivation, we can quickly become practitioners of LQR. provides a function,

> `K = LinearQuadraticRegulator(A, B, Q, R)`

for linear systems. It also provides a version

> `controller = LinearQuadraticRegulator(system, context, Q, R)`

that will linearize the system for you around an equilibrium and return the controller (in the original coordinates). Therefore, to use LQR, one simply needs to define the symmetric positive-definite cost matrices, ${\\bf Q}$ and ${\\bf R}$. In their most common form, ${\\bf Q}$ and ${\\bf R}$ are positive diagonal matrices, where the entries $Q\_{ii}$ penalize the relative errors in state variable $x\_i$ compared to the other state variables, and the entries $R\_{ii}$ penalize actions in $u\_i$.

Take a moment to appreciate this. If the linearized system is stabilizable, then for any (positive semi-definite) $\\bQ$ and (positive definite) $\\bR$ matrices, LQR will give us a stabilizing controller. If the system is not stabilizable, then LQR will tell us that no linear controller exists. Pretty amazing!

# LQR for the Acrobot and Cart-Pole

Take a minute to play around with the LQR controller for the Acrobot and the Cart-Pole

document.write(notebook\_link('acrobot','acrobot')) document.write(notebook\_link('acrobot','cartpole'))

Make sure that you take a minute to look at the code which runs during these examples. Can you set the ${\\bf Q}$ and ${\\bf R}$ matrices differently, to improve the performance?

Simulation of the closed-loop response with LQR feedback shows that the task is indeed completed - and in an impressive manner. Oftentimes the state of the system has to move violently away from the origin in order to ultimately reach the origin. Further inspection reveals the (linearized) closed-loop dynamics are in fact non-minimum phase (acrobot has 3 right-half zeros, cart-pole has 1).

# LQR for Quadrotors

LQR works essentially out of the box for Quadrotors, if linearized around a nominal fixed point (where the non-zero thrust from the propellers is balancing gravity).

document.write(notebook\_link('acrobot', 'planar\_quadrotor')) document.write(notebook\_link('acrobot', 'quadrotor'))

or [Click here for the animation](data/quadrotor2d_lqr.html).

Note that LQR, although it is optimal for the linearized system, is not necessarily the best linear control solution for maximizing basin of attraction of the fixed-point. The theory of _robust control_(e.g., Zhou97), which explicitly takes into account the differences between the linearized model and the nonlinear model, will produce controllers which outperform our LQR solution in this regard.

# Partial feedback linearization

In the introductory chapters, we made the point that the underactuated systems are not feedback equivalent to $\\ddot{q} = \\bu$. Although we cannot always simplify the full dynamics of the system, it is still possible to linearize a portion of the system dynamics. The technique is called _partial feedback linearization_.

Consider the cart-pole example. The dynamics of the cart are affected by the motions of the pendulum. If we know the model, then it seems quite reasonable to think that we could create a feedback controller which would push the cart in exactly the way necessary to counter-act the dynamic contributions from the pendulum - thereby linearizing the cart dynamics. What we will see, which is potentially more surprising, is that we can also use a feedback controller for the cart to feedback linearize the dynamics of the passive pendulum joint.

We'll use the term _collocated_ partial feedback linearization to describe a controller which linearizes the dynamics of the actuated joints. What's more surprising is that it is often possible to achieve _non-collocated_ partial feedback linearization - a controller which linearizes the dynamics of the unactuated joints. The treatment presented here follows from Spong94a.

# PFL for the Cart-Pole System

# Collocated

Starting from the equations \\ref{eq:simple} and \\ref{eq:simple2}, we have \\begin{gather\*} \\ddot\\theta = -\\ddot{x}c - s \\\\ % \\ddot\\theta = -\\frac{1}{l} (\\ddot{x} c + g s) \\ddot{x}(2-c^2) - sc - \\dot\\theta^2 s = f\_x \\end{gather\*} Therefore, applying the feedback control \\begin{equation} f\_x = (2 - c^2) \\ddot{x}^d - sc - \\dot\\theta^2 s % f = (m\_c + m\_p) u + m\_p (u c + g s) c - m\_p l \\dot\\theta^2 s \\end{equation} results in \\begin{align\*} \\ddot{x} =& \\ddot{x}^d \\\\ \\ddot{\\theta} =& -\\ddot{x}^dc - s, \\end{align\*} which are valid globally.

Look carefully at the resulting equations. Of course, it says that we can impose whatever accelerations we like on the cart. But even the resulting equations of the pole happen to take a nice form here: they have been reduced to the equations of the simple pendulum (without a cart), where the torque input is now given instead by $\\ddot{x}c$. It's as if we have a simple pendulum with torque control, except our command is modulated by a $\\cos\\theta$ term, and this $\\cos\\theta$ term is fundamental -- it's true that our control authority goes to zero when the pole is horizontal, because no amount of force on the cart in that configuration can act like a torque on the pole.

# Non-collocated

Starting again from equations \\ref{eq:simple} and \\ref{eq:simple2}, we have \\begin{gather\*} \\ddot{x} = -\\frac{\\ddot\\theta + s}{c} \\\\ \\ddot\\theta(c - \\frac{2}{c}) - 2 \\tan\\theta - \\dot\\theta^2 s = f\_x \\end{gather\*} Applying the feedback control \\begin{equation} f\_x = (c - \\frac{2}{c}) \\ddot\\theta^d - 2 \\tan\\theta - \\dot\\theta^2 s \\end{equation} results in \\begin{align\*} \\ddot\\theta =& \\ddot\\theta^d \\\\ \\ddot{x} =& -\\frac{1}{c} \\ddot\\theta^d - \\tan\\theta. \\end{align\*} Note that this expression is only valid when $\\cos\\theta \\neq 0$. Once again, we know that the force cannot create a torque when the pole is perfectly horizontal. In fact, the controller we have written will "blow-up" -- requesting infinite force at $\\cos\\theta = 0$; so make sure you saturate the command before you implement it on hardware (or even in simulation). Although it may come as a small consolation, at least we have that $(c - \\frac{2}{c})$ never goes to zero; in fact you can check for yourself that $|c - \\frac{2}{c}| \\ge 1$.

# General form

For systems that are trivially underactuated (torques on some joints, no torques on other joints), we can, without loss of generality, reorganize the joint coordinates in any underactuated system described by the manipulator equations into the form: \\begin{align} \\bM\_{11} \\ddot{\\bq}\_1 + \\bM\_{12} \\ddot{\\bq}\_2 &= \\btau\_1, \\label{eq:passive\_dyn}\\\\ \\bM\_{21} \\ddot{\\bq}\_1 + \\bM\_{22} \\ddot{\\bq}\_2 &= \\btau\_2 + \\bu, \\label{eq:active\_dyn} \\end{align} with $\\bq \\in \\Re^n$, $\\bq\_1 \\in \\Re^l$, $\\bq\_2 \\in \\Re^m$, $l=n-m$. $\\bq\_1$ represents all of the passive joints, and $\\bq\_2$ represents all of the actuated joints, and the $\\btau = \\btau\_g - \\bC\\dot\\bq$ terms capture all of the Coriolis and gravitational terms, and $$\\bM(\\bq) = \\begin{bmatrix} \\bM\_{11} & \\bM\_{12} \\\\ \\bM\_{21} & \\bM\_{22} \\end{bmatrix}.$$ Fortunately, because $\\bM$ is uniformly (e.g. for all $\\bq$) positive definite, $\\bM\_{11}$ and $\\bM\_{22}$ are also positive definite, by the [Schur complement condition for positive definiteness](https://en.wikipedia.org/w/index.php?title=Schur_complement).

# Collocated linearization

Performing the same substitutions into the full manipulator equations, we get: \\begin{gather} \\ddot\\bq\_1 = \\bM\_{11}^{-1} \\left\[ \\btau\_1 - \\bM\_{12} \\ddot\\bq\_2 \\right\] \\\\ (\\bM\_{22} - \\bM\_{21} \\bM\_{11}^{-1} \\bM\_{12}) \\ddot\\bq\_2 - \\btau\_2 + \\bM\_{21} \\bM\_{11}^{-1} \\btau\_1 = \\bu \\end{gather} It can be easily shown that the matrix $(\\bM\_{22} - \\bM\_{21} \\bM\_{11}^{-1} \\bM\_{12})$ is invertibleSpong94a; we can see from inspection that it is symmetric. PFL follows naturally, and is valid globally.

# Non-collocated linearization

\\begin{gather} \\ddot\\bq\_2 = \\bM\_{12}^+ \\left\[ \\btau\_1 - \\bM\_{11} \\ddot\\bq\_1 \\right\] \\\\ (\\bM\_{21} - \\bM\_{22} \\bM\_{12}^+ \\bM\_{11}) \\ddot\\bq\_1 - \\btau\_2 + \\bM\_{22} \\bM\_{12}^+ \\btau\_1 = \\bu \\end{gather} where $\\bM\_{12}^+$ is a Moore-Penrose pseudo-inverse. This inverse provides a unique solution when the rank of $\\bM\_{12}$ equals $l$, the number of passive degrees of freedom in the system (it cannot be more, since the matrix only has $l$ rows). The rank condition in this context is sometimes called the property of "Strong Inertial Coupling". It is state dependent; in the cart-pole example above $\\bM\_{12} = \\cos\\theta$ and drops rank exactly when $\\cos\\theta = 0$. A system has Global Strong Inertial Coupling if it exhibits Strong Inertial Coupling in every state.

# Task-space partial feedback linearization

In general, we can define some combination of active and passive joints that we would like to control. This combination is sometimes called a "task space". cite some task space refs here? Consider an output function of the form, $$\\by = \\bh(\\bq),$$ with ${\\bf y} \\in \\Re^p$, which defines the task space. Define $\\bH\_1 = \\frac{\\partial \\bh}{\\partial \\bq\_1}$, $\\bH\_2 = \\frac{\\partial \\bh}{\\partial \\bq\_2}$, $\\bH = \[\\bH\_1,\\bH\_2\]$.

# Task Space PFL

If the actuated joints are commanded so that \\begin{equation} \\ddot\\bq\_2 = \\bar\\bH^+ \\left \[\\ddot\\by^d - \\dot\\bH\\dot\\bq - \\bH\_1 \\bM\_{11}^{-1}\\btau\_1 \\right\], \\label{eq:q2cmd} \\end{equation} where $\\bar{\\bH} = \\bH\_2 - \\bH\_1 \\bM\_{11}^{-1} \\bM\_{12}.$ and $\\bar\\bH^+$ is the right Moore-Penrose pseudo-inverse, $$\\bar\\bH^+ = \\bar\\bH^T (\\bar\\bH \\bar\\bH^T)^{-1},$$ then we have \\begin{equation} \\ddot\\by = \\ddot\\by^d.\\end{equation} subject to \\begin{equation}\\text{rank}\\left(\\bar{\\bH} \\right) = p, \\label{eq:rank\_condition}\\end{equation}

**Proof Sketch.** Differentiating the output function we have \\begin{gather\*} \\dot\\by = \\bH \\dot\\bq \\\\ \\ddot\\by = \\dot\\bH \\dot\\bq + \\bH\_1 \\ddot\\bq\_1 + \\bH\_2 \\ddot\\bq\_2. \\end{gather\*} Solving \\ref{eq:passive\_dyn} for the dynamics of the unactuated joints we have: \\begin{equation} \\ddot\\bq\_1 = \\bM\_{11}^{-1} (\\btau\_1 - \\bM\_{12} \\ddot\\bq\_2) \\label{eq:q1cmd} \\end{equation} Substituting, we have \\begin{align} \\ddot\\by =& \\dot\\bH \\dot\\bq + \\bH\_1 \\bM\_{11}^{-1}(\\btau\_1 - \\bM\_{12}\\ddot\\bq\_2) + \\bH\_2 \\ddot\\bq\_2 \\\\ =& \\dot\\bH \\dot\\bq + \\bar{\\bH} \\ddot\\bq\_2 + \\bH\_1 \\bM\_{11}^{-1}\\btau\_1 \\\\ =& \\ddot\\by^d \\end{align} Note that the last line required the rank condition ($\\ref{eq:rank\_condition}$) on $\\bar\\bH$ to ensure that the rows of $\\bar{\\bH}$ are linearly independent, allowing $\\bar\\bH \\bar\\bH^+ = \\bI$.

In order to execute a task space trajectory one could command $$\\ddot\\by^d = \\bK\_d (\\dot{\\by}^d - \\dot\\by) + \\bK\_p (\\by^d -\\by).$$ Assuming the internal dynamics are stable, this yields converging error dynamics when $\\bK\_p,\\bK\_d > 0$Slotine90, which implies $y(t) \\rightarrow y^d(t)$. For a position control robot, the acceleration command of ($\\ref{eq:q2cmd}$) suffices. Alternatively, a torque command follows by substituting ($\\ref{eq:q2cmd}$) and ($\\ref{eq:q1cmd}$) into ($\\ref{eq:active\_dyn}$).

# End-point trajectory following with the Cart-Pole system

Consider the task of trying to track a desired kinematic trajectory with the endpoint of pendulum in the cart-pole system. With one actuator and kinematic constraints, we might be hard-pressed to track a trajectory in both the horizontal and vertical coordinates. But we can at least try to track a trajectory in the vertical position of the end-effector.

Using the task-space PFL derivation, we have: \\begin{gather\*} y = h(\\bq) = -l \\cos\\theta \\\\ \\dot{y} = l \\dot\\theta \\sin\\theta \\end{gather\*} If we define a desired trajectory: $$y^d(t) = \\frac{l}{2} + \\frac{l}{4} \\sin(t),$$ then the task-space controller is easily implemented using the derivation above.

add results here

The task space derivation above provides a convenient generalization of the partial feedback linearization (PFL) Spong94a, which encompasses both the collocated and non-collocated results. If we choose $\\by = \\bq\_2$ (collocated), then we have $$\\bH\_1 = 0, \\bH\_2 = \\bI, \\dot\\bH = 0, \\bar\\bH = \\bI, \\bar\\bH^+ = \\bI.$$ From this, the command in ($\\ref{eq:q2cmd}$) reduces to $\\ddot\\bq\_2 = \\ddot\\bq\_2^d$. The actuator command is then $$ \\bu = \\bM\_{21} \\bM\_{11}^{-1} (\\btau\_1 - \\bM\_{12} \\ddot\\bq\_2^d) + \\bM\_{22} \\ddot\\bq\_2^d - \\btau\_2,$$ and the rank condition ($\\ref{eq:rank\_condition}$) is always met.

If we choose ${\\bf y} = \\bq\_1$ (non-collocated), we have $$\\bH\_1 = \\bI, \\bH\_2 = 0, \\dot\\bH = 0, \\bar{\\bH}=-\\bM\_{11}^{-1}\\bM\_{12}.$$ The rank condition ($\\ref{eq:rank\_condition}$) requires that $\\text{rank}(\\bM\_{12}) = l$, in which case we can write $\\bar{\\bH}^+=-\\bM\_{12}^+\\bM\_{11}$, reducing the rank condition to precisely the "Strong Inertial Coupling" condition described in Spong94a. Now the command in ($\\ref{eq:q2cmd}$) reduces to \\begin{equation} \\ddot\\bq\_2 = \\bM\_{12}^+ \\left \[ \\btau\_1 - \\bM\_{11}\\ddot\\bq\_1^d \\right\] \\label{eq:q2ddnonco} \\end{equation} The actuator command is found by substituting ($\\ref{eq:q2ddnonco}$) into ($\\ref{eq:active\_dyn}$), yielding the same results as in Spong94a.

# Swing-up control

# Energy shaping

Recall in the last chapter, [we used energy shaping to swing up the simple pendulum](pend.html#energy_shaping). This idea turns out to be a bit more general than just for the simple pendulum. As we will see, we can use similar concepts of "energy shaping" to produce swing-up controllers for the acrobot and cart-pole systems. It's important to note that it only takes one actuator to change the total energy of a system.

Although a large variety of swing-up controllers have been proposed for these model systems (c.f. Fantoni02, Araki05, Xin04, Spong94, Mahindrakar05, Berkemeier99, Murray91, Lai06), the energy shaping controllers tend to be the most natural to derive and perhaps the most well-known.

# Cart-Pole

Let's try to apply the energy-shaping ideas to the cart-pole system. The basic idea, from Chung95, is to use collocated PFL to simplify the dynamics, use energy shaping to regulate the pendulum to its homoclinic orbit, then to add a few terms to make sure that the cart stays near the origin. This is a bit surprising... if we want to control the pendulum, shouldn't we use the non-collocated version? Actually, we ultimately want to control both the cart and the pole, and we will build on the collocated version both because it avoids inverting the $\\cos\\theta$ term that can go to zero and because (when all parameters are set to 1) we were left with a particularly simple form of the equations: \\begin{gather} \\ddot{x} = u \\\\ \\ddot\\theta = - u c - s.\\end{gather} The first equation is clearly simple, but even the second equation is exactly the equations of a _decoupled_ pendulum, just with a slightly odd control input that is modulated by $\\cos\\theta$.

Let us regulate the energy of this decoupled pendulum using energy shaping. The energy of the pendulum (a unit mass, unit length, simple pendulum in unit gravity) is given by: $$E(\\bx) = \\frac{1}{2} \\dot\\theta^2 - \\cos\\theta.$$ The desired energy, equivalent to the energy at the desired fixed-point, is $$E^d = 1.$$ Again defining $\\tilde{E}(\\bx) = E(\\bx) - E^d$, we now observe that \\begin{align\*} \\dot{\\tilde{E}}(\\bx) =& \\dot{E}(\\bx) = \\dot\\theta \\ddot\\theta + \\dot\\theta s \\\\ % \\dot\\tilde{E} = ml^2 \\dot\\theta \\ddot\\theta + mgl s =& \\dot\\theta \[ -uc - s\] + \\dot\\theta s \\\\ % - ml \\dot\\theta \[ u c + g s \] + mgl s =& - u\\dot\\theta \\cos\\theta. % - ml u \\dot\\theta c \\end{align\*}

Therefore, if we design a controller of the form $$u = k \\dot\\theta\\cos\\theta \\tilde{E},\\quad k>0$$ the result is $$\\dot{\\tilde{E}} = - k \\dot\\theta^2 \\cos^2\\theta \\tilde{E}.$$ This guarantees that $| \\tilde{E} |$ is non-increasing, but isn't quite enough to guarantee that it will go to zero. For example, if $\\theta = \\dot\\theta = 0$, the system will never move. However, if we have that $$\\int\_0^t \\dot\\theta^2(t') \\cos^2 \\theta(t') dt' \\rightarrow \\infty,\\quad \\text{as } t \\rightarrow \\infty,$$ then we have $\\tilde{E}(t) \\rightarrow 0$. This condition, a version of the LaSalle's theorem that we will develop in our notes on Lyapunov methods, is satisfied for all but the trivial constant trajectories at fixed points.

Now we return to the full system dynamics (which includes the cart). Chung95 and Spong96 use the simple pendulum energy controller with an addition PD controller designed to regulate the cart: $$\\ddot{x}^d = k\_E \\dot\\theta \\cos\\theta \\tilde{E} - k\_p x - k\_d \\dot{x}.$$ Chung95 provides a proof of convergence for this controller with some nominal parameters. Fantoni02 provides a slightly different controller derived directly from a Lyapunov argument.

![](figures/cartpole_swingup_phase.svg)

Cart-Pole Swingup: Example phase plot of the pendulum subsystem using energy shaping control. The controller drives the system to the homoclinic orbit, then switches to an LQR balancing controller near the top.

# Acrobot

Swing-up control for the acrobot can be accomplished in much the same way. Spong94 - pump energy. Clean and simple. No proof. Slightly modified version (uses arctan instead of sat) in Spong95. Clearest presentation in Spong96.

Use collocated PFL. ($\\ddot{q}\_2 = \\ddot{q}\_2^d$). $$E(\\bx) = \\frac{1}{2}\\dot\\bq^T\\bM\\dot{\\bq} + U(\\bx).$$ $$ E\_d = U(\\bx^\*).$$ $$\\bar{u} = \\dot{q}\_1 \\tilde{E}.$$ $$\\ddot{q}\_2^d = - k\_1 q\_2 - k\_2 \\dot{q}\_2 + k\_3 \\bar{u},$$

Extra PD terms prevent proof of asymptotic convergence to homoclinic orbit. Proof of another energy-based controller in Xin04.

# Discussion

The energy shaping controller for swing-up presented here is a pretty faithful representative from the field of nonlinear underactuated control. Typically these control derivations require some clever tricks for simplifying or canceling out terms in the nonlinear equations, then some clever Lyapunov function to prove stability. In these cases, PFL was used to simplify the equations, and therefore the controller design.

We will see another nice example of changing coordinates in the nonlinear equations to make the problem easier when we study "[differential flatness](trajopt.html#differential_flatness)" for trajectory optimization. This approach is perhaps most famous these days for very dynamic trajectory planning with quadrotors.

These controllers are important, representative, and relevant. But clever tricks with nonlinear equations seem to be fundamentally limited. Most of the rest of the material presented in this book will emphasize more general computational approaches to formulating and solving these and other control problems.

# Other model systems

The acrobot and cart-pole systems are just two of the model systems used heavily in underactuated control research. Other examples include:

*   Pendubot
*   Inertia wheel pendulum
*   Furuta pendulum (horizontal rotation and vertical pendulum)
*   Hovercraft

# Exercises

# Cart-Pole: Linearization and Balancing

For this exercise you will work exclusively in document.write(notebook\_link('acrobot', 'cartpole\_balancing', link\_text='this notebook')). You will be asked to complete the following steps.

1.  Derive the state-space dynamics $\\dot{\\bx} = f(\\bx, \\bu)$ of [the cart-pole system](#cart_pole).
2.  Linearize the dynamics from point (a) around the unstable equilibrium point.
3.  Analyze the linearization error for different values of the state $\\bx$ and the control $\\bu$.
4.  Among the states from point (c), identify which ones are stabilized to the origin by the LQR controller.

# Cart-Poles: Writing URDFs and balancing with LQR

For this exercise you will work in document.write(notebook\_link('acrobot', 'cartpoles\_urdf', link\_text='this notebook')). You will complete the following steps.

1.  Construct the cart-pole pendulum URDF structure for the [single pendulum cart-pole.](#cart_pole).
2.  Extend the single pendulum cart-pole to a double pendulum cart-pole by modifying the URDF model of the robot and testing LQR's ability to control it.

# Controllability of Discrete and Continuous LTI Systems

1.  (2-D grid with discrete double integrator) Consider a discretized grid world where $$\\bx = \\begin{bmatrix}x\_1 \\\\ x\_2\\end{bmatrix}, \\text{ and } x\_1\[n\], x\_2\[n\] \\in \\mathbb{Z}.$$ The dynamics of $x\[n\]$ is a discrete-time version of double integrator, $$ \\bx\[n+1\] = \\begin{bmatrix} x\_1\[n\] + x\_2\[n\] \\\\ x\_2\[n\] + u\[n\] \\end{bmatrix},$$ where the control input $u\[n\] \\in \\mathbb{Z}$ is also an integer.
    1.  Suppose we represent the discretized double integrator dynamics on the following graph, where the nodes $s\_{i, j}$ represent state vector values ($\\bx = s\_{i, j}$ means $x\_1 = i, x\_2 = j$) and edges are state transitions defined by a control law. In the following truncated graph, draw the edges when $u = -1$, $u=0$, and $u=1$. (Please draw edges for all the nine nodes in the truncated graph. If a node trainsitions to another node outside the graph, please draw the destination node as well.)
        
        ![](figures/exercises/2d_grid_double_integrator.svg)
        
    2.  Suppose we start from $s\_{0, 0}$, construct a control input sequence ($u\[n\]$ could be a function of time step and can take any arbitrary intger values) to reach an arbitrary state $s\_{i, j}$ that satisfies $j > i > 0$. What would be the minimal number of time steps needed to reach any $s\_{i,j}$ that satisfies $j > i > 0$? Show that the number you've chosen is indeed minimum.
    3.  Now consider a more general case, from an arbitrary initial state $s\_{i\_1, j\_1}$, can you find a control input sequence $u\[n\]$ with the same number of time steps as in (2) that allows the state to reach any final state $s\_{i\_2, j\_2}$? Also show that the number you've chosen is minimum.
2.  1.  Considering linear systems $$\\dot{\\bx} = {\\bf A} \\bx + {\\bf B} \\bu,$$ are the following linear systems controllable? \\begin{align\*} {\\bf A}\_1 = \\begin{bmatrix} 1 & 0 \\\\ 0 & 1 \\end{bmatrix}, {\\bf B}\_1 = \\begin{bmatrix} 0 \\\\ 1 \\end{bmatrix};\\qquad {\\bf A}\_2 = \\begin{bmatrix} 1 & 0 \\\\ 0 & 1 \\end{bmatrix}, {\\bf B}\_2 = \\begin{bmatrix} 1\\\\ 1 \\end{bmatrix}; \\\\ \\\\ {\\bf A}\_3 = \\begin{bmatrix} 0 & 1 \\\\ 0 & 0 \\end{bmatrix}, {\\bf B}\_3 = \\begin{bmatrix} 0 \\\\ 1 \\end{bmatrix};\\qquad {\\bf A}\_4 = \\begin{bmatrix} 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \\\\ 0 & 1 & 0 & 0 \\\\ 0 & 2 & 0 & 0 \\end{bmatrix}, {\\bf B}\_4 = \\begin{bmatrix} 0 \\\\ 0 \\\\ 1 \\\\ 1 \\end{bmatrix}. \\end{align\*}
        
        You should only need the conditions described in [the definition of controllability](#controllability_def) to explain your conclusion. You can also use the more general tools found in [this collapsable section](#controllability_matrix); these will prove especially for the $({\\bf A}\_4, {\\bf B}\_4)$ matrix pair.
        
    2.  Consider the two systems, $({\\bf A}\_3, {\\bf B}\_3)$ and $({\\bf A}\_4, {\\bf B}\_4)$. Can you make a conclusion whether these two systems are underactuated? (Note that the definition of underactuation requires the system to be interpreted as a second-order system, i.e., $\\bx = \[\\bq, \\dot{\\bq}\]^T$)
        

# Comparison between stability of nonlinear system and linearization

In this problem, we use phase portraits to examine how linearization technique helps with local stability analysis for nonlinear systems and its limitations using the nonlinear pendulum with zero torque as an example. The pendulum parameters are chosen as $m = 1$, $l = 1$, $g = 9.81$. Follow the exercises below and complete the graphical analysis for equilibrium points $\\bx^\* = \[0, 0\]^T$, $\\bx^\* = \[\\pi, 0\]^T$, which will guide you to studying the systems. When you are done, populate this table:

|     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- |
|     | sign$(Re(\\lambda\_1))$ | sign$(Re(\\lambda\_2))$ | stable i.s.L. | Asymp. Stable | Exp. Stable |
| Nonlinear  <br>$b = 0$, $\\bx^\* = \[0, 0\]^T$ | N/A | N/A |     |     |     |
| Nonlinear  <br>$b = 0$, $\\bx^\* = \[\\pi, 0\]^T$ | N/A | N/A |     |     |     |
| Linearization  <br>$b = 0$, $\\bx^\* = \[0, 0\]^T$ |     |     |     |     |     |
| Linearization  <br>$b = 0$, $\\bx^\* = \[\\pi, 0\]^T$ |     |     |     |     |     |
| Nonlinear  <br>$b = 1$, $\\bx^\* = \[0, 0\]^T$ | N/A | N/A |     |     |     |
| Nonlinear  <br>$b = 1$, $\\bx^\* = \[\\pi, 0\]^T$ | N/A | N/A |     |     |     |
| Linearization  <br>$b = 1$, $\\bx^\* = \[0, 0\]^T$ |     |     |     |     |     |
| Linearization  <br>$b = 1$, $\\bx^\* = \[\\pi, 0\]^T$ |     |     |     |     |     |

1.  Let's first look at an undamped pendulum $b = 0$.
    1.  Using document.write(notebook\_link('pend', 'attractivity\_vs\_stability', link\_text='this notebook')) as an example, draw the phase portrait using `plot_2d_phase_portrait` function around equilibrium $\\bx^\*$. Is the nonlinear undamped pendulum stable i.s.L.? asymptotically stable? exponentially stable?
    2.  Derive the linearization of the undamped nonlinear pendulum around equilibrium $\\bx^\*$. Compute the eigenvalues of the Jacobian $\\left\[\\frac{\\partial {\\bf f}}{\\partial \\bx} \\right\]\_{\\bx = \\bx^\*}$, what are the signs of the real parts of these two eigenvalues, $\\text{sign}(Re(\\lambda\_1))$, $\\text{sign}(Re(\\lambda\_2))$?
    3.  Using phase portrait, is the linearized system $\\dot{\\bx} = \\left\[ \\frac{\\partial {\\bf f}}{\\partial \\bx} \\right\]\_{\\bx = \\bx^\*}(\\bx - \\bx^\*)$ stable i.s.L.? exponentially stable?
2.  Now let's consider a pendulum with damping coefficient $b = 1$.
    1.  Using document.write(notebook\_link('pend','attractivity\_vs\_stability', link\_text='this notebook')) as an example, draw the phase portrait using `plot_2d_phase_portrait` function around equilibrium $\\bx^\*$. Is the nonlinear damped pendulum stable i.s.L.? asymptotically stable? exponentially stable?
    2.  Derive the linearization of the damped nonlinear pendulum around equilibrium $\\bx^\*$. Compute the eigenvalues of the Jacobian $\\left\[\\frac{\\partial {\\bf f}}{\\partial \\bx} \\right\]\_{\\bx = \\bx^\*}$, what are the signs of the real parts of these two eigenvalues, $\\text{sign}(Re(\\lambda\_1))$, $\\text{sign}(Re(\\lambda\_2))$?
    3.  Using phase portrait, is the linearized system $\\dot{\\bx} = \\left\[ \\frac{\\partial {\\bf f}}{\\partial \\bx} \\right\]\_{\\bx = \\bx^\*}(x - \\bx^\*)$ stable i.s.L.? exponentially stable?

# References

1.  Richard M. Murray and John Hauser, "A case Study in Approximate Linearization: The Acrobot Example", Memorandum No. UCB/ERL (unknown) , April, 1991.
  
3.  M. W. Spong, "Underactuated Mechanical Systems", Control Problems in Robotics and Automation , 1997.
  
5.  Hassan K. Khalil, "Nonlinear Systems", Prentice Hall , December, 2001.
  
7.  Jean-Jacques E. Slotine and Weiping Li, "Applied Nonlinear Control", Prentice Hall , October, 1990.
  
9.  Katsuhiko Ogata, "Modern Control Engineering", Prentice Hall Incorporated , August, 1996.
  
11.  Gilbert Strang, "Introduction to Linear Algebra", Wellesley-Cambridge Press , October, 1998.
  
13.  Chi-Tsong Chen, "Linear System Theory and Design", Oxford University Press , Sept 10, 1998.
  
15.  R.W. Brockett, "Asymptotic stability and feedback stabilization", Differential Geometric Control Theory , pp. 181-191, 1983.
  
17.  Kemin Zhou and John C. Doyle, "Essentials of Robust Control", Prentice Hall , 1997.
  
19.  Mark Spong, "Partial feedback linearization of underactuated mechanical systems", Proceedings of the IEEE International Conference on Intelligent Robots and Systems , vol. 1, pp. 314-321, September, 1994.
  
21.  Isabelle Fantoni and Rogelio Lozano, "Non-linear Control for Underactuated Mechanical Systems", Springer-Verlag , 2002.
  
23.  Araki and N.; Okada and M.; Konishi and Y.; Ishigaki and H.;, "Parameter identification and swing-up control of an Acrobot system", International Conference on International Technology, 2005.
  
25.  Xin Xin and M. Kaneda, "New analytical results of the energy based swinging up control of the Acrobot", Proceedings of the 43rd IEEE Conference on Decision and Control (CDC) , vol. 1, pp. 704 - 709, Dec, 2004.
  
27.  Mark W. Spong, "Swing up control of the Acrobot", Proceedings of the IEEE International Conference on Robotics and Automation (ICRA) , pp. 2356-2361, 1994.
  
29.  Arun D. Mahindrakar and Ravi N. Banavar, "A swing-up of the acrobot based on a simple pendulum strategy", International Journal of Control, vol. 78, no. 6, pp. 424-429, April, 2005.
  
31.  M.D. Berkemeier and R.S. Fearing, "Tracking fast inverted trajectories of the underactuated Acrobot", Robotics and Automation, IEEE Transactions on, vol. 15, no. 4, pp. 740-750, Aug, 1999.
  
33.  Xu-Zhi Lai and Jin-Hua She and Simon X. Yang and Min Wu, "Stability Analysis and Control Law Design for Acrobots", Proceedings of the 2006 IEEE International Conference on Robotics and Automation , May, 2006.
  
35.  Chung Choo Chung and John Hauser, "Nonlinear Control of a Swinging Pendulum", Automatica, vol. 31, no. 6, pp. 851-862, June, 1995.
  
37.  Mark W. Spong, "Energy Based Control of a Class of Underactuated Mechanical Systems", Proceedings of the 1996 IFAC World Congress , 1996.
  
39.  Mark Spong, "The Swingup Control Problem for the Acrobot", IEEE Control Systems Magazine, vol. 15, no. 1, pp. 49-55, February, 1995.
  

|     |     |     |
| --- | --- | --- |
| [Previous Chapter](pend.html) | [Table of contents](index.html) | [Next Chapter](simple_legs.html) |

- - -

|     |     |
| --- | --- |
| [Accessibility](https://accessibility.mit.edu/) | © Russ Tedrake, 2024 |