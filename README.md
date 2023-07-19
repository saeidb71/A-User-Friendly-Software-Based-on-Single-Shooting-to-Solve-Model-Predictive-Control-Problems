# A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems
This is a User-Friendly Software Based on Single Shooting to Solve Model Predictive Control Problems
![MPC GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/GUI.png)

This is a user-friendly Matlab program that simplifies the design of Model Predictive Control (MPC) systems. Our motivation behind creating this tool is to address the lack of user-friendly software that integrates both MPC and the Kalman filter techniques. While the Matlab MPC Toolbox is widely used, it still requires users to define control loops and specify state update rules, which can be complex for those lacking expertise. Our software streamlines the design process by offering an intuitive interface and seamlessly integrating the Kalman filter with MPC. We aim to support industry adoption of MPC by providing an accessible solution that bridges the gap between theory and practice. The software allows users to interact with the MPC system, visualize results, and optionally combine a Kalman filter. This software has been submitted to the Software Impacts journal. Upon publication, additional documents will be added to this repository, providing detailed instructions, examples, and analytical solutions to verify the correctness of the software.

---

# Introduction

Many modern engineering systems integrate a control component to
effectively modify system dynamic response and achieve desired outcomes.
The impact of a control system on system behavior can be significant,
e.g., it can transform an unstable system into a well-behaved one.
However, this positive impact is not without costs. Implementation of
active system control requires inclusion of various components such as
actuators, sensors, and processors. To enhance system capabilities and
performance while managing cost, optimization studies can be helpful in
exploring tradeoffs. One approach to obtaining optimal control solutions
is through formulating an open-loop optimal control (OLOC) problem. This
method assumes access to all information from the initial time to the
final time, and does not impose any structure on the controller. As a
result, it largely outperforms other control structures. In this
approach, the optimizer has the freedom to generate an optimal control
signal through time, independent of system states. More specifically,
the resulting control signal is not a function of system states, but
rather varies as a function of time. This approach helps to identify
physical system performance limits, but does have drawbacks.

When implementing practical control systems on real physical systems,
open-loop control typically is not used due to the lack of stability
guarantees and the potential for significant deviations in system
behavior caused by even minor discrepancies between the model and the
real system. In reality, it is rare to have access to complete and
sufficiently accurate information from start to finish. For example, in
the case of wind turbines continuous measurements and predictions of
incoming air velocity over long periods of time are normally
unavailable. These limitations render open-loop controls ineffective in
practical settings. To overcome these challenges, closed-loop controls
are utilized, where state feedback is employed to guide the behavior of
the controller under varying conditions. In closed-loop control, control
actions (signals) depend upon system state through a control law, and
often stability can be proven based on the controller design. This
allows for greater adaptability to model variations, as slight
differences between the model used to generate the control law and the
actual system can be accommodated. However, there is a trade-off
associated with this approach. The structured nature of the closed-loop
controller, even when optimized using algorithms, may not yield results
that perform as well as OLOC. This is because closed-loop controllers
have less freedom compared to open-loop controllers, and the information
used to determine control signals is limited. One exception is the
Linear Quadratic Regulator (LQR), which incorporates a state feedback
structure and generates a control signal to the OLOC solution for linear
systems with a particular problem formulation [@locatelli2002optimal].

OLOC utilizes all available information over the entire time horizon,
while closed-loop control relies solely on current information without
considering past or future data. However, it would be advantageous to
have a control structure that allows for flexibility in the amount of
information used. This would enable the system to function as a feedback
control system when information is limited to the present time, and as
an open-loop control system when comprehensive information is available.
Model Predictive Control (MPC) is a powerful tool that is flexible in
the amount of information that can be utilized, as well as the speed at
which the control signal can be computed, based on the capabilities of
the actuators, sensors, and processors. MPC provides a flexible approach
that adapts to the available information, allowing for efficient control
signal generation. Moreover, MPC can also be effectively applied in
Control Co-Design (CCD), where plant parameters and control strategies
are jointly optimized to leverage synergistic relationships between them
[@allison2013multidisciplinary]. For example, high-rate sensors or
actuators may incur higher costs, and using MPC in CCD allows for the
optimization of sensor, actuator, and hardware selection in conjunction
with the control signal. This results in an optimal solution for the
entire system, taking into account both control performance and cost
considerations. MPC is a versatile approach that enables the control
system to adapt to the available information, allowing for efficient
control signal computation. It is particularly valuable in CCD studies,
where it can support optimization of both the control strategy and
hardware selection to achieve an optimal solution for the system.

State estimation is a crucial aspect of control system design, as direct
access to all states may not always be feasible or cost-effective due to
the requirement of multiple sensors. State estimation techniques are
employed to estimate the states of a system based on available
measurements. One widely used method for state estimation is the Kalman
filter, as proposed by Kalman [@simon2006optimal]. The Kalman filter is
particularly effective for linear systems, and it can also handle minor
mismatches between the system model and the real system. For nonlinear
systems, the extended Kalman filter is commonly employed as it allows
for state estimation in the presence of nonlinearity.

Although there is extensive literature on the concepts of MPC and the
Kalman filter, there is currently a lack of user-friendly software that
integrates both of these techniques. While the Matlab MPC Toolbox
[@bemporad2004model] provides a popular option for MPC implementation,
users still need to define the control loop at each time horizon and
specify the state update rules, which can be complex tasks. This can be
a barrier to adoption for users in industries who may lack expertise in
control and optimization. In contrast, the Proportional Integral
Derivative (PID) controller is widely used in industry due to its
simplicity in design and implementation. However, advanced controllers
like MPC have the potential to offer significant benefits in terms of
improved control performance and net cost savings. By developing simple
and user-friendly software for advanced controllers like MPC, it may be
possible to facilitate their adoption in industry, resulting in a shift
towards more advanced control strategies. The potential cost savings
from using optimal control solutions such as MPC can be substantial. For
instance, a study by @schwenzer2021review showed that using MPC to
control the temperature of a small mock-up building in a thermal chamber
reduced energy consumption by $43\%$ compared to a constant temperature
controller. In addition, CCD studies can help reduce the cost of
implementing and maintaining the physical aspects of the system,
including through reduced physical system architecture complexity
[@herber2014dynamic]. Providing user-friendly software tools that
integrate MPC and the Kalman filter could enable users to test and
implement these advanced control strategies more easily, even if users
lack in-depth knowledge of control and optimization. This could pave the
way for wider adoption of MPC and other advanced controllers in the
industry, leading to improved control performance and cost savings in
various industrial applications
[@clements2006getting; @pass2014reducing].

## Model Predictive Control (MPC)

More than fifty years of literature exists on the topic of developing
closed-loop control designs based on open-loop control trajectories. MPC
can be viewed as using repeated OLOC solutions in real time to form the
basis of a control law. For example, @lee1967foundations described the
essence of MPC at a very early stage by stating that "One technique for
obtaining a feedback controller synthesis from knowledge of open-loop
controllers is to measure the current control process state and then
compute very rapidly for the open-loop control function. The first
portion of this function is then used during a short time interval,
after which a new measurement of the process state is made and a new
open-loop control function is computed for this measurement. The
procedure is then repeated."

As the MPC framework requires small sample rates, advanced processors
are required. In the early days of MPC development, it was used in
applications with large sample periods and slow dynamics. With the
development of more advanced transistors and the resulting advanced
processors, however, recent application of MPC includes systems
requiring faster sampling rates, such as active automotive suspensions
[@giorgetti2006hybrid], automotive powertrains [@saerens2008model],
power converters [@richter2010high], vehicle traction control
[@borrelli2006mpc], and so on. Furthermore, MPC is heavily used in
building operations, and several frameworks such MShoot
[@arendt2019mshoot], MPCPY [@blum2019mpcpy], and TACO
[@jorissen2019taco] have been developed for this task.

MPC has three main tuning factors, which can be viewed as
hyperparameters. The parameters are as follows:

-   Control Sampling Time ($T_s$): The control sampling time $T_s$ is
    the rate at which the controller executes the control algorithm
    (i.e., solves the associated OLOC and updates the control signal).
    In the time interval between each sampling, the control action
    remains constant, and is equal to the previous step. Having a large
    $T_s$ limits the controller's ability to react to disturbances
    quickly. Alternatively, when it is smaller, it responds much faster
    to disturbances and setpoint changes, but it takes longer to
    compute. As a rule of thumb, 10 to 20 samples should be included in
    the system rise time, which defines the time it takes for a state
    value to rise from 10% to 90% of the steady-state response. This is
    shown in Fig. [1](#fig:MPC-params){reference-type="ref"
    reference="fig:MPC-params"}(a).

-   Prediction Horizon ($p$): The value of $p\in\mathbb{Z}^+$ is the
    number of time steps used in defining the MPC OLOC problem. More
    specifically, $pT_s$ is the length of the time window for which MPC
    computes prediction based on the given model. If $p$ is small, this
    MPC control law behaves closer to a simple feedback control law. As
    $p$ is increased, it behaves closer to OLOC. It is generally
    recommended to have enough prediction samples to cover the settling
    time of the system, which is defined as the time when the error
    between the system step response and its steady-state is less than
    or equal to 2%. This is illustrated in
    Fig. [1](#fig:MPC-params){reference-type="ref"
    reference="fig:MPC-params"}(b). In the case where $p$ is large, we
    assume that our model is accurate enough for such a long horizon,
    and we have access to predicted system inputs during this future
    time interval. Reasonably accurate predictions of future inputs can
    be made for some systems. For example, for wind turbines, the
    predicted future wind speed $v(t)$ can be estimated using LiDAR
    measurements [@dunne2014importance]. Availability of wind speed
    estimates, along with a suitable real-time model, allow us to use a
    large prediction horizon.

-   Control Horizon ($m$): The control horizon, $m \le p$,
    $m\in\mathbb{Z}^+$, is the number of time steps for which MPC
    calculates optimal actions, and is less than or equal to the
    prediction horizon. $mT_s$ is the length of the optimized control
    time window, and the action after that remains constant. As a rule
    of thumb, set $m$ between $0.1 p$ and $0.2 p$ with a minimum value
    of $p=3$. This is shown in
    Fig. [1](#fig:MPC-params){reference-type="ref"
    reference="fig:MPC-params"}(c). On the real system, only the optimal
    control action at the first time step is implemented, and when the
    system proceeds to the next time step, this optimization will be
    repeated.

![MPCr GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/MPC_params.png)

At each time step, MPC solves a specific type of OLOC problem. To begin
with, we need to consider the general open-loop optimal control problem
formulation, which is presented in Lagrange form in
Eq. 1 [@herber2017advances; @herber2014dynamic].
Here $t$ is time, $t_0$ is initial time, $t_f$ is final time,
$\mathbf{u}(t)$ is the action (or control signal), $\mathbf \xi(t)$ is the
vector of state variables, $\mathcal{\mathbf C}(\cdot)$ is the path
constraint, and $\mathcal{\mathbf \phi}(\cdot)$ is the boundary constraint.

$$\begin{aligned}
    \min_{\mathbf u(t)} \int^{t_f}_{t_0} &\mathbf{\mathcal{L}}\left(t,\mathbf\xi(t),\mathbf u(t) \right)dt \\
    &\mathrm{Subject\,\, to:}\nonumber \\
    &\dot{\mathbf \xi}-\mathbf f\left(t,\mathbf \xi(t),\mathbf u(t)\right)=0\nonumber \\
    &\mathcal{\mathbf C}\left( t,\mathbf\xi(t),\mathbf u(t) \right)\le0\nonumber \\
    &\mathcal{\mathbf \phi} \left(t_0,\mathbf \xi(t_0),t_f,\mathbf \xi(t_f) \right)\le 0\nonumber 
\end{aligned}$$

MPC differs from the general OLOC problem formulation primarily in the
time intervals used, and in the assumed shape of the control signal at
times other than $T_s$. In open-loop optimal control, the problem is
solved once to obtain $\mathbf u(t)$ for all values of $t$ between $t_0$ and
$t_f$. In contrast, MPC is solved many times to provide updated control
signal values that can adapt to uncertainty and model error.

Suppose that in an MPC implementation we assume that $t_0 = 0$, and
$T_s$ and $t_f$ are specified. The number of MPC iterations (the number
of times the real-time MPC optimization problem is solved) is:

$$\begin{aligned}
    n_{\mathrm{iterations}}=\frac{t_f}{T_S}%+1
\end{aligned}$$

The basic MPC procedure is expressed in Algorithm 1. At
each time step, starting with $t=0$ and ending with $t=t_f-T_s$, the
`for` loop is executed to obtain $\mathbf{u}(t)$. This control signal is
then fed to the physical system actuators, and the resulting state value
is measured at the next time step $t+T_s$ and used to form the
optimization problem for the next MPC iteration. At each MPC iteration
the time interval over which the optimization problem is solved is
$t^{initial}$,
the boundaries of which are defined in Algorithm 1. The length of the control horizon is $t_f-t^{initial}$ or, whichever is shorter. The
prediction horizon is of length $p T_s$ or
$t_f - t^{initial}_{iter}$, whichever is shorter.

![MPCre GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/Alg.png
)

After initializing these time intervals and obtaining the current state
value $\mathbf{\xi}({t^{initial}})$ (using
estimation if required), the optimization problem for MPC iteration
`iter`, defined in
Eq.2, is solved for the optimal control signal
values $\mathbf{U}$, which is a matrix that includes the optimal control
signal at each discrete time step between
$t^{initial}$ and $t^{Control}$. At times between sample points
the control signal is equal to that of the most recent sample time.
After solution, the appropriate value from $\mathbf{U}$ is then used as
the control input to the physical system actuators. The dynamic system
continues to evolve, and at the next time step the process is repeated.

$$  \begin{aligned}
        \min_{\mathbf U} \int^{t_{iter}^{end}}_{t^{initial}} \mathbf{\mathcal{L}}\left(t,\mathbf\xi(t),\mathbf u(t) \right)dt \\
        \mathrm{Subject\, to:}\\
        \mathcal{\mathbf C}(t, \mathbf \xi (t), \mathbf u (t)) \le 0\\
        \mathcal{\mathbf \phi}(t_0,\mathbf \xi (t_0), t_f, \mathbf \xi (t_f))\le 0\\
        \mathrm{Where}\\
        \dot{\mathbf \xi}=\hat{\mathbf f}(t,\mathbf \xi (t) , \mathbf u (t))
    \end{aligned}$$

Please note that in
Eq. 2 a distinction is made between the dynamic
model of the system used for real-time computation, represented using
the state space model $\dot{\mathbf{\xi}} = \hat{\mathbf{f}}(\cdot)$, and the
true system dynamics, represented using model
$\dot{\mathbf{\xi}} = {\mathbf{f}}(\cdot)$. A tradeoff exists between the
accuracy of the real-time dynamic model and the computational expense of
solving the MPC optimization problem.

A graphical visualization of the MPC process is illustrated in
Fig.2. Each iteration minimizes the objective within the corresponding time
window while satisfying dynamic equations, path constraints, and
boundary conditions. After convergence, the control is applied to the
system, and `iter` is increased by 1. In this representation we have
assumed full access to all states, so no state estimation is necessary.
Additionally, the dynamic equation may be formulated differently
depending on the method used to solve the MPC problem. This will be
elaborated upon later.

![MPCre GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/MPC_Loop.png
)

## How to Run
