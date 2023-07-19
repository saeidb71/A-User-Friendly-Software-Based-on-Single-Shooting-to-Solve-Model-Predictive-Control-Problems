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
        \min_{\mathbf U} \int^{t^{end}_{iter}}_{t^{initial}_{iter}} \mathbf{\mathcal{L}}\left(t,\mathbf\xi(t),\mathbf u(t) \right)dt \\
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

## Methods to solve MPC

As mentioned earlier, the MPC problem is similar to the open-loop
optimal control problem in many ways. This section discusses different
methods used to solve open-loop optimal control problems. These methods,
alongside their advantages and disadvantages, are shown in
Fig.3. The first main method is "Direct" or
"Discretize then optimize", where the problem in Eq 2 is discretized, and then a Non-Linear Program
(NLP) is used to solve the optimization problem. The second main method
is "Indirect", or "optimize then discretize", where Pontryagin's Maximum
Principle (PMP) or Calculus of Variation (CV) is used to derive
optimality equations. Then numerical methods are used to solve two-point
boundary value problems.

The direct method itself consists of two methods: Sequential and
simultaneous. In the sequential method, only control is discretized, and
the state is obtained through simulation by solving the "ode" equation.
However, control and state are discretized in the simultaneous method,
and dynamic equations are satisfied by "defect" constraints and are
shown by $\zeta$.

The sequential method itself includes "Single Shooting" and "Multiple
Shooting". Single Shooting considers the whole time window as a single
section, and by discretizing control at some nodes, only the initial
state is needed to solve the problem. Multiple Shooting, on the other
hand, divides the time window into sections, and for each section, a
state value is required. In order to achieve a feasible solution, the
end value from the previous section must equal the initial value from
the current section, which is indicated by the defect constraint
($\zeta$).

There are two simultaneous methods: Single-Step and Pseudospectral. The
Single-Step method uses low-order polynomials and evenly distributes the
discretized nodes. In contrast, in the pseudo-spectral method,
high-order global polynomials such as lagrange functions are used, and
discretized nodes are not distributed evenly but rather obtained through
some methods such as roots of Legendre polynomials.

There is a main difference between the optimal control problem for
sequential and simultaneous approaches. The problem formulation for the
simultaneous approach was shown in
Eq.3. There, dynamic equations are viewed as defect
constraints ($/zeta$), and for optimization, they will be treated as
path constraints. However, this is different in the sequential method,
as shown in Eq. 4. As
we see, the dynamic is not considered as a constraint, but by having
control signals through discretization nodes, the dynamic response can
be obtained by simulation, and that's why it is shown under "where".

A comparison of the advantages and disadvantages of each method can be
found in Fig. 5. In this study "Single-Shooting" method is
used. Even though there are more powerful approaches than
Single-Shooting, there are two reasons why Single-Shooting seems to make
more sense in this case: first, the prediction horizon in MPC is usually
much shorter than the final time, so we won't see some of the problems
associated with open-loop optimal control, since the time window is
already broken down into smaller sections and the solution will not
diverge. Secondly, the Single-Shooting method has significantly less
parameters than other approaches, so the optimization algorithm has less
trouble finding the optimal solution. For this study, MATLAB "fmincon"
is used. However, if more advanced algorithms that work well for large
problems like "IPOPT' and "SNOPT" are utilized, then other methods like
simultaneous that generate large but sparse NLP problems can be used.

![MPCred GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/MPC_Methods.png
)

## Kalman Filter

Usually, in engineering systems, state estimation is necessary because
we cannot access all states, or even if accessing all states was
possible, a lot of sensors would be needed, and costs would increase.
Furthermore, since each sensor has some noise associated with it, direct
use is not recommended. Furthermore, the system model differs from the
real plant due to disturbances and model mismatches. The Kalman filter
provides a good state estimation even when there is measurement noise,
disturbance noise, and mismatch between the model and measurement data.

MPC and Kalman filter loop is shown in
Fig. 4. Here $K$ is Kalamn gain, $H$ is system
output matrix, $Q$ is process noise, $R$ is measurement noise, and $P$
is estimation error covariance. $\hat{x}_{k|k-1}$ means state
esdtimation at step $k$ given information up to $k-1$. Kalman filter has
two phases: prediction and update. In the prediction phase, the model of
the system is used for state propagation, while measurement data is used
to update the state during the update. It should be noted that the MPC
code developed here works for any nonlinear system, but if the Kalman
filter flag is activated, only linear systems should be used.

![MPCred GUI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/Kalman.png
)

# Software Illustration

The code generated in this work can be used in Matlab by calling its
function or by using the Graphical User Interface shown in
Fig.5 The
components that need to be defined in this code are shown in
Fig.6

![MPCred GUrI](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/GUI.png
)

![MPCred GU2I](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems/blob/main/Images/Structure.png
)

The structure of the components used to run this code is shown in
Fig.[7](#fig:Code_struct){reference-type="ref"
reference="fig:Code_struct"}. The first one is Model Data. It contains
initial time, final time, the initial state for real dynamic, the
initial state for model dynamic, scaled matrices, and all variables and
functions used to define dynamics and path constraints. $t_0$ is initial
time, $t_f$ is final time, $A_{\mathrm{scaled}}$ is a vector whose size
is equal to number of states and its elements are equal
$(\mathrm{ub}_x+\mathrm{lb}_x)/2$ where $\mathrm{ub}_x$ and
$\mathrm{lb}_x$ shows upper-bound and lower-bound considered for that
state and $B_{\mathrm{scaled}}$ is a diagonal matrix and its elements
are $(\mathrm{ub}_x-\mathrm{lb}_x)/2$. $C_{\mathrm{scaled}}$ is a vector
whose size is equal to number of controls and its elements are equal
$(\mathrm{ub}_u+\mathrm{lb}_u)/2$ where $\mathrm{ub}_u$ and
$\mathrm{lb}_u$ shows upper-bound and lower-bound considered for that
control and $D_{\mathrm{scaled}}$ is a diagonal matrix and its elements
are $(\mathrm{ub}_u-\mathrm{lb}_u)/2$. $\mathrm{Initial\_state\_real}$
shows state initial value considered for real model and,
$\mathrm{initial\_state\_hat}$ shows state initial value considered for
estimation model. It should be mentioned that the objective is only
considered in lagrange form, so there is no Meyr cost, and it is
included as the last state in the dynamic equation, so after running the
code, the final time of the last state shows the scaled objective value.

The system under study may have several control signals with different
values. For example, the control signals in wind turbines can be
generator torque and blade pitch. These two control signals have very
different values; one is in the order of $10^6$ and the other in the
order of $10^0$. So if they are not scaled, the optimization algorithm
has difficulty finding the correct optimum. Therefore, in this study,
all control signals and states are scaled by matrices defined in the
previous paragraph. As a result, the corresponding dynamic also needs to
be scaled. Using scaled matrices, the relation between $i^{\mathrm{th}}$
scaled and unscaled state and control is:

$$\begin{aligned}
    \xi^i_{\mathrm{unscaled}}&=A_{\mathrm{scaled}}(i)+B_{\mathrm{scaled}}(i,i)\,\xi^i_{\mathrm{scaled}}\\
    u^i_{\mathrm{unscaled}}&=C_{\mathrm{scaled}}(i)+D_{\mathrm{scaled}}(i,i)\,\xi^i_{\mathrm{scaled}}
\end{aligned}$$

As a result, we have:

$$\begin{aligned}
    \dot{\xi}^i_{\mathrm{scaled}}&=B_{\mathrm{scaled}}^{-1}(i,i)\dot{\xi}^i_{\mathrm{unscaled}}
\end{aligned}$$

So, the scaled dynamics can be obtained for the scaled states and
controls. In the Section [3](#sec:Examples){reference-type="ref"
reference="sec:Examples"} it is shown how to define these parameters for
a simple example.

The next part in Fig.[6](#fig:Run_alg){reference-type="ref"
reference="fig:Run_alg"} is dynamic and path constraint. Inputs of
dynamic function is time ($t$), scaled state ($\xi_{\mathrm{scaled}}$),
scaled control ($u_{\mathrm{scaled}}$), and Model data, and the output
is scaled state time derivative ($\dot{\xi}_{\mathrm{scaled}}$).
$\mathrm{Dynamic\_hat}$ is a model of dynamics used in MPC, and
$\mathrm{Dynamic\_Real}$ is the dynamic considered as real model and
used to provide output measurement in the Kalman filter.
$\mathrm{Path\_constraint}$ is a vector whose elements are less than or
equal to zero when all path constraints are satisfied. If there is no
path constraint, it should be null ($\{\}$).

The third structure is the MPC parameters. $T_s, m$ and $p$ are control
sampling time, control horizon, and prediction horizon, respectively.
$h$ is the time step used to solve dynamics, and $\mathrm{const}_h$ is
the time step used to provide path constraints, so it can be as small as
$h$. $x_0$ is a vector with size $n_u$, where $n_u$ is the number of
controllers and contains the initial guess for control parameters. Last
but not least, we have the Dynamic Solver Method, which determines how
the dynamic equation should be solved. In the shooting method, the
dynamic is satisfied through simulation rather than defect constraint
($\zeta$) in the optimization algorithm. The dynamic equation is:

$$\begin{aligned}
    &\dot{\bm \xi}=\bm f\left(t,\bm \xi(t),\bm u(t)\right)
\end{aligned}$$

To get the solution of this problem in time interval
$[t_{k-1}\,\, t_{k}]$, we have:

$$\begin{aligned}
    \bm \xi [t_k]=\bm \xi [t_{k-1}]+\int_{t_{k-1}}^{t_k}\bm f\left( \tau,\bm \xi (\tau), \bm u(\tau) \right) d\tau
\end{aligned}$$

There are different methods to compute the integral. here we consiered
two methods: Euler and 4th order Runge-kutta. For the Euler method, we
have [@herber2014dynamic]:

$$\begin{aligned}
    \bm \xi [t_k]=\bm \xi [t_{k-1}]+h_k \bm f[t_{k-1}]
\end{aligned}$$

and for 4th order Runge-kutta we have [@herber2017advances]:

$$\begin{aligned}
    \bm \xi [t_k]&=\bm \xi [t_{k-1}]+\frac{h_k}{6}\left( \bm k_1+ 2 \bm k_2+ 2\bm k_3+ \bm k_4 \right)\\
    \bm k_1 &= \bm f[t_{k-1}]\\
    \bm k_2 &=\bm f\left( \frac{t_{k-1}+t_k}{2}, \bm \xi[t_{k-1}]+\frac{h_k \bm k_1}{2}, \bm{ \overline{u}}_k \right)\\
    \bm k_3 &=\bm f\left( \frac{t_{k-1}+t_k}{2}, \bm \xi[t_{k-1}]+\frac{h_k \bm k_2}{2}, \bm{ \overline{u}}_k \right)\\
    \bm k_4 &=\bm f\left( t_k, \bm \xi[t_{k-1}]+h_k \bm k_3, \bm u [t_k] \right)\\
    \bm {\overline{u}}_k &=\frac{1}{2}\left( \bm u[t_{k-1}] +\bm u [t_k] \right)
\end{aligned}$$

To ensure that the dynamic is solved accurately, the Runge-Kutta method
of 4th order is recommended. The simulation accuracy can also be
increased by decreasing time step $h$, but the computation cost will
rise as a result.

The next structure defines Kalman filter parameters and functions. In
the Kalman structure, "flag" shows whether the Kalman filter is used or
we have assumed full access to all states is achieved.
$Q_\mathrm{Kalman}$ shows process noise, $R_\mathrm{Kalman}$ shows
measurement noise, $P_0$ shows estimation error covariance at the
initial time, $H_\mathrm{Kalman}$ shows system output matrix, and
$P_\mathrm{dynamics}$ is dynamics of estimation error covariance.

The last structure is "FMINCON options", which allows the user to
specify parameters for FMINCON. In addition, the user can define
${\mathrm{number\_of\_cores}}$ as an integer value bigger than or equal
to one. This software is used to solve several problems in the following
section. Please refer to this [[GitHub
repository]{.underline}](https://github.com/saeidb71/A-User-Friendly-Software-Based-on-Single-Shooting-to-Solve-Model-Predictive-Control-Problems.git)
to see how the problem is defined based on the structure discussed.

<figure id="fig:Code_struct">

<figcaption>Different structures used in MPC software</figcaption>
</figure>

# Examples {#sec:Examples}

In this section 4 examples are considered and their results are shown.
The first 3 examples have an exact solution, but the fourth one is a
vehicle suspension problem defined in @herber2019problem. These examples
examine the effects of control sampling time, control horizon, and
prediction horizon. In addition, the code in GitHub gives a good example
of how the user should define a problem.

## Example 1 {#sec:sub-Example1}

The first problem is obtained from @bryson2018applied pp.120-122.

$$\begin{aligned}
    &\min_{\bm u(t)} \int^{t_f}_{t_0} \frac{1}{2}u^2 dt \label{eq: ex1}\\
    &\mathrm{where:}\\
    &\dot{\bm \xi}=\begin{bmatrix} 
                    0 & 1\\
                    0 & 0
                    \end{bmatrix}
                    \bm \xi +
                    \begin{bmatrix} 
                    0\\
                    1
                    \end{bmatrix} u\\
    & \xi_1(0)=0,\,\, \xi_2(0)=v_0,\,\, \xi_1(t_f)=1,\,\, \xi_2(t_f)=-1\\
    &\xi_1(t)\le l
\end{aligned}$$

In this example, there are two states. Furthermore, as was mentioned
before, the objective function will be added to the states, resulting in
a total of three states and one control. On the basis of
Eq.[\[eq: ex1\]](#eq: ex1){reference-type="ref" reference="eq: ex1"},
the scaled matrices can be defined as follows:

$$\begin{aligned}
    A_{\mathrm{scaled}}&=\begin{bmatrix}
    (l-l)/2\\
    (2-2)/2\\
    (50-50)/2
    \end{bmatrix} \\
    B_{\mathrm{scaled}}&=\begin{bmatrix}
    (l+l)/2 & 0 & 0\\
    0 &(2+2)/2 & 0\\
    0 & 0 & (50+50)/2
    \end{bmatrix} \\
    C_{\mathrm{scaled}}&=\begin{bmatrix}
    (20-20)/2
    \end{bmatrix} \\
    D_{\mathrm{scaled}}&=\begin{bmatrix}
    (20+20)/2
    \end{bmatrix} 
\end{aligned}$$

The exact open-loop optimal control when $l=1/9$ is:
$$u^{\ast} = \left\{ \begin{array}{ll}
         -\frac{2}{3l}\left( 1-\frac{t}{3l} \right) & \mbox{if $0 \leq t \le 3l$};\\
         0 & \mbox{if $3l \leq t \le 1-3l$};\\
        -\frac{2}{3l}\left( 1-\frac{1-t}{3l} \right) & \mbox{if $1-3l \leq t$}.\end{array} \right.$$

The MPC result alongside the exact solution is shown in
Fig.[8](#fig:Ex1){reference-type="ref" reference="fig:Ex1"}. In each of
these cases, the prediction horizon covers the entire simulation window,
and only the control sampling time is altered. As a general rule, the
results are close to the exact solution, and when the sampling time for
the control action is shortened, the results will be close to the exact
solution.

<figure id="fig:Ex1">

<figcaption>Examples 1 response through different scenarios. Here
“exact” is obtained by analytical solution, and other legends show the
corresponding MPC parameters in each scenario.</figcaption>
</figure>

Running this code using the Matlab function instead of GUI will give the
user more accessibility to different signals. For example consider
solving this problem with $T_S=0.2$, $p=3$, $m=2$.
Fig.[9](#fig:Ex1-study2){reference-type="ref"
reference="fig:Ex1-study2"} shows the result in each iteration. At each
iteration, the MPC tries to solve the problem in time intervals shown in
the legends. Because $m=2$ and $p=3$, the controller has only two free
variables in each interval, and after that, it is fixed and is equal to
the last step. In addition, after each iteration, only the control at
the initial point is implemented on the system. Then it goes to the next
iteration and solves the MPC problem over the new time interval. Here,
dash lines ($--$) show control and state values at each iteration. Also,
the control applied to the system, which is equal to control at the
initial point at each iteration, is shown by "Correct" and the obtained
state is also shown by "Correct". In this way, the designer will be able
to understand what is going on and decide whether the prediction
horizon, control horizon, or control sampling time have been selected
correctly or not.

<figure id="fig:Ex1-study2">

<figcaption>Investigating Example 1 results when <span
class="math inline"><em>T</em><sub><em>s</em></sub> = 0.2,  <em>p</em> = 3,  <em>m</em> = 2</span>.</figcaption>
</figure>

## Example 2 {#sec:sub-Example2}

The second problem is obtained from @bryson2018applied pp.166-167:

$$\begin{aligned}
    &\min_{\bm u(t)}  \frac{1}{2}\int^{t_f}_{t_0} u^2 dt\\
    &\mathrm{where:}\\
    &\dot{\bm \xi}=\begin{bmatrix} 
                    0 & 1\\
                    -1 & 0
                    \end{bmatrix}
                    \bm \xi +
                    \begin{bmatrix} 
                    0\\
                    1
                    \end{bmatrix} u\\
    & \xi_1(0)=x_0,\,\, \xi_2(0)=v_0,\,\, \xi_1(t_f)=0,\,\, \xi_2(t_f)=0              
\end{aligned}$$

The exact open-loop optimal control when $x_0=-1/2$ and $v_0=1$ is:
$$\begin{aligned}
    u^{\ast}(t)=-\frac{2}{t_f^2-\sin^2(t_f)}
    \begin{bmatrix}
    x_0\\
    v_0
    \end{bmatrix}^T
    \begin{bmatrix}
    \sin(t_f-t)\sin(t_f)-t_f\sin(t)\\
    -\cos(t_f-t)\sin(t_f)+t_f\cos(t)
    \end{bmatrix}
\end{aligned}$$

The MPC result alongside the exact solution is shown in
Fig.[10](#fig:Ex2){reference-type="ref" reference="fig:Ex2"}. In the
first two MPC cases, the prediction horizon is the same as the final
time, and the difference is in the control sampling time. So these two
cases have a preview of the system during the whole time window.
However, in the third MPC case, the time window is
$pT_s=1.0\, \mathrm{s}$. In addition, because the final time is $2$ s,
the MPC does not know anything about boundary constraints at the final
point until the start time reaches 1 s. This can be seen in
Fig.[10](#fig:Ex2){reference-type="ref" reference="fig:Ex2"} (c), where
control is equal to $0$ from $t=0$ to $t=1$. Here the system objective
was to minimize the control signal, and the system has no information
about boundary constraint at the final time. However, it sees the
boundary constraint when it reaches $1$ s, and the control signal
changes to satisfy that constraint.

<figure id="fig:Ex2">

<figcaption>Examples 2 response through different scenarios. Here
“exact” is obtained by analytical solution, and other legends show the
corresponding MPC parameters in each scenario.</figcaption>
</figure>

## Example 3 {#sec:sub-Example3}

The third problem is obtained from @bryson2018applied pp.109-110.

$$\begin{aligned}
    \min_{\bm u(t)} \frac{a^2}{2}\xi^2_{t_f}+& \int^{t_f}_{t_0} \frac{1}{2}u^2 dt\\
    &\mathrm{where:}\\
    &\dot{\bm \xi}=b(t)u\\
    & \xi(0)=\xi_0\\
    &|u(t)|\le 1
\end{aligned}$$

The exact open-loop optimal control is:
$$u^{\ast} = -\mathrm{sat}\left[ a^2b(t)\xi(t_f) \right]$$

The MPC result alongside the exact solution for the case where $t_f=1$,
$\xi_0=1$, $a=1$ and $b(t)=t\cos(20 \pi t)-1/4$ is shown in
Fig.[11](#fig:Ex3){reference-type="ref" reference="fig:Ex3"}. In both
MPC cases, the control sampling time is fixed, and only the prediction
horizon is changed. In the first case, the prediction horizon covers the
full time horizon, but in the second case, it only covers $0.2$ s. In
the second case, the MPC does not have access to Meyr cost, and the MPC
action is zero until it reaches $0.8$ s, where it starts to have
information on the Meyr cost. However, in the first case, MPC has access
to Meyr cost from the beginning, and the result is close to the
open-loop control.

<figure id="fig:Ex3">

<figcaption>Examples 3 response through different scenarios. Here
“exact” is obtained by analytical solution, and other legends show the
corresponding MPC parameters in each scenario.</figcaption>
</figure>

## Example 4 {#sec:sub-Example4}

The fourth problem is a simple model of vehicle suspension system and is
obtained from @herber2019problem and is shown in
Fig.[12](#fig:VHC){reference-type="ref" reference="fig:VHC"}, where
$\delta$ is road profile, $U$ is unsprung mass, $S$ is sprung mass,
$z_\mathrm{U}$ is the displacement of unsprung mass, and $z_\mathrm{S}$
is the displacement of the sprung mass. Additionally, $F$ represents
control force, $m$ represents mass, $k$ represents spring, and $b$
represents damper. By transferring forces through springs, dampers,
mass, and actuators, suspensions in automobiles aim to provide
passengers with a smooth ride. The objective is defined based on
@herber2019problem and is shown in
Eq.[\[eq: obj\]](#eq: obj){reference-type="ref" reference="eq: obj"},
where $w_1(z_U-\delta)^2$ shows handling performance, $w_2\Ddot{z}_S^2$
represents passenger comfort, and $w_3F^2$ is a penalty function for
control effort. The initial value of states is set to zero, the
displacements between sprung and unsprung mass are bound by
$r_\mathrm{max}$, and all displacements between two consecutive masses
are bound by $s_\mathrm{max}$. These parameters are shown in
Table.[1](#table: Vh_params){reference-type="ref"
reference="table: Vh_params"}. In this example, $T_s=0.01$, $p=50$, and
$m=50$, so the length of time window interval is $0.5$ s.

$$\begin{aligned}
\label{eq: obj}
    \Uppsi_d=\int_{t_0}^{t_F} \left( w_1(z_U-\delta)^2+w_2\Ddot{z}_S^2+w_3F^2\right)\mathrm{d}t
\end{aligned}$$

![Vehicle suspension system](Images/case_2_b.pdf){#fig:VHC}

::: {#table: Vh_params}
       Parameter                 Value                 Parameter                Value
  -------------------- -------------------------- -------------------- -----------------------
         $t_0$                    0 s                    $t_f$                   3 s
          $b$                 $100.8$ Ns/m                $k$              $2.15*10^4$ N/m
   $r_{\mathrm{max}}$           $0.04$ m           $s_{\mathrm{max}}$         $0.04$ m
    $m_{\mathrm{U}}$            $65$ kg             $m_{\mathrm{S}}$          $325$ kg
         $w_1$           $10^5\, s^{-1}m^{-2}$           $k_t$          $232.5\times10^3$ N/m
         $w_2$            $0.5\, s^{3}m^{-2}$            $b_t$                $0$ Ns/m
         $w_3$          $10^{-5}\, s^{-1}N^{-2}$                       

  : Vehicle suepnsion parameters
:::

In this problem also Kalman filter is activated. It is assumed that the
displacement between sprung and unsprung mass is measured with some
noise, and the Klaman filter is used to provide estimation for all
states. Additionally, the MPC model's initial condition differs from the
real model's, which is at rest. The result is shown in
Fig.[13](#fig:Ex4){reference-type="ref" reference="fig:Ex4"}. Parts (a)
and (b) shows Sprung and Unsprung mass positions for real and model
system. Using the Kalman filter makes these two close to each other,
despite the model's initial state differing from the real one. In part
(c), the control signal is shown, and in part (d), the real displacement
difference ($z_{\mathrm{S}}-z_{\mathrm{U}}$), the estimated displacement
differnce ($\hat{z}_{\mathrm{S}}-\hat{z}_{\mathrm{U}}$) and the
measurement ($z_k$) that was used in Kalman filter is shown. As we see,
generally, the estimated signal is somewhere between the real state and
the measurement, and that's what Kalman is doing through Kalman gain.
The innovation term, which is the difference between measured and
estimated output, is a zero mean with covariance ($H_kP_k^-H_k^T+R_k$)
[@simon2006optimal]. In part (e), it is shown that all innovation values
lie between $-2\sigma$ to $2\sigma$, where
$\sigma=\sqrt{H_kP_k^-H_k^T+R_k}$, so the Kalman filter is working
correctly. In the last part, estimation error and measurement error are
shown. As we expect, the estimation error is generally less than the
measurement error because the Kalman filter uses both measurement and
estimations to provide better accuracy than simply using measurement.

<figure id="fig:Ex4">

<figcaption>Vehicle suspension results</figcaption>
</figure>

# Conclusion {#sec:Concl}

In this article, a user-friendly software is developed to solve MPC
problems. Here, the single shooting method is used to convert the
problem to a nonlinear program, and then Matlab "fmincon" is used to
solve the optimization problem. Furthermore, MPC is equipped with a
Kalman filter that can be activated by flag. This code can be run both
by calling its function and by GUI. This code was tested under different
examples with exact solutions, and the MPC results were compared with
the exact one. In addition, the code was used to solve the vehicle
suspension problem defined in @herber2019problem. This paper
demonstrated how this code works and how different parts should be
defined. So it can be used by engineers with different majors because no
detailed information on MPC is needed.

# Declaration of Competing Interest {#declaration-of-competing-interest .unnumbered}

The authors declare that they have no competing interest.

# Data Availability {#data-availability .unnumbered}

All data required to replicate the results can be generated by the
MATLAB optimization code. The MATLAB optimization codes for all the
problems demonstrated in the manuscript are available upon request to
the first or corresponding authors.
