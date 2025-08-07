# Lab MIMO

::: highlight
##### Overview

This lab is dedicated to linear control. Its goals are to make you understand:

1. How to **obtain data** from a SOFA scene that corresponds to the real robot.
2. How to **reduce** the model order to a manageable size.
3. **Identify** a linear model of the dynamic of the reduced state.
4. The use of this model to **estimate** the state.
5. The use ot this model to **control** the robot.

This lab relies on linear algebra and control theory. It is recommended to check the [numpy linear algebra](https://numpy.org/doc/stable/reference/routines.linalg.html) and [control](https://python-control.readthedocs.io/en/0.10.2/) libraries documentation for more details on the implementation.
:::

::: collapse {open} Set up Emio  for the Lab
## Set up Emio

In this lab session, we will use only the following configuration: Emio with two <span style="color:rgba(200, 0, 200, 1);">*magenta legs*</span>,
the <span style="color:grey">*grey mass*</span>, and the <span style="color:green">*green marker*</span>. We also use the **FEM** modeling of the leg to simulate the dynamics of the system.

![](assets/data/images/lab6-setup-emio.png){width=75% .center}
:::



::::::: collapse Open Loop Control
## Open Loop Control

**Open Loop Control.**
The first step consists in using open-loop control within the simulation environment to generate data required for the identification of a linear model of the system. It is crucial that the simulation accurately captures the system’s dynamics, and that an appropriate time step has been chosen beforehand.

The motors used in the simulation are subject to physical constraints, including a maximum speed of 75 revolutions per minute (rpm), which must be strictly enforced. Additionally, the initial position of the motors must be specified, along with the corresponding minimum and maximum angles that can be reached from this starting point.

To avoid high-frequency oscillations in motor position, a low-pass filter is applied, defined by a given cutoff frequency.

To excite the system in a way that is informative for model identification, it is recommended to apply inputs drawn from a normal distribution, resulting in a pseudo-random motion of the motor. To further enrich the data, random noise is also added to the input commands.

The final open loop control structure is as follows:
|  ![](assets/data/images/lab6-openloop-structure.png)   |
|:------------------------------------------------:|
| **Open loop control structure** |


:::::: exercise
**Exercise 1:**

Set up the scene with the following parameters and try to simulate the robot with the open loop.

::::: group-grid {style="grid-template-rows:repeat(5, 0fr);"}
**Motor n°1**
Init, Min, Max (rad)
#input("motor1Init")

#input("motor1Min")

#input("motor1Max")

**Motor n°2**
Init, Min, Max (rad)
#input("motor2Init")

#input("motor2Min")

#input("motor2Max")

* * *
**Frame rate (Hz)**
:::: select fps
::: option 60
::: option 120
::: option 180
::: option 240
::::

**Cutoff frequency (Hz)**
#input("cutoffFreq")

:::::

#runsofa-button("assets/labs/EmioLabs_Mimo/lab_mimo.py" "--controller" "openloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorsInit" "motor1Init" "motor2Init" "--motorsMin" "motor1Min" "motor2Min" "--motorsMax" "motor1Max" "motor2Max")
::::::

:::::::

:::::: collapse Model Order Reduction
## Model Order Reduction

**Model Order Reduction.**
From the previous step, we are able to generate trajectories of the system. We run simulations starting from various initial conditions, applying random inputs with added noise to excite the dynamics.

However, since we are using a finite element model (FEM), the number of degrees of freedom is quite high. For example, if we consider only the positions and velocities of all the mesh nodes in the Y-Z plane, we end up with around 3000 states. This is too large for efficient model identification and control. To address this, we apply a model order reduction technique to reduce the system's dimensionality to a more manageable size.

The idea is to collect all system measurements into a single data matrix. In our case, we retain only the positions of the nodes, where $p_i$ denotes the position of the FEM node $i$, and $L$ is the total number of measurements. The matrix $M$ is defined as:

$$M=\begin{bmatrix} p_1 & \ldots & p_L \end{bmatrix}$$

We then apply *Singular Value Decomposition* (SVD) to this matrix. SVD allows us to decompose $M$ into three matrices:

$$M = U \Sigma V^\top$$

where $U$ contains the left singular vectors (spatial modes), $\Sigma$ is a diagonal matrix of singular values (related to the importance of each mode), and $V^\top$ contains the right singular vectors (temporal information).

The columns of $U$ represent spatial modes sorted by importance. These are linear combinations of node positions that best capture the variance in the data. We then construct a reduction matrix $T$ using the first $r$ dominant modes:

$$T=\begin{bmatrix} u_1 & \ldots & u_r \end{bmatrix}$$

where $u_i$ are the first $r$ columns of $U$.  The reduced position is obtained by projecting the full position vector pp onto this subspace:

$$p_{red} = T^{\top} p$$

The quality of the approximation can be measured using the retained energy (information), computed as the ratio of the sum of the first $r$ singular values to the total sum:

$$\text{error} = 1 - \displaystyle\frac{\sum_{i=1}^{r} \sigma_i}{\sum_{i=1}^{L} \sigma_i}$$

where $\sigma_i$ are the eigenvalues from $\Sigma$.

To preserve the relationship between positions and velocities in the state vector, we apply the same transformation to the velocities. The final reduction matrix and the mapping from the full state $x$ (positions and velocities) to the reduced state $x_{red}$ is:

$$R = \begin{bmatrix} T & 0 \\ 0 & T \end{bmatrix} \Rightarrow x_{red} = R^{\top} x$$

::::: exercise

**Exercise 2:**

Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Mimo/scripts/reduction.py")

Given the data generated in the previous step, you will:
1. Compute the SVD of the matrix $M$.
2. Compute the reduction error for different orders.

#runsofa-button("assets/labs/EmioLabs_Mimo/scripts/reduction.py" "--mode" "0")

3. Select the order of the reduction $r$ to compute the transformation matrix $T$.:
:::: select order
::: option 1
::: option 2
::: option 3
::: option 4
::: option 5
::: option 6
::: option 7
::: option 8
::: option 9
::: option 10
::::

#runsofa-button("assets/labs/EmioLabs_Mimo/scripts/reduction.py" "--mode" "1" "--order" "order")
:::::
::::::

::::: collapse Linear Model Identification

## Linear Model Identification

**Linear Model Identification.**
From the previous step, we have reduced the dimensionality of the state vector, making it suitable for control and model identification. We now aim to identify a discrete-time linear state-space model of the system.

A linear state-space model is defined by the state matrix $A$, the input matrix $B$, and the output matrix $C$, following the equations::

$$\begin{array}{rcl} x(k+1) &=& Ax(k) + Bu(k) \\ y(k) &=& Cx(k) \end{array}.$$

For simplicity, we will use $x$ to denote $x(k)$ and $x^{+}$ for $x(k+1)$.

By stacking all collected measurements into matrices, the system dynamics can be compactly written as:

$$ \begin{bmatrix} x_1^+ & \ldots & x_L^+ \end{bmatrix} =  \begin{bmatrix} A & B \end{bmatrix} \begin{bmatrix} x_1 & \ldots & x_L \\ u_1 & \ldots & u_L \end{bmatrix}$$

The matrices $A$ and $B$ can therefore be identified by solving a least-squares problem, using the Moore–Penrose pseudo-inverse. Specifically:

$$\begin{bmatrix} A & B \end{bmatrix} = \begin{bmatrix} x_{1}^{+} & \ldots & x_{L}^{+}\end{bmatrix} \begin{bmatrix} x_{1} & \ldots & x_{L} \\ u_{1} & \ldots & u_{L} \end{bmatrix}^{\dagger} $$

where $*^\dagger$ is the pseudo inverse of matrix.

Similarly, the output matrix $C$ can be identified. Since the output corresponds to marker positions, which depend only on the position components $p$ of the state vector (and not the velocities), we restrict the output identification to the position subspace.

Let $C_p$ be the matrix mapping from the reduced positions to the measured outputs. It can be obtained by solving the following least-squares problem:

$$C_p = \begin{bmatrix} y_{1} & \ldots & y_{L}\end{bmatrix} \begin{bmatrix} p_{1} & \ldots & p_{L}\end{bmatrix}^{\dagger}$$

where $y_i$ is the measured output (e.g., marker position) and $p_i$ is the corresponding position extracted from the full state vector.

Since the full reduced state vector is composed of both velocities and positions (i.e., $x^\top=\begin{pmatrix} v^\top & p^\top\end{pmatrix}$), the final output matrix $C$, which maps the full reduced state to the output, is given by:

$$C = \begin{bmatrix} 0 & C_p \end{bmatrix}$$

This structure reflects the fact that the output depends solely on the position part of the state.

:::: exercise

**Exercise 3:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Mimo/scripts/identification.py")

Given the reduction matrix that you generated on the previous step, you will:
1. Reduce the data.
2. Compute the state matrix and input matrix.
3. Compute the output matrix.
4. Simulate the model given the same input as the system and compare the results.
#runsofa-button("assets/labs/EmioLabs_Mimo/scripts/identification.py" "--order" "order")
::::
:::::

::::: collapse State Feedback Control

## State Feedback Control

**State Feedback Control.**
Based on the identified linear model, we can implement a state feedback controller with feedforward compensation, defined as
$$ u = Gr - Kx $$
where $r$ is the reference signal that the controlled output should track.

This results in the following closed-loop dynamics:
$$ x^+ = (A-BK)x + BGr$$

The gain matrix $K$ can be computed using several techniques:
1. *Pole placement* (if system is controlable).
2. *Linear quadratic regulation* (LQR).
3. *Linear matrix inequalities* (LMI).

The first two methods can be implemented using the [control library](https://python-control.readthedocs.io/en/0.10.2/) in Python. The LMI approach, on the other hand, provides guarantees on performance and robustness and can be solved using convex optimization tools such as [cvxpy](https://www.cvxpy.org/index.html).

Once the matrix $K$ is known, the feedforward gain $G$ must be chosen to ensure that the controlled output $y_c$ converges to the reference $r$ in steady state.
::: highlight
#icon("warning") **Warning:** Here you need to be careful between two notions:
1. *The ouput of the system* $y$: what you can really measure. In our case it is the positions of all markers.
2. *The controlled output* $y_c$: this is the specific output you want to regulate or track. It must have the same dimension as the input $u$. Therefore, we define a new output matric for control $C_{c}$, which maps the state to the controlled output.
:::

At steady state, $x^+=x$. Plugging this into the closed-loop dynamics yields:

$$(I - A + BK)x = BGr \Leftrightarrow x = (I - A + BK)^{-1}BGr$$

We now substitute this into the controlled output equation:

$$y_{c} = C_{c}x = C_{c}(I-A+BK)^{-1}BGr$$

To ensure perfect tracking (i.e., $y_c=r$), we choose the feedforward matrix $G$ as:

$$G = (C_{ctr} (I-A+BK)^{-1}B)^{-1}$$

:::: exercise

**Exercise 4:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Mimo/scripts/controller.py")

Given the linear model that you generated on the previous step, you will:
1. Use one of the methods given earlier to compute the state feedback matrix $K$.
2. Compute the feedforward matrix $G$
#runsofa-button("assets/labs/EmioLabs_Mimo/scripts/controller.py" "--order" "order")
3. Test on Sofa to see how you can control the robot using this control law.
#runsofa-button("assets/labs/EmioLabs_Mimo/lab_mimo.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorsInit" "motor1Init" "motor2Init" "--motorsMin" "motor1Min" "motor2Min" "--motorsMax" "motor1Max" "motor2Max" "--order" "order" "--useObserver" "0")
::::
:::::

::::: collapse Observer Design

## Observer Design

**Observer Design.**
The control law presented in the previous section relies on access to the full system state.

In simulation, it is possible to measure the full state, reduce it, and use it directly for control. However, in practice—on a physical robot—this is not feasible. Only a limited set of outputs can be measured. Therefore, we need a method to estimate the full state from partial measurements.

To do this, we implement a Luenberger observer based on measurements of the marker positions in the Y-Z plane. The observer dynamics are given by:
$$\hat{x}^+ = A\hat{x} + Bu + L(y - \hat{y})$$
where $\hat{x}$ is the estimated state, $y$ is the measured output, and $\hat{y}=C\hat{x}$ is the predicted output from the estimated state.

To analyze the convergence of the observer, we define the estimation error $e=x-\hat{x}$. The error dynamics become:

$$\begin{array}{rcl} e^+ &=& x^+ - \hat{x}^+ \\&=& Ax + Bu - A\hat{x} - Bu - L(y - \hat{y}) \\ &=& A(x - \hat{x}) - LC(x - \hat{x}) \\ &=& (A - LC)(x - \hat{x}) \\ e^+ &=& (A - LC)e \end{array}$$

Thus, the error dynamics are governed by the matrix $A-LC$, and the observer is stable if this matrix is asymptotically stable (i.e., all eigenvalues inside the unit circle).

The observer gain $L$ can be computed using the same techniques as the feedback gain $K$:
1. *Pole placement* (if system is controlable).
2. *Linear quadratic regulation* (LQR).
3. *Linear matrix inequalities* (LMI).

::: highlight
#icon("warning") **Warning:** As seen in the error dynamics, there is a duality between the closed-loop control system and the observer. In fact, the observer design is the transpose problem of state feedback design. This means that, when using functions originally intended for control synthesis, the following substitution holds:
$$(A, B, K) \rightarrow (A^\top, C^\top, L^\top)$$.
So you can compute $L$ using the same methods and tools as for $K$, by transposing the appropriate matrices.
:::

:::: exercise

**Exercise 5:**

Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Mimo/scripts/observer.py")

1. Implement one the method to compute the observer gain $L$
#runsofa-button("assets/labs/EmioLabs_Mimo/scripts/observer.py" "--order" "order")
2. Test the observer on Sofa to see how it can estimate the state of the system.
::::
:::::

::::: collapse Closed Loop Control

## Closed Loop Control

**Closed Loop Control.** In simulation, we have access to the full (reduced) state of the system. However, to better reflect realistic conditions, we simulate the control using an observer rather than the true state.

The control law becomes:

$$u = Gr - K\hat{x}$$

where $\hat{x}$ is the estimated state from the observer, and $r$ is the reference for the controlled output.

The simulation loop proceeds as follows:

- Initialize the estimated state $\hat{x}$.
- At each time step:
  1. Measure the output y and estimate output $\hat{y} = C\hat{x}$.
  2. Compute the control input $u = Gr - K\hat{x}$.
  3. Update the observer using $\hat{x}^+ = A\hat{x} + Bu + L(y−\hat{y})$.


This setup will first be implemented in SOFA, which provides the system dynamics and sensor measurements. It enables evaluation of the full closed-loop system without requiring physical hardware, while still respecting realistic sensing and estimation constraints.

#runsofa-button("assets/labs/EmioLabs_Mimo/lab_mimo.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorsInit" "motor1Init" "motor2Init" "--motorsMin" "motor1Min" "motor2Min" "--motorsMax" "motor1Max" "motor2Max" "--order" "order" "--useObserver" "1")

In a second step, the same control law can be deployed on the real robot. The observer will estimate the state based on measured outputs (e.g., marker positions), and the computed control input $u$ will be applied to the motors.

#runsofa-button("assets/labs/EmioLabs_Mimo/lab_mimo.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorsInit" "motor1Init" "motor2Init" "--motorsMin" "motor1Min" "motor2Min" "--motorsMax" "motor1Max" "motor2Max" "--order" "order" "--useObserver" "True" "--realRobot" "1")

::: highlight
#icon("warning") Insight:
This architecture enables real-time control using only partial measurements, and bridges the gap between model-based design and physical implementation. The exact same controller and observer gains designed in simulation can be reused on the real system, as long as the model accurately captures the dynamics.
:::
:::::
