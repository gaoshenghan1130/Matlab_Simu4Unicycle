# Controller design with coupleness analysis

Given a nonlinear system:

$$
\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^m
$$

When some variables are coupled, we need the controller to first focus on controlling the strongest coupled value, and then control the weaker coupled value.

Based on this philosophy, and with the definition of coupleness matrix:


$$
\mathcal{C}^z(z) = \gamma A(z) + \gamma^2 A^2(z) + \gamma^3 A^3(z) + \cdots \\
\implies \mathcal{C}^z(z) = (I - \gamma A(z))^{-1} - I
$$

$$
\mathcal{C}^u(z) = \mathcal{C}^z(z) \cdot B(z)
$$

Where:

$$
\rho(\gamma A(z)) < 1 \\
A(z) =  \frac{\partial f}{\partial z}(z) , \quad B(z) =  \frac{\partial f}{\partial u}(z) 
$$

In other words, 

$$
z_{\pi_1} \longrightarrow z_{\pi_2} \longrightarrow \cdots \longrightarrow z_{\pi_n}
$$

Where $\pi_i$ is the index of the $i$-th strongest coupled state, and $\pi_1$ is the strongest coupled state.

## Controller design:

The first step for the controller is to identify with state should be controlled at this point. The strongest controller may not be the one with the largest value of difference, so in mathmatical form we can give a evaluation function to find the strongest coupled state:

$$
\pi_i = \arg\max_{k \notin \{\pi_{<i}\}} \sum_{j=1}^n \left| \mathcal{C}^z_{kj}(z) \right| \cdot G(|z_j - z_j^{ref}|)
$$

where $G(\cdot)$ must be increasing function, and $z_j^{ref}$ is the reference value of the $j$-th state.

### Step 1: Find the strongest coupled state

In fact, after finding the strongest coupled state, for an underactuated system, we only need to find the number of the coupled states to be controlled to be the dimension of the actuator($m\le n$ is the dimension of the actuator).

$$\Omega = \{\pi_1, \pi_2, \dots, \pi_m\}$$

### Step 2: Design the controller

Extract the sub-matrix of the coupleness matrix:

$$
\mathcal{C}^u_{\Omega,m}(z) = \begin{bmatrix} \text{Row } \pi_1 \text{ of } \mathcal{C}^u \\ \text{Row } \pi_2 \text{ of } \mathcal{C}^u \\ \vdots \\ \text{Row } \pi_m \text{ of } \mathcal{C}^u \end{bmatrix}
$$

### Step 3: Solve the controller

Let difference between the current state and the reference state be:

$$
e_{\Omega} = z_{\Omega} - z_{\Omega}^{\text{ref}}
$$

So generally, the controller can be designed as:

$$
u = \Phi \Big( \mathcal{C}^u(z), \, \mathcal{C}^z(z), \, \mathbf{v}(e, \dot{e}, \dots) \Big)
$$

For instance:

$$
u = \left( \mathcal{C}^u_{\Omega,m}(z) \right)^{-1} \mathbf{v}(e_{\Omega}, \dot{e}_{\Omega})
$$






