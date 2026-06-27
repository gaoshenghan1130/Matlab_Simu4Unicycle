# Nonlinearity analysis

Given any nonlinear system, we can linearize it around a point and analyze the stability of the linearized system. However, the controller designed for the linearized system may not stabilize the original nonlinear system in all regions. Therefore, it is important to analyze the nonlinearity of the system and determine the regions where the linearized controller can stabilize the nonlinear system.

## Setup

Given any nonlinear system, it is of the form:

$$
\dot{z} = f(z, u), \quad z \in \mathbb{R}^n, u \in \mathbb{R}^m
$$

To simplify the analysis, we assume the force input is linear in the input $u$, so the system can be expressed as:

$$
\dot{z} = f(z) + g(z) u = f(z) + Bu
$$

With the linearization of the system around a point $z_0$, we have:

$$
\dot{z} = f(z_0) + \frac{\partial f}{\partial z} (z_0) (z - z_0) + B u = A (z - z_0) + B u
$$

For further simplification, we can assume $z_0 = \hat{0}$, so the linearized system becomes:

$$
\dot{z} = A z + B u
$$

## Linearized controller

Given some cost function $J(z, u)$, we can design a linearized controller for the linearized system. The controller is of the form:

$$
u = -K z
$$

And:

$$
\dot{z} = (A - BK) z
$$

Depend on the cost function definition, the $K$ will have different values, however, ususal assumption of cost function of the form: $J(z, u) = \int_0^\infty (z^T Q z + u^T R u) dt$ may lead to limitations on time resonse thus not be able to cover all the nonlinear system behavior. 

Therefore, here we assume K is any random combination of the form $K = [k_1, k_2, \ldots, k_n], \qquad k_i \in \mathbb{R}$.

## Nonlinearity analysis

### Discretization

Use MPC ideaology, we can assume the system to be discretized with a time step $\Delta t$, and the system can be expressed as:

$$
\begin{aligned}
z_{k+1} &= z_k + \Delta t f(z_k) + \Delta t B u_k \\
\implies \frac{z_{k+1}}{\Delta t} &= \frac{z_k}{\Delta t} + f(z_k) + B u_k \\
\implies \frac{z_{k+1}}{\Delta t} &= f(z_k) + \left( \frac{1}{\Delta t} I_n - BK \right) z_k
\end{aligned}
$$

The linearized system can be expressed as:

$$
\frac{z_{k+1}}{\Delta t} = \frac{z_k}{\Delta t} + A z_k + B u_k =\frac{z_k}{\Delta t} + \frac{\partial f}{\partial z} z_k + B u_k \\
\implies \frac{z_{k+1}}{\Delta t} = \left( \frac{1}{\Delta t} I_n + A - BK \right) z_k = \left( \frac{1}{\Delta t} I_n + \frac{\partial f}{\partial z} - BK \right) z_k
$$

Normally we do:

$$
\left( \frac{z_{k+1}}{\Delta t} \right)_{\text{nonlinear}} - \left( \frac{z_{k+1}}{\Delta t} \right)_{\text{linear}} = f(z_k) - \frac{\partial f}{\partial z} z_k
$$

This cancels out the input, and decide that the nonlinearity of the system is determined by the difference between the nonlinear function $f(z_k)$ and its linear approximation $\frac{\partial f}{\partial z} z_k$, and is independent of the input $u_k$. However, this is not true in general, as the input $u_k$ can also affect the nonlinearity of the system. Therefore, we need to consider the effect of the input on the nonlinearity of the system.

## Nonlinearity measure

### Target

Go back to the original nonlinear system:

$$
\dot{z} = f(z) + Bu
$$

And the linearized system:

$$
\dot{z} = A z + Bu
$$

In mathematical terms, we want to find a state $z_c$ such that under the linearized control law $u_L(z) = -Kz$, the true nonlinear system escapes the controllable zone $Z_{controllable}$ in the next time step $\Delta t$, while the linearized model incorrectly predicts that the state remains safe. Crucially, to prove that this point is truly a part of the performance boundary (and not just an inherently uncontrollable state), there must exist some alternative feasible control input $u_{nl}$ that successfully keeps the nonlinear system within the zone.This gives us three conditions for $z_c$:

The linear controller fails on the true nonlinear system($\mathcal{B_2}$):

$$
z_c + \Delta t f(z_c) - \Delta t BK z_c \notin Z_{controllable}
$$

The linear prediction model claims the state is safe ($\mathcal{B_1}$):
$$z_c + \Delta t A z_c - \Delta t BK z_c \in Z_{controllable}
$$

To be more precise:

$$
\text{Linear controller predicts safe} (\mathcal{B_1}) \\
 \text{Linear controller works actually}(\mathcal{B_2}) \\
\text{Linear controller doesn't even work}
$$

**Our goal is to find $\mathcal{B_1}$ and $\mathcal{B_2}$**.

### Set theorm

Writen in set theory $\mathcal{R} = Z_{controllable}$ , $\mathcal{C}$ is the ring of the set that the linear controller don't claim to be controllable, but the nonlinear system is actually controllable by this linear controller.

$$
\mathcal{C} = \left\{ z_c \in \mathcal{R} \; \middle| \; 
\begin{aligned}
& z_c + \Delta t \big(f(z_c) - BK z_c\big) \notin \mathcal{R} \\
\text{and } & z_c + \Delta t \big(A - BK\big) z_c \in \mathcal{R}
\end{aligned}
\right\}
$$

Or with boundary definition, $\mathcal{C}$ is in the middle of $\mathcal{B_1}$ and $\mathcal{B_2}$, where $\mathcal{B_1}$ is the boundary of the linearized controllable set, and $\mathcal{B_2}$ is the boundary of controllable set.


This way it actually assumes that $\mathcal{B_1}$ is a subset of $\mathcal{B_2}$, we will discuss this assumption later.

### Quantify the nonlinearity

First some simplifications:

$$
\mathcal{C} = \left\{ z_c \in \mathcal{R} \; \middle| \; 
\begin{aligned}
& H_1(z_c) \notin \mathcal{R} \\
\text{and } & H_2(z_c) \in \mathcal{R}
\end{aligned}
\right\}
$$

And our target is to find the expression of $G_1(z), G_2(z)$ with $H$ such that:

$$
\mathcal{C} = \left\{ z \in \mathbb{R}^n \; \middle| \; G_2(z) \ge 0, G_1(z) \le 0; \right \}
$$

By this definition, the actual linearized-controllable set is:

$$
\mathcal{C_l} = \left\{ z \in \mathbb{R}^n \; \middle| \; G_2(z) \le 0 \right \}
$$

As $\mathcal{C_l} \subseteq \mathcal{R}$ and $\mathcal{C_l}$ is a convex set, we can first calculate the boundary of $\mathcal{C_l}$, and then find the boundary of $\mathcal{C}$, and then find the difference between the two boundaries to find the nonlinearity of the system.

Given that at the boundary the tendency of the system must be tangent to the boundary, we can give the following definition of $G_1$ and $G_2$:

$$
\begin{aligned}
G_1(z) &= -\langle \nabla G(z), A_{cl}z \rangle \\
G_2(z) &= -\langle \nabla G(z), A_{cl}z + \phi(z) \rangle \\
\end{aligned}
$$

And $\mathcal{B_1} = \{ z \in \mathbb{R}^n \; | \; G_1(z) = 0 \}$, $\mathcal{B_2} = \{ z \in \mathbb{R}^n \; | \; G_2(z) = 0 \}$.

As:

$$
G_2(z) - G_1(z) = -\langle \nabla G(z), \phi(z) \rangle
$$

$\mathcal{C} = \emptyset$ if and only if $\langle \nabla G(z), \phi(z) \rangle \le 0$ for all $z \in \mathcal{R}$, which means that the linearized controller can stabilize the nonlinear system in the whole controllable set, and the nonlineaity of the system is actually helping with the stablization.

We will focus on the case where $\mathcal{C} \neq \emptyset$, as that will be most of the time we are interested in, and we will discuss the case where $\mathcal{C} = \emptyset$ later.

#### Linearized controllable set with boundary $\mathcal{B_1}$

Given: 

$$
-\langle \nabla G(z), A_{cl}z \rangle = 0
$$


Give $G(z) = z^T P z - 1$, we have $\nabla G(z) = 2 P z$, and the boundary of the linearized controllable set is:

$$
\mathcal{B_1} = \left\{ z \in \mathbb{R}^n \; \middle| \; \langle Pz, A_{cl}z \rangle = 0 \right\}
$$

The selection of the form of $G(z)$ is not unique, but by inexplicit function theorm, all the boundaries of the different forms of $G(z)$ are equivalent (actually they should be identical, as they only differ by a scaling on the speed of decay).

$$
\begin{aligned}
\mathcal{B}_1 &= \left\{ z \in \mathbb{R}^n \; \middle| \; \langle Pz, A_{cl}z \rangle = 0 \right\} \\
&= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T P A_{cl} z = 0 \right\} \\
&= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T (P A_{cl} + A_{cl}^T P) z = 0 \right\}
\end{aligned}
$$

This actually means that linearized controllers (and there algrithoms) will always assume it can control the entire $\mathbb{R}^n$ space. (If without any constraints).

#### Nonlinear controllable set with boundary $\mathcal{B_2}$

Given:

$$
-\langle \nabla G(z), A_{cl}z + \phi(z) \rangle = 0
$$



