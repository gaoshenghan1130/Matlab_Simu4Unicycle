# Nonlinearity analysis

Given any nonlinear system, we can linearize it around a point and analyze the stability of the linearized system. However, the controller designed for the linearized system may not stabilize the original nonlinear system in all regions. Therefore, it is important to analyze the nonlinearity of the system and determine the regions where the linearized controller can stabilize the nonlinear system.

[TOC]

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

## Quantify the nonlinearity

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

### Linearized controllable set with boundary $\mathcal{B_1}$

Given: 

$$
-\langle \nabla G(z), A_{cl}z \rangle = 0
$$


Given $G(z) = z^T P(z) z - 1$, the boundary condition is defined by the tendency of the system being tangent to the boundary surface. Instead of a static gradient, the directional derivative of $G(z)$ along the system dynamics $\dot{z}$ must be zero:$$\dot{G}_1(z) = \langle \nabla G(z), \dot{z}_{lin} \rangle = 0$$

By applying the product rule and chain rule to $G(z)$ with $\dot{z}_{lin} = A_{cl}z$, we have:$$\dot{G}_1(z) = \dot{z}_{lin}^T P(z) z + z^T P(z) \dot{z}_{lin} + z^T \dot{P}_{lin}(z) z = 0$$Where $\dot{P}_{lin}(z) = \sum_{i=1}^n \frac{\partial P(z)}{\partial z_i} (A_{cl} z)_i$ captures how the shape of the Lyapunov level set deforms along the linear trajectory.

The selection of the form of $G(z)$ is not unique, but by inexplicit function theorm, all the boundaries of the different forms of $G(z)$ are equivalent (actually they should be identical, as they only differ by a scaling on the speed of decay when P is independent of z).

$$
\begin{aligned}
\mathcal{B}_1 &= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T A_{cl}^T P(z) z + z^T P(z) A_{cl} z + z^T \dot{P}_{lin}(z) z = 0 \right\} \\
&= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T \big(P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \big) z = 0 \right\}
\end{aligned}
$$

This actually means that linearized controllers (and there algrithoms) will always assume it can control the entire $\mathbb{R}^n$ space. (If without any constraints). $P$ don't matter in this case, as there region will always be $\mathbb{R}^n$.

### Nonlinear controllable set with boundary $\mathcal{B_2}$
For the true nonlinear system, the dynamics are $\dot{z}_{nl} = A_{cl}z + \phi(z)$. The rate of change of the $P$ matrix along this true trajectory is $\dot{P}_{nl}(z) = \sum_{i=1}^n \frac{\partial P(z)}{\partial z_i} \dot{z}_{nl, i}$.Given the boundary condition:

$$\dot{G}_2(z) = \langle \nabla G(z), A_{cl}z + \phi(z) \rangle = 0$$

Similarly, we expand this using the state-dependent $P(z)$:

$$\begin{aligned}
\mathcal{B}_2 &= \left\{ z \in \mathbb{R}^n \; \middle| \; \dot{z}_{nl}^T P(z) z + z^T P(z) \dot{z}_{nl} + z^T \dot{P}_{nl}(z) z = 0 \right\} \\
&= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T \big( P(z)(A_{cl}z + \phi(z)) + (A_{cl}z + \phi(z))^T P(z) \big) + z^T \dot{P}_{nl}(z) z = 0 \right\} \\
&= \left\{ z \in \mathbb{R}^n \; \middle| \; z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z + 2 z^T P(z) \phi(z) = 0 \right\}
\end{aligned}$$

Intrinsically, $z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z \leq 0$ represents the stability of the linear baseline (now correctly penalized by boundary deformation), and $2 z^T P(z) \phi(z)$ is the nonlinearity of the system that contributes to the difference. To find the maximal boundary exactly, this equation must be satisfied by a valid $P(z)$ topology, making the definition:

$$\mathcal{B}_2 = \left\{ z \in \mathbb{R}^n \; \middle| \; \max_{P(z)} \left[ z^T \big( P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{nl}(z) \big) z + 2 z^T P(z) \phi(z) \right] = 0 \right\}
$$

Solving the boundary $\mathcal{B}_2$: Notice that $z^T M z = \text{Tr}(M z z^T)$. We can separate the static matrix variables and the derivative constraints:

$$
\begin{aligned}
& z^T \big( P(z) A_{cl} + A_{cl}^T P(z) \big) z + 2 z^T P(z) \phi(z) + z^T \dot{P}_{nl}(z) z \\
&= \text{Tr}\big( P(z) \cdot \underbrace{[A_{cl} z z^T + z z^T A_{cl}^T + \phi(z) z^T + z \phi(z)^T]}_{M(z)} \big) + \text{Tr}\big( \dot{P}_{nl}(z) z z^T \big)
\end{aligned}
$$

So the boundary reduces to solving for a matrix function $P(z)$ such that:

$$\max_{P(z)} \left[ \text{Tr}\big( P(z) \cdot M(z) \big) + \text{Tr}\big( \dot{P}_{nl}(z) z z^T \big) \right] = 0$$

Here, $\text{Tr}\big( \dot{P}_{nl}(z) z z^T \big)$ serves as the mathematical "deformation penalty". It prevents the optimization from arbitrarily changing $P(z)$ to enclose unstable vectors by forcing the boundary's shape to evolve consistently with the system dynamics.The condition for the matrix function $P(z)$ over the domain is:

$$
P(z) \succ 0, \\
\quad P(z) A_{cl} + A_{cl}^T P(z) + \dot{P}_{lin}(z) \prec 0, \quad \text{(stable linear baseline)} \\
 \quad - \big( P(z) \cdot M(z) + \dot{P}_{nl}(z) z z^T \big) \succeq 0, \quad (\text{from } \dot{V}(x) \le 0)
$$
